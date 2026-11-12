#!/usr/bin/env node
'use strict';

// CamNode test bridge
// Camera (ESP32-S3) --TCP:8899--> bridge --HTTP:8080--> browser
//
// From camera:
//   ['C','A','M','0'][uint32 BE len][JPEG]          frame
//   ['C','A','M','T'][uint32 BE len][telemetry 16B] telemetry
// To camera:
//   ['C','M','D'][uint8 code][uint8 len][payload]   command
//   'H' (1 byte)                                    heartbeat

const net  = require('net');
const http = require('http');
const fs   = require('fs');
const path = require('path');
const { WebSocketServer } = require('ws');

const TCP_PORT   = 8899;
const HTTP_PORT  = 8080;
const HB_MS      = 15000;
const BOUNDARY   = 'camnodeframe';
const PAGE       = path.join(__dirname, 'index.html');

let camSock    = null;
let lastFrame  = null;
let telemetry  = null;

const viewers   = new Set();   // MJPEG HTTP responses
const wsClients = new Set();   // browser WebSockets

const stat = { frames: 0, bytes: 0, t0: 0, fps: 0, mbit: 0, avgKB: 0 };

const mqtt = require('mqtt');

const MQTT_URL  = 'mqtt://url';
const MQTT_USER = 'user';
const MQTT_PASS = 'passwd';

const mq = mqtt.connect(MQTT_URL, {
  username: MQTT_USER, password: MQTT_PASS, clientId: 'camnode-bridge',
});

mq.on('connect', () => {
  log('I', 'MQTT', 'Broker connected');
  mq.subscribe('camera/01/state/#');
  mq.subscribe('camera/01/evt/#');
});

mq.on('message', (topic, buf) => {
  const s = buf.toString();
  log('I', 'MQTT', `${topic} = ${s}`);

  if (topic === 'camera/01/state/status') {
    try { broadcast({ type:'telemetry', data: JSON.parse(s) }); } catch {}
  }
  broadcast({ type:'mqtt', topic, payload: s });
});

function log(level, tag, text) {
  const line = { level, tag, text, t: Date.now() };
  const stamp = new Date().toISOString().slice(11, 23);
  console.log(`${level} (${stamp}) ${tag}: ${text}`);
  broadcast({ type: 'log', line });
}

function broadcast(obj) {
  const s = JSON.stringify(obj);
  for (const ws of wsClients) {
    if (ws.readyState === 1) ws.send(s);
  }
}

function parseTelemetry(b) {
  if (b.length < 16) return null;
  return {
    battery_mv: b.readUInt16LE(0),
    solar_mv:   b.readUInt16LE(2),
    state:      b.readUInt8(4),
    pan:        b.readInt16LE(5),
    tilt:       b.readInt16LE(7),
    pir_count:  b.readUInt16LE(9),
    rssi:       b.readInt8(11),
    uptime_s:   b.readUInt32LE(12),
  };
}

function pushFrame(jpeg) {
  lastFrame = jpeg;

  if (stat.t0 === 0) stat.t0 = Date.now();
  stat.frames++;
  stat.bytes += jpeg.length;

  const dt = Date.now() - stat.t0;
  if (dt > 0) {
    stat.fps   = +(stat.frames * 1000 / dt).toFixed(1);
    stat.mbit  = +(stat.bytes * 8 / dt / 1000).toFixed(2);
    stat.avgKB = +(stat.bytes / stat.frames / 1024).toFixed(1);
  }

  const head = Buffer.from(
    `\r\n--${BOUNDARY}\r\n` +
    `Content-Type: image/jpeg\r\n` +
    `Content-Length: ${jpeg.length}\r\n\r\n`
  );

  for (const res of viewers) {
    // Drop frame for slow clients instead of queueing
    if (res.writableLength > 512 * 1024) continue;
    res.write(head);
    res.write(jpeg);
  }
}

const tcp = net.createServer((sock) => {
  // New connection replaces old one: previous session is dead (reboot without FIN)
  if (camSock) {
    log('W', 'BRIDGE', 'New camera, dropping previous connection');
    camSock.destroy();
    camSock = null;
  }

  camSock = sock;
  sock.setNoDelay(true);

  // Aggressive keepalive: detect dead socket in ~15 s instead of Linux default 2 h
  sock.setKeepAlive(true, 5000);
  sock.setTimeout(20000);
  sock.on('timeout', () => {
    log('W', 'BRIDGE', 'Camera silent for 20 s, disconnecting');
    sock.destroy();
  });
  Object.assign(stat, { frames: 0, bytes: 0, t0: 0, fps: 0, mbit: 0, avgKB: 0 });
  log('I', 'BRIDGE', `Camera connected: ${sock.remoteAddress}`);
  broadcast({ type: 'link', camera: true });

  const hb = setInterval(() => {
    if (sock.writable) sock.write(Buffer.from('H'));
  }, HB_MS);

  let buf = Buffer.alloc(0);

  sock.on('data', (chunk) => {
    buf = Buffer.concat([buf, chunk]);

    for (;;) {
      if (buf.length < 8) break;

      const magic = buf.toString('ascii', 0, 4);
      if (magic !== 'CAM0' && magic !== 'CAMT') {
        buf = buf.subarray(1);          // resync
        continue;
      }

      const len = buf.readUInt32BE(4);
      if (len > 4 * 1024 * 1024) {      // garbage guard
        log('E', 'BRIDGE', `Suspicious length ${len}, resyncing`);
        buf = buf.subarray(1);
        continue;
      }
      if (buf.length < 8 + len) break;

      const payload = buf.subarray(8, 8 + len);
      buf = buf.subarray(8 + len);

      if (magic === 'CAM0') {
        pushFrame(Buffer.from(payload));
      } else {
        telemetry = parseTelemetry(payload);
        if (telemetry) broadcast({ type: 'telemetry', data: telemetry });
      }
    }
  });

  let closed = false;
  const done = () => {
    if (closed) return;             // close may fire twice
    closed = true;
    clearInterval(hb);
    if (camSock === sock) {         // skip if a new camera already connected
      camSock = null;
      log('W', 'BRIDGE', 'Camera disconnected');
      broadcast({ type: 'link', camera: false });
    }
  };

  sock.on('close', done);
  sock.on('error', (e) => log('E', 'BRIDGE', `Camera socket: ${e.message}`));
});

function sendCommand(code, payload = Buffer.alloc(0)) {
  if (!camSock || !camSock.writable) return false;
  const pkt = Buffer.concat([
    Buffer.from([0x43, 0x4d, 0x44, code, payload.length]),  // 'C','M','D'
    payload,
  ]);
  camSock.write(pkt);
  return true;
}

const CMD = {
  START_STREAM: 0x01,
  STOP_STREAM:  0x02,
  SERVO_PAN:    0x03,
  SERVO_TILT:   0x04,
  GET_STATUS:   0x05,
  SET_PARAMS:   0x06,
  SERVO_MOVE:   0x07,
  SERVO_VEL:    0x08,
  SERVO_CAL:    0x09,
  SERVO_HOME:   0x0a,
};

const i8 = (v) => Math.max(-100, Math.min(100, v | 0));

const httpSrv = http.createServer((req, res) => {
  const url = req.url.split('?')[0];

  if (url === '/' || url === '/index.html') {
    fs.readFile(PAGE, (err, data) => {
      if (err) {
        res.writeHead(500, { 'Content-Type': 'text/plain' });
        res.end('index.html not found next to bridge.js');
        return;
      }
      res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
      res.end(data);
    });
    return;
  }

  if (url === '/stream') {
    res.writeHead(200, {
      'Content-Type': `multipart/x-mixed-replace; boundary=${BOUNDARY}`,
      'Cache-Control': 'no-store, no-cache, must-revalidate',
      'Pragma': 'no-cache',
      'Connection': 'close',
    });
    viewers.add(res);
    if (lastFrame) pushFrame(lastFrame);
    req.on('close', () => viewers.delete(res));
    return;
  }

  if (url === '/snapshot') {
    if (!lastFrame) {
      res.writeHead(503, { 'Content-Type': 'text/plain' });
      res.end('No frames yet');
      return;
    }
    res.writeHead(200, {
      'Content-Type': 'image/jpeg',
      'Content-Length': lastFrame.length,
      'Cache-Control': 'no-store',
    });
    res.end(lastFrame);
    return;
  }

  res.writeHead(404, { 'Content-Type': 'text/plain' });
  res.end('404');
});

const wss = new WebSocketServer({ server: httpSrv });

wss.on('connection', (ws) => {
  wsClients.add(ws);
  ws.send(JSON.stringify({
    type: 'hello',
    camera: !!camSock,
    telemetry,
    stat,
  }));

  ws.on('message', (raw) => {
    let msg;
    try { msg = JSON.parse(raw.toString()); } catch { return; }

    let ok = false;
    switch (msg.cmd) {
      case 'start':
        mq.publish('camera/01/cmd/stream', 'START', { qos: 1 });
        ok = true;
        break;
      case 'stop':
        mq.publish('camera/01/cmd/stream', 'STOP', { qos: 1 });
        ok = true;
        break;
      case 'status':
        mq.publish('camera/01/cmd/stream', 'STATUS', { qos: 1 });
        ok = true;
        break;
      case 'params':
        mq.publish('camera/01/cmd/config',
            JSON.stringify({ resolution: msg.resolution, quality: msg.quality }),
            { qos: 1 });
        ok = true;
        break;
      case 'pan': {
        const b = Buffer.alloc(2);
        b.writeInt16LE(msg.value | 0);
        ok = sendCommand(CMD.SERVO_PAN, b);
        break;
      }
      case 'tilt': {
        const b = Buffer.alloc(2);
        b.writeInt16LE(msg.value | 0);
        ok = sendCommand(CMD.SERVO_TILT, b);
        break;
      }
      case 'move': {
        const b = Buffer.alloc(4);
        b.writeInt16LE(msg.pan  | 0, 0);
        b.writeInt16LE(msg.tilt | 0, 2);
        ok = sendCommand(CMD.SERVO_MOVE, b);
        break;
      }
      case 'vel': {                      // joystick mode
        const b = Buffer.alloc(2);
        b.writeInt8(i8(msg.vx), 0);
        b.writeInt8(i8(msg.vy), 1);
        ok = sendCommand(CMD.SERVO_VEL, b);
        break;
      }
      case 'home':
        ok = sendCommand(CMD.SERVO_HOME);
        break;
      default:
        return;
    }

    if (!ok && msg.cmd !== 'vel') {
      log('W', 'BRIDGE', `Command "${msg.cmd}" not sent: no camera`);
    }
  });

  ws.on('close', () => wsClients.delete(ws));
});

setInterval(() => {
  broadcast({ type: 'stat', stat, viewers: viewers.size });
}, 1000);

tcp.listen(TCP_PORT, () => log('I', 'BRIDGE', `Camera TCP on :${TCP_PORT}`));
httpSrv.listen(HTTP_PORT, () => log('I', 'BRIDGE', `HTTP and WS on :${HTTP_PORT}`));
