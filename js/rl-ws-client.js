/**
 * js/rl-ws-client.js — WebSocket client connecting browser simulator to Python RL trainer
 *
 * Protocol:
 *   Python → Browser: { type: 'reset', seed: N } or { type: 'step', action: {steering, throttle, brake} }
 *   Browser → Python: { type: 'obs', observation: [...], reward: N, done: bool, truncated: bool, info: {...} }
 */

export class RLWebSocketClient {
  constructor(env, url = 'ws://localhost:8765') {
    this.env = env;
    this.url = url;
    this.ws = null;
    this.connected = false;
    this.messageCount = 0;
    this.onStatusChange = null;
  }

  connect() {
    return new Promise((resolve, reject) => {
      try {
        this.ws = new WebSocket(this.url);
      } catch (e) {
        reject(e);
        return;
      }

      this.ws.onopen = () => {
        this.connected = true;
        console.log('[RL-WS] Connected to trainer at', this.url);
        if (this.onStatusChange) this.onStatusChange('connected');

        // Send environment info
        this.ws.send(JSON.stringify({
          type: 'env_info',
          observationSize: this.env.currentObs ? this.env.currentObs.length : 97,
          actionSize: 3,
          actionNames: ['steering', 'throttle', 'brake'],
          actionLow: [-1, 0, 0],
          actionHigh: [1, 1, 1]
        }));

        resolve();
      };

      this.ws.onmessage = (event) => {
        this.messageCount++;
        try {
          const msg = JSON.parse(event.data);
          this._handleMessage(msg);
        } catch (e) {
          console.error('[RL-WS] Parse error:', e);
        }
      };

      this.ws.onclose = () => {
        this.connected = false;
        console.log('[RL-WS] Disconnected');
        if (this.onStatusChange) this.onStatusChange('disconnected');
      };

      this.ws.onerror = (e) => {
        console.error('[RL-WS] Error:', e);
        if (this.onStatusChange) this.onStatusChange('error');
        reject(e);
      };
    });
  }

  disconnect() {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
      this.connected = false;
    }
  }

  _handleMessage(msg) {
    switch (msg.type) {
      case 'reset': {
        const obs = this.env.reset(msg.seed ?? undefined);
        this._send({
          type: 'obs',
          observation: Array.from(obs),
          reward: 0,
          done: false,
          truncated: false,
          info: { step: 0 }
        });
        break;
      }

      case 'step': {
        const result = this.env.step(msg.action);
        this._send({
          type: 'obs',
          observation: Array.from(result.observation),
          reward: result.reward,
          done: result.done,
          truncated: result.truncated,
          info: result.info
        });
        break;
      }

      case 'get_metrics': {
        this._send({
          type: 'metrics',
          data: this.env.getMetrics().toJSON()
        });
        break;
      }

      case 'set_curriculum': {
        this.env.curriculumStage = msg.stage ?? 1;
        this._send({ type: 'ack', msg: `curriculum set to stage ${msg.stage}` });
        break;
      }

      case 'set_v2v': {
        this.env.v2vEnabled = !!msg.enabled;
        this._send({ type: 'ack', msg: `v2v ${msg.enabled ? 'enabled' : 'disabled'}` });
        break;
      }

      case 'ping': {
        this._send({ type: 'pong', messageCount: this.messageCount });
        break;
      }

      default:
        console.warn('[RL-WS] Unknown message type:', msg.type);
    }
  }

  _send(data) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(data));
    }
  }
}
