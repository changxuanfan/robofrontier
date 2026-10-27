// Codex CLI app-server transport. No inference until an explicit turn/start.
const {spawn} = require('node:child_process');
const readline = require('node:readline');

class RpcError extends Error {
  constructor(value) { super(value.message); this.detail = value; }
}
class CodexRpc {
  constructor(args = [], {stderr = 'ignore', onEvent = () => {}, onRequest} = {}) {
    this.next = 0; this.pending = new Map(); this.onEvent = onEvent;
    this.process = spawn('codex', ['app-server', ...args, '--listen', 'stdio://'],
      {stdio:['pipe','pipe',stderr]});
    const fail = error => {
      for (const {reject,timer} of this.pending.values()) { clearTimeout(timer); reject(error); }
      this.pending.clear(); this.closed = true;
    };
    this.process.on('error', fail);
    this.process.on('exit', code => fail(new Error('Codex app-server exited: '+code)));
    readline.createInterface({input:this.process.stdout}).on('line', async line => {
      let message;
      try { message = JSON.parse(line); } catch { return; }
      if (message.method && message.id !== undefined) {
        try {
          if (!onRequest) throw new Error('Unsupported server request: '+message.method);
          const result = await onRequest(message.method, message.params);
          this.write({id:message.id,result});
        } catch (error) {
          this.write({id:message.id,error:{code:-32603,message:String(error.message)}});
        }
      } else if (message.id !== undefined) {
        const p = this.pending.get(message.id); if (!p) return;
        clearTimeout(p.timer); this.pending.delete(message.id);
        if (message.error) p.reject(new RpcError(message.error)); else p.resolve(message.result);
      } else this.onEvent(message);
    });
  }
  write(value) {
    if (!this.closed) this.process.stdin.write(JSON.stringify(value)+'\n');
  }
  call(method, params = {}, timeoutMs = 30000) {
    if (this.closed) return Promise.reject(new Error('Codex connection closed'));
    const id = ++this.next;
    return new Promise((resolve,reject) => {
      const timer = setTimeout(() => {
        this.pending.delete(id);
        reject(new Error(method+' acknowledgement unknown; do not repeat mutations'));
      }, timeoutMs);
      this.pending.set(id,{resolve,reject,timer}); this.write({id,method,params});
    });
  }
  async initialize() {
    const result = await this.call('initialize',{
      clientInfo:{name:'robofrontier_cli',title:'RoboFrontier benchmark',version:'1.0'},
      capabilities:{experimentalApi:true}});
    this.write({method:'initialized'}); return result;
  }
  async close() {
    if (this.closed) return;
    const exited = new Promise(resolve => this.process.once('exit',resolve));
    this.process.kill('SIGTERM');
    const timer = setTimeout(() => this.process.kill('SIGKILL'),3000);
    await exited; clearTimeout(timer);
  }
}
function capacityFailure(error) {
  const text = typeof error === 'string' ? error : JSON.stringify(error);
  return /model[_ ]?capacity|capacity[_ ]?limit|rate[_ ]?limit|too many requests|\b429\b|overloaded|server[_ ]?overloaded|insufficient[_ ]?capacity/i.test(text)
    && !/insufficient[_ ]quota|billing|invalid[_ ]api[_ ]key/i.test(text);
}
module.exports = {CodexRpc,RpcError,capacityFailure};
