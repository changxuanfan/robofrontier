// Initialize Codex state without starting a model turn or making inference requests.
const {CodexRpc}=require('./codex_rpc');
async function main() {
  const rpc=new CodexRpc(process.argv.slice(2),{stderr:'inherit'});
  try { await rpc.initialize(); } finally { await rpc.close(); }
}
main().catch(error=>{console.error('Codex initialization failed:',error.message);process.exitCode=1;});
