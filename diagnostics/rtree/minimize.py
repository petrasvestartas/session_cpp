import argparse,json,subprocess,math
from pathlib import Path
parser=argparse.ArgumentParser()
parser.add_argument("--exe",required=True)
parser.add_argument("--target",type=int,choices=[394,395],default=394)
parser.add_argument("--output",type=Path,required=True)
a=parser.parse_args()
a.output.mkdir(parents=True,exist_ok=True)
mate=31 if a.target==394 else 42
input_path=a.output/"current-indices.txt"
cache={}
def reproduces(order):
 key=tuple(order)
 if key in cache:return cache[key]
 input_path.write_text("\n".join(map(str,order))+"\n")
 result=subprocess.run([a.exe,"--target",str(a.target),"--indices",str(input_path)],capture_output=True,text=True)
 if result.returncode not in [0,1]:raise RuntimeError((result.returncode,result.stdout,result.stderr))
 data=json.loads(result.stdout)
 bad=any(r["query"]==a.target and mate in r["expected"] and mate not in r["found"] for r in data["mismatches"])
 with (a.output/"trials.jsonl").open("a") as log:log.write(json.dumps({"indices":order,"reproduces":bad,"result":data})+"\n")
 cache[key]=bad
 return bad
order=list(range(a.target+1))
if not reproduces(order):
 (a.output/"result.json").write_text(json.dumps({"reproduced":False,"target":a.target,"mate":mate}))
 raise SystemExit(0)
granularity=2
while True:
 removable=[i for i in order if i not in [mate,a.target]]
 if not removable:break
 chunk=math.ceil(len(removable)/granularity)
 reduced=False
 for start in range(0,len(removable),chunk):
  dropped=set(removable[start:start+chunk])
  proposal=[i for i in order if i not in dropped]
  if reproduces(proposal):
   order=proposal;granularity=max(2,granularity-1);reduced=True;break
 if reduced:continue
 if granularity>=len(removable):break
 granularity=min(len(removable),granularity*2)
(a.output/"minimal-indices.txt").write_text("\n".join(map(str,order))+"\n")
(a.output/"result.json").write_text(json.dumps({"reproduced":True,"target":a.target,"mate":mate,"indices":order,"trials":len(cache)},indent=2))
print(json.dumps({"insertions":len(order),"indices":order,"trials":len(cache)}))
