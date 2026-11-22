import re,sys,numpy as np,csv
fn=sys.argv[1]
runs=[];cur=None;raw=None
for line in open(fn,errors='ignore'):
    line=line.strip().replace('\x00','')
    m=re.search(r'# command: velocity .*raw (-?\d+)',line)
    if m: raw=int(m.group(1)); continue
    if line.startswith('# exp 3 start'): cur={'raw':raw,'rows':[]}; continue
    if line.startswith('# exp 3 end'):
        if cur: runs.append(cur); cur=None
        continue
    m=re.search(r'status: volt=([\d.]+) V temp=(\d+)',line)
    if m and runs and 'temp_end' not in runs[-1]: runs[-1]['temp_end']=int(m.group(2))
    if cur is not None and re.match(r'^3,\d',line):
        p=line.split(',')
        if len(p)==13: cur['rows'].append([float(x) for x in p])
out=[]
for k,r in enumerate(runs):
    a=np.array(r['rows']); 
    if len(a)==0: continue
    ok=a[:,12]==0
    w=ok&(a[:,1]>=2000)&(a[:,1]<4000)
    t=a[w,1]/1000; pos=a[w,6]
    slope=np.polyfit(t,pos,1)[0] if w.sum()>10 else np.nan  # counts/s
    rpm_pos=slope*60/4096
    pwm=a[w,3]; 
    # stick-slip indicator: std of velocity / mean; fraction of samples with vel==0
    vel=a[w,5]
    out.append(dict(run=k+1,raw=r['raw'],rpm_cmd=r['raw']*0.229,rpm_pos=rpm_pos,
        rpm_vel=vel.mean()*0.229,pwm_pct=pwm.mean()/885*100,pwm_std=pwm.std()/885*100,
        cur_mA=a[w,4].mean(),volt=a[w,10].mean()/10,temp=a[w,11].mean(),
        zero_vel_frac=(vel==0).mean(),n=int(w.sum()),n_err=int((~ok).sum())))
with open('velA_runs.csv','w',newline='') as f:
    wr=csv.DictWriter(f,fieldnames=list(out[0].keys())); wr.writeheader(); wr.writerows(out)
for o in out: print({k:(round(v,3) if isinstance(v,float) else v) for k,v in o.items()})
