import pandas as pd,numpy as np,matplotlib
matplotlib.use('Agg');import matplotlib.pyplot as plt
plt.rcParams['font.family']='Noto Sans CJK TC';plt.rcParams['axes.unicode_minus']=False
d=pd.read_csv('velA_runs.csv');V=4.93;Ra=2.55
d['w']=d.rpm_pos*2*np.pi/60
d['nu']=d.pwm_pct/100*np.where(d.raw.abs()==445,d.volt,V)
m=d[d.raw.abs()!=445]
A=np.vstack([np.sign(m.w),m.w]).T;(c0,s),_,_,_=np.linalg.lstsq(A,m.nu,rcond=None)
fig,ax=plt.subplots(1,3,figsize=(15,4.4))
for sg,col,lab in [(1,'#185FA5','正轉'),(-1,'#993C1D','反轉')]:
    q=d[np.sign(d.raw)==sg]
    ax[0].plot(q.w,q.nu,'o',color=col,ms=4,label=lab)
    ax[1].plot(q.w.abs(),(q.nu-s*q.w)*1000*sg,'o',color=col,ms=4,label=lab)
    ax[2].plot(q.rpm_pos.abs(),q.cur_mA.abs(),'o',color=col,ms=4,label=lab)
x=np.linspace(-11,11,10);ax[0].plot(x,s*x,'k--',lw=1,label=f'斜率 {s:.3f} V·s/rad')
ax[0].set_xlabel('ω 輸出端 (rad/s)');ax[0].set_ylabel('ν0 = d·V_bus (V)');ax[0].set_title('維持等速所需電壓（空力臂、水平）');ax[0].legend(fontsize=8);ax[0].grid(alpha=.3)
ax[1].axhline(0,color='k',lw=.8);ax[1].axhline(58,color='gray',ls=':');ax[1].text(0.03,61,'學長 10% duty 時端電壓多出 58 mV',fontsize=8,color='gray')
ax[1].set_xscale('log');ax[1].set_xlim(0.02,3);ax[1].set_ylim(-30,75)
ax[1].set_xlabel('|ω| (rad/s)');ax[1].set_ylabel('|ν0| − 斜率·|ω| (mV)');ax[1].set_title('扣掉直線後的剩餘電壓（＝摩擦訊號）');ax[1].legend(fontsize=8);ax[1].grid(alpha=.3,which='both')
ax[2].set_xscale('log');ax[2].set_xlabel('|轉速| (rpm)');ax[2].set_ylabel('|回報電流| (mA)');ax[2].set_title('回報電流（只作比較）');ax[2].grid(alpha=.3,which='both');ax[2].legend(fontsize=8)
fig.text(0.5,-0.03,'標籤 XL330、5 V 供電（V_bus 暫用 4.93 V）、Mode 1 速度控制、每筆取命令後 1–3 s；速度由位置斜率計算；3 輪共 42 筆＋1 筆 102 rpm',ha='center',fontsize=9)
fig.tight_layout();fig.savefig('velA_1122.png',dpi=150,bbox_inches='tight')
print('slope',s,'sign-offset',c0)
