import csv, math, numpy as np, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
for n in ['Noto Sans CJK TC','Noto Sans CJK JP','WenQuanYi Zen Hei']:
    if any(n in f.name for f in fm.fontManager.ttflist): plt.rcParams['font.family']=n; break
plt.rcParams['axes.unicode_minus']=False
Rs=3.2
rows=[r for r in csv.DictReader(open('data_L_20261119.csv')) if 'before averaging' not in r['note'] and float(r['f_Hz'])<60e3]
f=np.array([float(r['f_Hz']) for r in rows]); V1=np.array([float(r['V1_rms_mV']) for r in rows]); V2=np.array([float(r['V2_rms_mV']) for r in rows]); ph=np.radians([float(r['phase12_deg']) for r in rows])
K=182/180  # probe gain CH2/CH1 measured with both tips on same node
Z=V2/(V1/Rs)/K; w=2*np.pi*f
L=Z*np.sin(ph)/w*1e6; R=Z*np.cos(ph)-Rs
dL=Z*np.cos(ph)*math.radians(1)/w*1e6; dR=Z*np.sin(ph)*math.radians(1)
o=np.argsort(f); f,L,R,dL,dR=f[o],L[o],R[o],dL[o],dR[o]
for a,b,c in zip(f,L,R): print(f'{a:9.0f} Hz  L={b:6.1f} uH  R={c:6.2f} ohm')
fig,ax=plt.subplots(1,2,figsize=(11,4.3))
ax[0].errorbar(f/1e3,L,yerr=dL,fmt='o-',color='#185FA5',capsize=3)
ax[0].axhline(50.3,color='#A32D2D',ls='--'); ax[0].text(0.55,62,'學長 50.3 µH',color='#A32D2D')
ax[0].set_xscale('log'); ax[0].set_ylim(0,700); ax[0].set_xlabel('頻率 (kHz)'); ax[0].set_ylabel('L (µH)')
ax[0].set_title('馬達電感 L(f)，轉子位置 1'); ax[0].grid(alpha=.3,which='both')
ax[1].errorbar(f/1e3,R,yerr=dR,fmt='s-',color='#993C1D',capsize=3)
ax[1].axhline(2.55,color='gray',ls='--'); ax[1].text(5,2.15,'直流 4 線量測 2.55 Ω',color='gray')
ax[1].set_xscale('log'); ax[1].set_yscale('log'); ax[1].set_xlabel('頻率 (kHz)'); ax[1].set_ylabel('R_m (Ω)')
ax[1].set_title('馬達等效電阻 R_m(f)'); ax[1].grid(alpha=.3,which='both')
from matplotlib.ticker import ScalarFormatter
for a in ax: a.xaxis.set_major_formatter(ScalarFormatter())
ax[1].yaxis.set_major_formatter(ScalarFormatter())
fig.text(0.5,-0.02,'AFG 4 Vpp、R_s = 3.2 Ω、平均 16 次；誤差棒 = 相位 ±1°；未含 100 kHz（已呈電容性）及已修正探棒增益差 1.1%（CH2/CH1 = 1.011）',ha='center',fontsize=9)
fig.tight_layout(); fig.savefig('L_R_vs_f_1119.png',dpi=160,bbox_inches='tight')
