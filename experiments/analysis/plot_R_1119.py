"""四線法量馬達線圈電阻（2026-11-19，拆開的小馬達，單一轉子位置，軸固定）
用法：python3 plot_R_1119.py data_R_20261119.csv out.png"""
import sys, glob
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
for fp in glob.glob("/usr/share/fonts/**/Noto*CJK*Regular*.tt*", recursive=True): fm.fontManager.addfont(fp)
for f in ["PingFang TC","Heiti TC","Noto Sans CJK TC","Noto Serif CJK TC","Noto Sans CJK JP"]:
    if any(f == x.name for x in fm.fontManager.ttflist): plt.rcParams["font.family"] = f; break
plt.rcParams["axes.unicode_minus"] = False
BLUE, ORANGE, GRAY = "#2a78d6", "#eb6834", "#8a8a85"
d = pd.read_csv(sys.argv[1]); d["absV"] = d.V_motor_V.abs()
fig, ax = plt.subplots(1, 2, figsize=(13, 5.3))
xx = np.linspace(0, 0.45, 50)
I_all, V_all = d.I_set_A.values, d.absV.values
R0 = (I_all @ V_all) / (I_all @ I_all)
ax[0].plot(xx, R0 * xx, c=GRAY, ls="--", lw=1.2, label=f"通過原點（兩方向合併）：R = {R0:.2f} Ω")
for dirn, c, lab in [("pos", BLUE, "正向"), ("neg", ORANGE, "反向")]:
    s = d[d.direction == dirn]; p = np.polyfit(s.I_set_A, s.absV, 1)
    ax[0].plot(s.I_set_A, s.absV, "o", c=c, ms=8, label=f"{lab}：斜率 {p[0]:.2f} Ω、截距 {p[1]*1000:+.0f} mV")
    ax[0].plot(xx[xx >= 0.18], np.polyval(p, xx[xx >= 0.18]), c=c, lw=1.2)
    ax[1].plot(s.I_set_A, s.absV / s.I_set_A, "o-", c=c, ms=8, lw=1.5, label=lab)
ax[0].set(xlim=(0, 0.45), ylim=(0, 1.2), xlabel="電流 I (A)", ylabel="馬達端電壓 |V| (V)",
          title="(a) V–I：幾乎是通過原點的直線 → 碳刷壓降可忽略")
ax[0].legend(frameon=False, loc="upper left", fontsize=9)
ax[1].axhline(R0, c=GRAY, ls="--", lw=1)
ax[1].axhline(2.6, c="#c33", ls=":", lw=1); ax[1].text(0.202, 2.605, "學長 電表 2.6 Ω", color="#c33", fontsize=9, ha="left", va="bottom")
ax[1].set(xlabel="電流 I (A)", ylabel="V / I (Ω)", ylim=(2.3, 2.75),
          title="(b) V/I：2.45–2.64 Ω，大電流偏高（推測發熱）")
ax[1].legend(frameon=False, loc="lower right")
for a in ax:
    a.grid(alpha=.25)
    for sp in ["top", "right"]: a.spines[sp].set_visible(False)
fig.suptitle("馬達線圈電阻（四線法，定電流，軸固定，單一轉子位置，2026-11-19）", fontsize=14, x=0.01, ha="left")
fig.text(0.01, 0.01, "電流取電源設定值（解析度 0.01 A）；電壓為電表直接量馬達端子。由小到大依序量測。", fontsize=9, color=GRAY)
fig.tight_layout(rect=[0, 0.03, 1, 0.94]); fig.savefig(sys.argv[2], dpi=140)
print("R0", round(R0, 3))
