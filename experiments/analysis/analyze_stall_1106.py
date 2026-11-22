"""
analyze_stall_1106.py — 堵轉扭力實驗（PWM 模式，手臂 r = 40 mm 壓電子秤）分析

用法（在 XL330_STM32 資料夾執行）：
    python3 analysis/analyze_stall_1106.py [label-XL330]

輸入：
    data/stall_scale_<motor>_20261106.csv   秤重（motor,r_mm,pwm_pct,scale_g,...）
    data/stall_<motor>_20261106.txt         screen -L 原始 log（實驗 4，w N 3）
輸出：
    data/stall_summary_<motor>_20261106.csv
    figures/stall_<motor>_1106.png
注意：label-XL330 的秤重來自第一輪、電流來自第二輪（兩輪秤重實際差異很小，未逐筆記錄）。
電流取命令後 1.5–2.0 s 的平均（對應讀秤的時間點）。
"""
import os, io, sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
D = lambda *p: os.path.join(ROOT, *p)
import glob
for fp in glob.glob("/usr/share/fonts/**/Noto*CJK*Regular*.tt*", recursive=True):
    fm.fontManager.addfont(fp)
for f in ["PingFang TC", "Heiti TC", "Arial Unicode MS", "Noto Sans CJK TC", "Noto Serif CJK TC", "Noto Sans CJK JP", "Noto Serif CJK JP"]:
    if any(f == x.name for x in fm.fontManager.ttflist):
        plt.rcParams["font.family"] = f
        break
plt.rcParams["axes.unicode_minus"] = False
G, PWM_FULL = 9.80665, 885
MOTOR = sys.argv[1] if len(sys.argv) > 1 else "label-XL330"
BLUE, ORANGE, GRAY, GREEN = "#2a78d6", "#eb6834", "#8a8a85", "#1baf7a"


def parse_runs(path):
    """screen log -> 每次 w 指令一個 DataFrame（t_ms 歸零即為新的一筆）"""
    hdr, runs, cur = None, [], []
    for line in open(path, encoding="utf-8", errors="replace"):
        line = line.replace("\r", "").strip()
        if line.startswith("exp,"):
            hdr = line.split(","); continue
        if hdr and line[:1].isdigit() and line.count(",") == len(hdr) - 1:
            r = list(map(int, line.split(",")))
            if cur and r[1] < cur[-1][1]:
                runs.append(cur); cur = []
            cur.append(r)
    if cur:
        runs.append(cur)
    return [pd.DataFrame(r, columns=hdr) for r in runs]


runs = [r for r in parse_runs(D("data", f"stall_{MOTOR}_20261106.txt")) if (r.exp == 4).all()]
rows = []
for r in runs:
    r = r[r.err == 0]
    w = r[(r.t_ms >= 2500) & (r.t_ms < 3000) & (r.cur > 0)]   # 命令 (t=1 s) 後 1.5–2.0 s；Overload 後電流 = 0 的點不算
    rows.append(dict(pwm_pct=round(r.goal.max() / PWM_FULL * 100),
                     cur_mA=w.cur.mean(), volt_V=w.volt.mean() / 10,
                     temp_C=r.temp.max(), pos_end=int(r.pos.iloc[-1]), n_err=0))
log = pd.DataFrame(rows)
log = log[log.cur_mA > 5].drop_duplicates("pwm_pct", keep="last")   # 電流 ≈ 0 = 馬達已因錯誤關閉扭力
sc = pd.read_csv(D("data", f"stall_scale_{MOTOR}_20261106.csv"))
sc = sc[~sc.notes.fillna("").astype(str).str.startswith("排除")][["pwm_pct", "r_mm", "scale_g"]]   # notes 以「排除」開頭的列不用
s = sc.merge(log, on="pwm_pct").sort_values("pwm_pct")
s["d"] = s.pwm_pct / 100
s["torque_Nm"] = s.scale_g / 1000 * G * s.r_mm / 1000
s["torque_per_I"] = s.torque_Nm / (s.cur_mA / 1000)

# 模型：tau = k * (I - I0) / d   （回報電流 = 輸入端電流；I0 = 待機電流）
best = None
for I0 in np.arange(0, 60.5, 0.5):
    x = (s.cur_mA - I0) / 1000 / s.d
    k = float(x @ s.torque_Nm / (x @ x))
    sse = float(((s.torque_Nm - k * x) ** 2).sum())
    if best is None or sse < best[0]:
        best = (sse, I0, k)
_, I0, K = best
s["I_coil_est_A"] = (s.cur_mA - I0) / 1000 / s.d
r2 = lambda y, yh: 1 - ((y - yh) ** 2).sum() / ((y - y.mean()) ** 2).sum()
R2_model = r2(s.torque_Nm, K * s.I_coil_est_A)
p_lin = np.polyfit(s.cur_mA / 1000, s.torque_Nm, 1)
R2_lin = r2(s.torque_Nm, np.polyval(p_lin, s.cur_mA / 1000))
s.round(4).to_csv(D("data", f"stall_summary_{MOTOR}_20261106.csv"), index=False)
print(s.round(3).to_string(index=False))
print(f"\n模型 tau = {K:.3f} N·m/A × (I - {I0:.1f} mA)/d   R² = {R2_model:.4f}")
print(f"直接線性 tau = {p_lin[0]:.3f}·I + {p_lin[1]:.3f}   R² = {R2_lin:.4f}")

# ---- 圖 ----
fig, ax = plt.subplots(2, 2, figsize=(13, 9.5))
a = ax[0, 0]
low = s[s.pwm_pct <= 40]; pl = np.polyfit(low.pwm_pct, low.torque_Nm, 1)
xx = np.linspace(0, 100, 50)
a.plot(xx, np.polyval(pl, xx), c=GRAY, ls="--", lw=1.3, label="10–40 % 直線外插")
a.plot(s.pwm_pct, s.torque_Nm, "o-", c=BLUE, lw=2, label="實測扭力")
a2 = a.twinx(); a2.plot(s.pwm_pct, s.volt_V, "s:", c=ORANGE, ms=4); a2.set_ylabel("馬達端電壓 (V)", color=ORANGE)
a2.set_ylim(4.0, 5.2)
a.set(xlabel="PWM (%)", ylabel="扭力 (N·m)", title=f"(a) 扭力 vs PWM：高 PWM 時變彎（壓降 {s.volt_V.iloc[0]:.1f}→{s.volt_V.iloc[-1]:.1f} V）")
a.legend(frameon=False, loc="upper left")

a = ax[0, 1]
a.plot(s.cur_mA, s.torque_Nm, "o-", c=BLUE, lw=2)
for _, r in s.iterrows():
    a.annotate(f"{r.pwm_pct:.0f}%", (r.cur_mA, r.torque_Nm), textcoords="offset points", xytext=(6, -10), fontsize=8, color=GRAY)
a.set(xlabel="回報電流 Present Current (mA)", ylabel="扭力 (N·m)",
      title=f"(b) 扭力 vs 回報電流：不成正比（τ/I 從 {s.torque_per_I.iloc[0]:.2f} 掉到 {s.torque_per_I.iloc[-1]:.2f}）")

a = ax[1, 0]
a.plot(s.I_coil_est_A, s.torque_Nm, "o", c=BLUE, ms=7, label="實測")
xx = np.linspace(0, s.I_coil_est_A.max() * 1.05, 20)
a.plot(xx, K * xx, c=GREEN, lw=2, label=f"τ = {K:.3f}·(I−{I0:.0f} mA)/d,  R² = {R2_model:.3f}")
a.set(xlabel="推估線圈電流 (I_回報 − I0) / d  (A)", ylabel="扭力 (N·m)",
      title="(c) 把回報電流當輸入端電流換算後：" + ("接近直線" if R2_model > 0.98 else "仍偏離直線"))
a.legend(frameon=False, loc="upper left")

a = ax[1, 1]
cmap = plt.get_cmap("viridis")
runs_ok = [r for r in runs if (r.cur > 5).any()]   # 扭力已關（電流全 0）的那筆不畫
for i, r in enumerate(runs_ok):
    pct = round(r.goal.max() / PWM_FULL * 100)
    r = r[r.err == 0]
    a.plot(r.t_ms / 1000 - 1, r.cur, c=cmap(i / max(1, len(runs_ok) - 1)), lw=1.4, label=f"{pct}%")
a.axvspan(1.5, 2.0, color="#ddd", alpha=.6)
a.set(xlabel="時間（命令 = 0）[s]", ylabel="回報電流 (mA)", title="(d) 每筆的電流時間曲線（灰色 = 取值區間）", xlim=(-0.5, 5))
a.legend(frameon=False, ncol=2, fontsize=8, loc="upper right")
for a in ax.flat:
    a.grid(alpha=.25)
    for sp in ["top", "right"]:
        a.spines[sp].set_visible(False)
fig.suptitle(f"堵轉扭力實驗：{MOTOR}，r = 40 mm" + ("（秤重第一輪＋電流第二輪）" if MOTOR == "label-XL330" else "（80–100 % 在命令後 1.9–4.6 s 觸發 Overload）"), fontsize=15, x=0.02, ha="left")
fig.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig(D("figures", f"stall_{MOTOR}_1106.png"), dpi=130)
print("saved", f"figures/stall_{MOTOR}_1106.png")
