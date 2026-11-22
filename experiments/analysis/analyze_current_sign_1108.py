"""
analyze_current_sign_1108.py — 老師 11/08 信第 1 點：電流命令為負時，PWM 是否也為負？

用法（在 XL330_STM32 資料夾執行）：
    python3 analysis/analyze_current_sign_1108.py
輸入：data/current_explore_20261108.txt（screen -L 原始紀錄；掃描段落 = 從 `c 0` 開始的 13 個 c 命令）
輸出：data/current_sign_summary_20261108.csv、figures/current_sign_1108.png
穩態 = 命令最後 1 s（t_ms 3000–4000）的平均 ± 標準差。
欄位：exp,t_ms,tick,pwm,cur,vel,pos,vtraj,ptraj,goal,volt,temp,err
"""
import os, glob
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
D = lambda *p: os.path.join(ROOT, *p)
for fp in glob.glob("/usr/share/fonts/**/Noto*CJK*Regular*.tt*", recursive=True):
    fm.fontManager.addfont(fp)
for f in ["PingFang TC", "Heiti TC", "Arial Unicode MS", "Noto Sans CJK TC", "Noto Serif CJK TC", "Noto Sans CJK JP", "Noto Serif CJK JP"]:
    if any(f == x.name for x in fm.fontManager.ttflist):
        plt.rcParams["font.family"] = f; break
plt.rcParams["axes.unicode_minus"] = False
PWM_FULL, RPM = 885, 0.229
BLUE, ORANGE, GRAY = "#2a78d6", "#eb6834", "#8a8a85"

# ---- 解析：只取掃描段（第一個 "> c 0" 之後）----
hdr, runs, cur, started = None, [], [], False
for line in open(D("data", "current_explore_20261108.txt"), encoding="utf-8", errors="replace"):
    line = line.replace("\r", "").strip()
    if line.startswith("> c 0") and not started:
        started = True
    if not started:
        continue
    if line.startswith("exp,"):
        hdr = line.split(","); continue
    if hdr and line[:1].isdigit() and line.count(",") == len(hdr) - 1:
        r = list(map(int, line.split(",")))
        if cur and r[1] < cur[-1][1]:
            runs.append(cur); cur = []
        cur.append(r)
if cur:
    runs.append(cur)
runs = [pd.DataFrame(r, columns=hdr) for r in runs]
runs = [r for r in runs if (r.exp == 1).all()]

rows = []
for r in runs:
    goal = int(r.goal.abs().max() * np.sign(r.goal.sum() or 1))
    ok = r[r.err == 0]
    ss = ok[(ok.t_ms >= 3000) & (ok.t_ms < 4000)]
    rows.append(dict(goal_mA=goal, n=len(ss), n_err=int((r.err != 0).sum()),
                     pwm_pct=ss.pwm.mean() / PWM_FULL * 100, pwm_sd=ss.pwm.std() / PWM_FULL * 100,
                     cur_mA=ss.cur.mean(), cur_sd=ss.cur.std(),
                     vel_rpm=ss.vel.mean() * RPM, vel_sd=ss.vel.std() * RPM,
                     sign_agree_pct=100 * ((np.sign(ok[(ok.t_ms >= 1100) & (ok.t_ms < 4000)].pwm) == np.sign(goal)).mean()) if goal else np.nan,
                     volt_V=ss.volt.mean() / 10, temp_C=int(ss.temp.max())))
S = pd.DataFrame(rows).sort_values("goal_mA")
S.round(2).to_csv(D("data", "current_sign_summary_20261108.csv"), index=False)
print(S.round(2).to_string(index=False))

# ---- 圖 ----
fig, ax = plt.subplots(2, 2, figsize=(13, 9.5))
def style(a):
    a.grid(alpha=.25); a.axhline(0, c="#999", lw=.8); a.axvline(0, c="#999", lw=.8)
    for sp in ["top", "right"]: a.spines[sp].set_visible(False)
pos, neg = S[S.goal_mA > 0], S[S.goal_mA < 0]

a = ax[0, 0]
for d, c, lab in [(pos, BLUE, "正電流命令"), (neg, ORANGE, "負電流命令")]:
    a.errorbar(d.goal_mA, d.pwm_pct, yerr=d.pwm_sd, fmt="o", c=c, ms=7, capsize=3, label=lab)
z = S[S.goal_mA == 0]; a.plot(z.goal_mA, z.pwm_pct, "s", c=GRAY, ms=7, label="0 mA")
a.axhline(100, c="#c33", ls=":", lw=1); a.axhline(-100, c="#c33", ls=":", lw=1)
a.set(xlabel="電流命令 Goal Current (mA)", ylabel="穩態 PWM (%)", title="(a) PWM 的正負號跟著電流命令走")
a.legend(frameon=False, loc="upper left"); style(a)

a = ax[0, 1]
lim = S.goal_mA.abs().max() * 1.1
a.plot([-lim, lim], [-lim, lim], c=GRAY, ls="--", lw=1, label="y = x（完全追上）")
for d, c in [(pos, BLUE), (neg, ORANGE)]:
    a.errorbar(d.goal_mA, d.cur_mA, yerr=d.cur_sd, fmt="o", c=c, ms=7, capsize=3)
a.plot(z.goal_mA, z.cur_mA, "s", c=GRAY, ms=7)
a.set(xlabel="電流命令 (mA)", ylabel="穩態回報電流 (mA)", title="(b) 回報電流追蹤命令（±100 mA 追不上）")
a.legend(frameon=False, loc="upper left"); style(a)

a = ax[1, 0]
for d, c in [(pos, BLUE), (neg, ORANGE)]:
    a.errorbar(d.goal_mA, d.vel_rpm, yerr=d.vel_sd, fmt="o", c=c, ms=7, capsize=3)
a.plot(z.goal_mA, z.vel_rpm, "s", c=GRAY, ms=7)
a.set(xlabel="電流命令 (mA)", ylabel="穩態速度 (rpm)", title="(c) 轉速：±10 mA 轉不動（靜摩擦）"); style(a)

a = ax[1, 1]
sel = [r for r in runs if abs(r.goal.abs().max()) == 40]
for r in sel:
    g = int(r.goal.max() if r.goal.max() > 0 else r.goal.min()); c = BLUE if g > 0 else ORANGE
    r = r[r.err == 0]; t = r.t_ms / 1000 - 1
    a.plot(t, r.pwm / PWM_FULL * 100, c=c, lw=2, label=f"PWM (%)，命令 {g:+d} mA")
    a.plot(t, r.cur, c=c, lw=1.2, ls="--", label=f"回報電流 (mA)，命令 {g:+d} mA")
a.axvline(0, c="#bbb", ls="--", lw=1)
a.set(xlabel="時間（命令 = 0）[s]", ylabel="PWM (%) / 電流 (mA)", title="(d) ±40 mA 的時間曲線：暫態期間符號也一致", xlim=(-0.5, 3.2))
a.legend(frameon=False, fontsize=8, loc="center right"); style(a)

fig.suptitle("電流模式正負號實驗（2026-11-08，裝手臂，空載轉動）", fontsize=15, x=0.02, ha="left")
fig.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig(D("figures", "current_sign_1108.png"), dpi=130)
print("saved figures/current_sign_1108.png")
