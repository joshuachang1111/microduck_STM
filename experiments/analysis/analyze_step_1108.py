"""
analyze_step_1108.py — 老師 11/08 信第 2 點：位置小步階（PWM 不飽和）與控制參數

用法（在 XL330_STM32 資料夾執行）：python3 analysis/analyze_step_1108.py [label-XL330]
輸入：data/step_<motor>_20261108.txt（v4 韌體 s 指令，500 Hz）
輸出：data/step_runs_<motor>_20261108.csv（每次一列）、data/step_summary_<motor>_20261108.csv（同設定平均）
      figures/step_size_<motor>_1108.png（實驗 A）、figures/step_gains_<motor>_1108.png（實驗 B）
欄位：exp,t_ms,tick,pwm,cur,vel,pos,goal,err（步階在 t_ms = 200）
"""
import os, re, sys, glob
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
MOTOR = sys.argv[1] if len(sys.argv) > 1 else "label-XL330"
PWM_FULL, DEG, T_STEP = 885, 360 / 4096, 200

# ---- parse ----
runs, meta, rows, hdr = [], None, [], None
for line in open(D("data", f"step_{MOTOR}_20261108.txt"), encoding="utf-8", errors="replace"):
    line = line.replace("\r", "").strip()
    m = re.search(r"# step: start=(-?\d+) goal=(-?\d+) step=(-?\d+) raw .*P=(\d+) D=(\d+) I=(\d+)", line)
    if m:
        meta = dict(zip(["start", "goal", "step", "P", "D", "I"], map(int, m.groups()))); rows = []; continue
    if line.startswith("exp,t_ms"):
        hdr = line.split(","); continue
    if meta and hdr and line.startswith("6,") and line.count(",") == len(hdr) - 1:
        rows.append(list(map(int, line.split(","))))
    if line.startswith("# exp 6 end") and meta:
        d = pd.DataFrame(rows, columns=hdr); runs.append((meta, d)); meta = None

def metrics(m, d):
    ok = d[d.err == 0].copy()
    a, sgn = abs(m["step"]), np.sign(m["step"])
    ok["ts"] = ok.t_ms - T_STEP
    ok["y"] = (ok.pos - m["start"]) * sgn / a           # 0 -> 1
    post = ok[ok.ts >= 0]
    t10 = post[post.y >= 0.1].ts.min(); t90 = post[post.y >= 0.9].ts.min()
    move = post[(post.pos - m["start"]).abs() >= 2].ts.min()
    band = max(a / 20, 2) / a
    out = post[(post.y - 1).abs() > band]
    fin = post[post.ts >= 1600]
    return dict(step_deg=round(m["step"] * DEG, 1), P=m["P"], D=m["D"], I=m["I"],
                first_move_ms=move, rise_ms=t90 - t10, overshoot_pct=max(0, (post.y.max() - 1) * 100),
                settle_ms=(out.ts.max() + 2) if len(out) else 0,
                final_err_deg=(fin.y.mean() - 1) * a * DEG, peak_pwm_pct=post.pwm.abs().max() / PWM_FULL * 100,
                sat_ms=int((post.pwm.abs() >= PWM_FULL).sum() * 2), read_err=int((d.err != 0).sum()))

R = pd.DataFrame([metrics(m, d) for m, d in runs])
R.round(2).to_csv(D("data", f"step_runs_{MOTOR}_20261108.csv"), index=False)
key = ["step_deg", "P", "D", "I"]
S = R.groupby(key, sort=False).agg(n=("rise_ms", "size"), first_move_ms=("first_move_ms", "mean"),
        rise_ms=("rise_ms", "mean"), rise_min=("rise_ms", "min"), rise_max=("rise_ms", "max"),
        overshoot_pct=("overshoot_pct", "mean"), settle_ms=("settle_ms", "mean"),
        final_err_deg=("final_err_deg", "mean"), peak_pwm_pct=("peak_pwm_pct", "mean"),
        sat_ms=("sat_ms", "mean"), read_err=("read_err", "sum")).reset_index()
S.round(2).to_csv(D("data", f"step_summary_{MOTOR}_20261108.csv"), index=False)
print(S.round(1).to_string(index=False))

def first(cond):
    for m, d in runs:
        if cond(m): return m, d

def norm(m, d):
    ok = d[d.err == 0]
    return (ok.t_ms - T_STEP), (ok.pos - m["start"]) * np.sign(m["step"]) / abs(m["step"]), ok.pwm / PWM_FULL * 100 * np.sign(m["step"])

def style(a):
    a.grid(alpha=.25)
    for sp in ["top", "right"]: a.spines[sp].set_visible(False)

# ---- Fig A: step size, P = 400 ----
sizes = [5, 10, 20, 40, 60, 90]
cmap = plt.get_cmap("Blues")
fig, ax = plt.subplots(1, 3, figsize=(17, 5.4))
for i, deg in enumerate(sizes):
    md = first(lambda m: m["P"] == 400 and m["D"] == 0 and abs(round(m["step"] * DEG) - deg) < 1)
    if not md: continue
    t, y, p = norm(*md)
    c = cmap(0.35 + 0.65 * i / (len(sizes) - 1))
    ls = "--" if deg >= 60 else "-"
    ax[0].plot(t, y, c=c, lw=2, ls=ls, label=f"{deg}°" + ("（飽和）" if deg >= 60 else ""))
    ax[1].plot(t, p, c=c, lw=2, ls=ls, label=f"{deg}°")
ax[0].axhline(1, c="#999", lw=.8); ax[0].axvline(0, c="#bbb", ls="--", lw=1)
ax[0].set(xlim=(-20, 400), xlabel="時間（步階 = 0）[ms]", ylabel="正規化位置（0 → 1）",
          title="(a) 50° 以下的響應幾乎完全重疊；60°、90° 變慢")
ax[0].legend(frameon=False, loc="lower right")
ax[1].axhline(100, c="#c33", ls=":", lw=1)
ax[1].set(xlim=(-20, 400), xlabel="時間（步階 = 0）[ms]", ylabel="PWM (%)",
          title="(b) PWM 峰值和步階成正比，60° 起碰到 100 %")
A = S[(S.P == 400) & (S.D == 0)].sort_values("step_deg")
ax[2].errorbar(A.step_deg, A.rise_ms, yerr=[A.rise_ms - A.rise_min, A.rise_max - A.rise_ms], fmt="o-", c="#2a78d6", ms=7, capsize=3)
lim = 885 * 256 / 400 * DEG
ax[2].axvline(lim, c="#c33", ls=":", lw=1.2); ax[2].text(lim + 1.5, A.rise_ms.min() - 3, f"不飽和上限 ≈ {lim:.0f}°", color="#c33")
for _, r in A.iterrows():
    ax[2].annotate(f"{r.rise_ms:.0f}", (r.step_deg, r.rise_ms), textcoords="offset points", xytext=(0, 8), ha="center", fontsize=9)
ax[2].set(xlabel="步階大小 (°)", ylabel="上升時間 10–90 % (ms)", title="(c) 上升時間：不飽和時固定約 122 ms", ylim=(100, 165))
for a in ax: style(a)
fig.suptitle(f"實驗 A：步階大小（{MOTOR}，P = 400、D = I = 0，500 Hz，無手臂）", fontsize=14, x=0.01, ha="left")
fig.tight_layout(rect=[0, 0, 1, 0.94]); fig.savefig(D("figures", f"step_size_{MOTOR}_1108.png"), dpi=130); plt.close(fig)

# ---- Fig B: gains at 10 deg ----
fig, ax = plt.subplots(2, 2, figsize=(14, 10))
Ps = [400, 800, 1600, 3200]; Ds = [0, 200, 500, 1000]
for i, P in enumerate(Ps):
    md = first(lambda m, P=P: m["P"] == P and m["D"] == 0 and round(m["step"] * DEG) == 10)
    t, y, p = norm(*md); c = plt.get_cmap("Blues")(0.35 + 0.65 * i / 3)
    ax[0, 0].plot(t, y, c=c, lw=2, label=f"P = {P}"); ax[1, 0].plot(t, p, c=c, lw=2, label=f"P = {P}")
for i, Dg in enumerate(Ds):
    md = first(lambda m, Dg=Dg: m["P"] == 1600 and m["D"] == Dg and round(m["step"] * DEG) == 10)
    t, y, p = norm(*md); c = plt.get_cmap("Oranges")(0.35 + 0.65 * i / 3)
    ax[0, 1].plot(t, y, c=c, lw=2, label=f"P = 1600, D = {Dg}")
for a, ttl in [(ax[0, 0], "(a) 加大 P（D = 0）：越快，但超越量越大"), (ax[0, 1], "(b) P = 1600 再加 D：壓住超越量，速度幾乎不變")]:
    a.axhline(1, c="#999", lw=.8); a.axhspan(0.95, 1.05, color="#eee", zorder=0)
    a.set(xlim=(-10, 300), ylim=(-0.05, 1.45), xlabel="時間（步階 = 0）[ms]", ylabel="正規化位置（0 → 1）", title=ttl)
    a.legend(frameon=False, loc="lower right")
ax[1, 0].axhline(100, c="#c33", ls=":", lw=1)
ax[1, 0].set(xlim=(-10, 300), xlabel="時間（步階 = 0）[ms]", ylabel="PWM (%)", title="(c) P 越大 PWM 越大；P = 3200 時 10° 已飽和")
ax[1, 0].legend(frameon=False, loc="upper right")
B = S[(S.step_deg == 10.0)]
a = ax[1, 1]
for _, r in B.iterrows():
    c = "#2a78d6" if r.D == 0 else "#eb6834"
    a.plot(r.rise_ms, r.overshoot_pct, "o", c=c, ms=9)
    a.annotate(f"P{int(r.P)}" + (f" D{int(r.D)}" if r.D else ""), (r.rise_ms, r.overshoot_pct),
               textcoords="offset points", xytext=(7, 4), fontsize=9)
a.set(xlabel="上升時間 10–90 % (ms)", ylabel="超越量 (%)", title="(d) 取捨：左下角最好（快又不超越）")
for a in ax.flat: style(a)
fig.suptitle(f"實驗 B：控制參數（{MOTOR}，10° 步階，500 Hz，無手臂）", fontsize=14, x=0.01, ha="left")
fig.tight_layout(rect=[0, 0, 1, 0.95]); fig.savefig(D("figures", f"step_gains_{MOTOR}_1108.png"), dpi=130)
print("saved figures")
