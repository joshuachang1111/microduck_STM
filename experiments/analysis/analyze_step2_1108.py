"""
analyze_step2_1108.py — 第二輪：D 加大（C）與調好的參數在大步階（D），合併第一輪一起比較

用法：python3 analysis/analyze_step2_1108.py [label-XL330]
輸入：data/step_<motor>_20261108.txt（第一輪）、data/step2_<motor>_20261108.txt（第二輪）
輸出：data/step_all_runs_<motor>_20261108.csv、data/step_all_summary_<motor>_20261108.csv
      figures/step_tuning_<motor>_1108.png、figures/step_large_<motor>_1108.png
停止後的抖動 = 步階後 1.0–1.8 s 的 PWM 標準差（%），抖/嗡嗡聲會讓它變大。
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
BLUE, ORANGE, GRAY, GREEN = "#2a78d6", "#eb6834", "#8a8a85", "#1baf7a"

def parse(path, rnd):
    out, meta, rows, hdr = [], None, [], None
    for line in open(path, encoding="utf-8", errors="replace"):
        line = line.replace("\r", "").strip()
        m = re.search(r"# step: start=(-?\d+) goal=(-?\d+) step=(-?\d+) raw .*P=(\d+) D=(\d+) I=(\d+)", line)
        if m:
            meta = dict(zip(["start", "goal", "step", "P", "D", "I"], map(int, m.groups()))); meta["round"] = rnd; rows = []; continue
        if line.startswith("exp,t_ms"):
            hdr = line.split(","); continue
        if meta and hdr and line.startswith("6,") and line.count(",") == len(hdr) - 1:
            rows.append(list(map(int, line.split(","))))
        if line.startswith("# exp 6 end") and meta:
            out.append((meta, pd.DataFrame(rows, columns=hdr))); meta = None
    return out

runs = parse(D("data", f"step_{MOTOR}_20261108.txt"), 1) + parse(D("data", f"step2_{MOTOR}_20261108.txt"), 2)

def metrics(m, d):
    ok = d[d.err == 0].copy()
    a, sgn = abs(m["step"]), np.sign(m["step"])
    ok["ts"] = ok.t_ms - T_STEP
    ok["y"] = (ok.pos - m["start"]) * sgn / a
    post = ok[ok.ts >= 0]
    t10 = post[post.y >= 0.1].ts.min(); t90 = post[post.y >= 0.9].ts.min()
    band = max(a / 20, 2) / a
    out = post[(post.y - 1).abs() > band]
    tail = post[(post.ts >= 1000) & (post.ts < 1800)]
    return dict(round=m["round"], step_deg=round(m["step"] * DEG), P=m["P"], D=m["D"], I=m["I"],
                rise_ms=t90 - t10, overshoot_pct=max(0, (post.y.max() - 1) * 100),
                settle_ms=(out.ts.max() + 2) if len(out) else 0,
                final_err_deg=(tail.y.mean() - 1) * a * DEG, peak_pwm_pct=post.pwm.abs().max() / PWM_FULL * 100,
                sat_ms=int((post.pwm.abs() >= PWM_FULL).sum() * 2),
                jitter_pwm_pct=tail.pwm.std() / PWM_FULL * 100, read_err=int((d.err != 0).sum()))

R = pd.DataFrame([metrics(m, d) for m, d in runs])
R.round(2).to_csv(D("data", f"step_all_runs_{MOTOR}_20261108.csv"), index=False)
key = ["step_deg", "P", "D"]
S = R.groupby(key).agg(n=("rise_ms", "size"), rise_ms=("rise_ms", "mean"), rise_min=("rise_ms", "min"), rise_max=("rise_ms", "max"),
                       overshoot_pct=("overshoot_pct", "mean"), settle_ms=("settle_ms", "mean"),
                       final_err_deg=("final_err_deg", "mean"), peak_pwm_pct=("peak_pwm_pct", "mean"),
                       sat_ms=("sat_ms", "mean"), jitter_pwm_pct=("jitter_pwm_pct", "mean"), read_err=("read_err", "sum")).reset_index()
S.round(2).to_csv(D("data", f"step_all_summary_{MOTOR}_20261108.csv"), index=False)
print(S.round(1).to_string(index=False))

def first(cond):
    for m, d in runs:
        if cond(m): return m, d

def style(a):
    a.grid(alpha=.25)
    for sp in ["top", "right"]: a.spines[sp].set_visible(False)

# ---- Fig: tuning at 10 deg ----
T = S[S.step_deg == 10]
fig, ax = plt.subplots(1, 2, figsize=(15, 6.2))
a = ax[0]
a.axhspan(0, 5, color="#e8f4ec", zorder=0); a.text(128, 4.2, "超越量 ≤ 5 %", color=GREEN, ha="right", fontsize=9)
for P, c in [(400, GRAY), (800, GRAY), (1600, BLUE), (3200, ORANGE)]:
    t = T[T.P == P].sort_values("D")
    a.plot(t.rise_ms, t.overshoot_pct, "-o", c=c, ms=8, lw=1.4, label=f"P = {P}" + ("（D 0→3000）" if len(t) > 1 else ""))
    for _, r in t.iterrows():
        if (P, r.D) in [(3200, 3000), (1600, 1500)]: continue        # 這兩點由「推薦」標籤說明
        dx = -40 if P == 3200 else 6
        a.annotate(f"D{int(r.D)}", (r.rise_ms, r.overshoot_pct), textcoords="offset points", xytext=(dx, 5), fontsize=8, color=c)
for P, Dg, lab, xy in [(3200, 3000, "P3200 D3000：最快", (62, 19)), (1600, 1500, "P1600 D1500：零超越、PWM 不飽和", (62, 13))]:
    r = T[(T.P == P) & (T.D == Dg)].iloc[0]
    a.plot(r.rise_ms, r.overshoot_pct, "o", ms=17, mfc="none", mec=GREEN, mew=2)
    a.annotate(lab, (r.rise_ms, r.overshoot_pct), textcoords="data", xytext=xy, color=GREEN, fontsize=10,
               arrowprops=dict(arrowstyle="-", color=GREEN, lw=0.8))
a.set(xlabel="上升時間 10–90 % (ms)", ylabel="超越量 (%)", title="(a) 10° 步階：每條線是固定 P、D 由 0 加到 3000", xlim=(25, 130))
a.legend(frameon=False, loc="upper right"); style(a)
a = ax[1]
for (P, Dg), c, lab in [((400, 0), GRAY, "P400（預設）"), ((1600, 1000), "#7fb2ea", "P1600 D1000"),
                        ((1600, 1500), BLUE, "P1600 D1500"), ((3200, 3000), ORANGE, "P3200 D3000")]:
    m, d = first(lambda m, P=P, Dg=Dg: m["P"] == P and m["D"] == Dg and round(m["step"] * DEG) == 10)
    ok = d[d.err == 0]
    a.plot(ok.t_ms - T_STEP, (ok.pos - m["start"]) / m["step"], c=c, lw=2.2, label=lab)
a.axhline(1, c="#999", lw=.8); a.axhspan(0.95, 1.05, color="#eee", zorder=0)
a.set(xlim=(-10, 250), xlabel="時間（步階 = 0）[ms]", ylabel="正規化位置（0 → 1）", title="(b) 代表性響應：預設 vs 調整後")
a.legend(frameon=False, loc="lower right"); style(a)
fig.suptitle(f"實驗 C：P 和 D 一起調（{MOTOR}，10° 步階，500 Hz，無手臂）", fontsize=14, x=0.01, ha="left")
fig.tight_layout(rect=[0, 0, 1, 0.93]); fig.savefig(D("figures", f"step_tuning_{MOTOR}_1108.png"), dpi=130); plt.close(fig)

# ---- Fig: large steps ----
fig, ax = plt.subplots(1, 2, figsize=(15, 6))
a = ax[0]
base = S[(S.P == 400) & (S.D == 0)].sort_values("step_deg")
tuned = S[(S.P == 1600) & (S.D == 1000) & (S.step_deg > 0)].sort_values("step_deg")
a.plot(base.step_deg, base.rise_ms, "-o", c=GRAY, ms=8, lw=2, label="P400（預設）")
a.plot(tuned.step_deg, tuned.rise_ms, "-o", c=BLUE, ms=8, lw=2, label="P1600 D1000")
for _, r in tuned.iterrows():
    b = base[base.step_deg == r.step_deg]
    if len(b):
        a.annotate(f"快 {b.rise_ms.iloc[0] / r.rise_ms:.1f}×", (r.step_deg, r.rise_ms), textcoords="offset points", xytext=(0, -18), ha="center", color=BLUE, fontsize=9)
lim = 885 * 256 / 1600 * DEG
a.axvline(lim, c=BLUE, ls=":", lw=1); a.text(lim + 1, 160, f"P1600 不飽和上限 ≈ {lim:.0f}°", color=BLUE, fontsize=9)
a.set(xlabel="步階大小 (°)", ylabel="上升時間 10–90 % (ms)", title="(a) 步階越大，調整參數的優勢越小", ylim=(0, 170))
a.legend(frameon=False, loc="lower right"); style(a)
a = ax[1]
for i, deg in enumerate([10, 20, 40, 60, 90]):
    md = first(lambda m, deg=deg: m["P"] == 1600 and m["D"] == 1000 and round(m["step"] * DEG) == deg)
    if not md: continue
    m, d = md; ok = d[d.err == 0]; c = plt.get_cmap("Blues")(0.35 + 0.65 * i / 4)
    a.plot(ok.t_ms - T_STEP, ok.pwm / PWM_FULL * 100, c=c, lw=1.8, label=f"{deg}°")
a.axhline(100, c="#c33", ls=":", lw=1)
a.set(xlim=(-10, 300), xlabel="時間（步階 = 0）[ms]", ylabel="PWM (%)", title="(b) P1600 D1000 的 PWM：20° 起就飽和，90° 飽和約 160 ms")
a.legend(frameon=False, loc="upper right", title="步階"); style(a)
fig.suptitle(f"實驗 D：調整後的參數在大步階（{MOTOR}，500 Hz，無手臂）", fontsize=14, x=0.01, ha="left")
fig.tight_layout(rect=[0, 0, 1, 0.93]); fig.savefig(D("figures", f"step_large_{MOTOR}_1108.png"), dpi=130)
print("saved")
