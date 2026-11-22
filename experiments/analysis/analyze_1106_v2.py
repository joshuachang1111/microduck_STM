"""
analyze_1106_v2.py — 新版程式（11 欄）資料分析：標籤 XL330 vs 標籤 XC330（2026-11-06 下午）

用法（在 XL330_STM32 資料夾執行）：
    python3 analysis/analyze_1106_v2.py

輸入（screen -L 的原始紀錄）：
    data/label-XL330_v2_raw_20261106.txt
    data/label-XC330_v2_raw_20261106.txt
另外讀舊版資料做重現性比較：
    data/label-XL330_20261106.csv、data/label-XC330_20261106.csv、data/reference_label-XC330_20261030.csv
輸出：
    data/label-XL330_v2_20261106.csv、data/label-XC330_v2_20261106.csv   整理好的 CSV
    data/summary_metrics_v2_1106.csv                                     指標（含舊資料，比較重現性）
    figures/XL_vs_XC_v2_1106.png                                         三個實驗 × 四個量
    figures/position_step_3curves_1106.png                               Goal / Position Trajectory / 實際位置

欄位：exp,t_ms,tick,pwm,cur,vel,pos,vtraj,ptraj,goal,err
    pwm 885 = 100 %；cur 約 1 mA/單位（輸入端電流）；vel/vtraj 0.229 rpm/單位；pos/ptraj 4096 = 一圈
命令在 t = 1.0 s 送出，圖上以命令時刻為 0。
"""
import os
import io
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
D = lambda *p: os.path.join(ROOT, *p)
for f in ["PingFang TC", "Heiti TC", "Arial Unicode MS", "Noto Sans CJK TC", "Noto Sans CJK JP"]:
    if any(f == x.name for x in fm.fontManager.ttflist):
        plt.rcParams["font.family"] = f
        break
plt.rcParams["axes.unicode_minus"] = False
PWM_FULL, RPM, DEG = 885, 0.229, 360 / 4096
BLUE, ORANGE, GRAY = "#2a78d6", "#eb6834", "#8a8a85"


def parse_raw(path):
    """screen 原始紀錄 -> 乾淨 CSV（只留第一個標題列和數字開頭的列）"""
    out, header = [], False
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.replace("\r", "").strip()
            if line.startswith("exp,") and not header:
                out.append(line); header = True
            elif line[:1].isdigit() and "," in line:
                out.append(line)
    return pd.read_csv(io.StringIO("\n".join(out)))


def prep(d, time_col="t_ms"):
    n_all = len(d)
    n_err = int((d.err != 0).sum())
    d = d[d.err == 0].copy()
    d["t"] = d[time_col] / (1000 if time_col == "t_ms" else 1) - 1.0
    return d, n_err, n_all


XLv2_raw = parse_raw(D("data", "label-XL330_v2_raw_20261106.txt"))
XCv2_raw = parse_raw(D("data", "label-XC330_v2_raw_20261106.txt"))
XLv2_raw.to_csv(D("data", "label-XL330_v2_20261106.csv"), index=False)
XCv2_raw.to_csv(D("data", "label-XC330_v2_20261106.csv"), index=False)

ref = pd.read_csv(D("data", "reference_label-XC330_20261030.csv")).rename(
    columns={"pwm_raw": "pwm", "current_raw": "cur", "velocity_raw": "vel", "position_raw": "pos"})

RUNS = [  # (名稱, 資料)
    ("標籤 XL330｜11/06 新版程式", prep(XLv2_raw)),
    ("標籤 XL330｜11/06 舊版程式", prep(pd.read_csv(D("data", "label-XL330_20261106.csv")))),
    ("標籤 XC330｜11/06 新版程式", prep(XCv2_raw)),
    ("標籤 XC330｜11/06 舊版程式", prep(pd.read_csv(D("data", "label-XC330_20261106.csv")))),
    ("標籤 XC330｜10/30 舊版程式", prep(ref, "t_s")),
]


def metrics(name, df, n_err, n_all):
    r = {"資料": name, "逾時比例_%": round(100 * n_err / n_all, 1)}
    for e, tag in [(1, "實驗1_80mA"), (3, "實驗3_11.5rpm")]:
        s = df[(df.exp == e) & (df.t >= 2.0)]
        r[f"{tag}_PWM_%"] = round(s.pwm.mean() / PWM_FULL * 100, 1)
        r[f"{tag}_電流_mA"] = round(s.cur.mean(), 1)
        r[f"{tag}_速度_rpm"] = round(s.vel.mean() * RPM, 1)
    x = df[(df.exp == 2) & (df.t >= 0)]
    t10 = x[x.pos >= 0.1 * 1024].t.iloc[0]
    t90 = x[x.pos >= 0.9 * 1024].t.iloc[0]
    r["實驗2_上升時間10-90_s"] = round(t90 - t10, 3)
    r["實驗2_超越量_deg"] = round((x.pos.max() - 1024) * DEG, 2)
    r["實驗2_安定時間±0.9deg_s"] = round(x[(x.pos - 1024).abs() > 10].t.iloc[-1], 2)
    r["實驗2_峰值速度_rpm"] = round(x.vel.max() * RPM, 1)
    r["實驗2_峰值電流_mA"] = int(x.cur.max())
    r["實驗2_PWM滿載時間_ms"] = int((x.pwm >= PWM_FULL).sum() * 10)
    return r


summary = pd.DataFrame([metrics(n, *d) for n, d in RUNS])
summary.to_csv(D("data", "summary_metrics_v2_1106.csv"), index=False, encoding="utf-8-sig")
print(summary.T.to_string(header=False))

# ---- 圖 1：兩顆馬達三個實驗 ----
XL, XC = RUNS[0][1][0], RUNS[2][1][0]
titles = {1: "實驗 1：固定電流 80 mA", 2: "實驗 2：位置 0°→90°", 3: "實驗 3：固定速度 11.5 rpm"}
fig, ax = plt.subplots(3, 4, figsize=(16, 10.5), sharex=True)
for i, e in enumerate([1, 2, 3]):
    for df, c, lab in [(XL, BLUE, "標籤 XL330"), (XC, ORANGE, "標籤 XC330")]:
        x = df[df.exp == e]
        p0 = x[x.t < 0].pos.mean()
        ax[i, 0].plot(x.t, (x.pos - p0) * DEG, c=c, lw=2, label=lab)
        ax[i, 1].plot(x.t, x.vel * RPM, c=c, lw=2)
        ax[i, 2].plot(x.t, x.cur, c=c, lw=2)
        ax[i, 3].plot(x.t, x.pwm / PWM_FULL * 100, c=c, lw=2)
    ax[i, 0].set_ylabel(titles[e] + "\n\n位置變化 (°)", fontsize=11)
    for j, yl in enumerate(["", "速度 (rpm)", "回報電流 (mA)", "PWM (%)"]):
        a = ax[i, j]
        if yl:
            a.set_ylabel(yl)
        a.axvline(0, c="#bbb", lw=1, ls="--"); a.grid(alpha=.25)
        for s in ["top", "right"]:
            a.spines[s].set_visible(False)
for a in ax[2]:
    a.set_xlabel("時間（送出命令 = 0）[s]")
ax[0, 0].legend(frameon=False, loc="upper left", fontsize=10)
fig.suptitle("兩顆馬達比較（新版程式，通訊逾時 0 %）", fontsize=15, x=0.02, ha="left")
fig.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig(D("figures", "XL_vs_XC_v2_1106.png"), dpi=130)
plt.close(fig)

# ---- 圖 2：老師 §7 的三條位置曲線 ----
fig, ax = plt.subplots(3, 2, figsize=(13, 9), sharex=True, gridspec_kw={"height_ratios": [2, 1, 1]})
for j, (df, lab) in enumerate([(XL, "標籤 XL330"), (XC, "標籤 XC330")]):
    x = df[(df.exp == 2) & (df.t >= -0.1) & (df.t <= 0.8)]
    ax[0, j].step(x.t, x.goal * DEG, where="post", c=GRAY, lw=1.5, ls="--", label="Goal Position")
    ax[0, j].step(x.t, x.ptraj * DEG, where="post", c="#1baf7a", lw=1.5, label="Position Trajectory")
    ax[0, j].plot(x.t, x.pos * DEG, c=BLUE if j == 0 else ORANGE, lw=2.2, label="實際位置")
    ax[0, j].set_title(lab, fontsize=13, loc="left")
    ax[1, j].plot(x.t, x.pwm / PWM_FULL * 100, c="#444", lw=1.8)
    ax[1, j].axhline(100, c="#c33", lw=1, ls=":")
    ax[2, j].plot(x.t, x.cur, c="#444", lw=1.8)
    ax[0, j].set_ylabel("角度 (°)"); ax[1, j].set_ylabel("PWM (%)"); ax[2, j].set_ylabel("回報電流 (mA)")
    ax[2, j].set_xlabel("時間（送出命令 = 0）[s]")
    for a in ax[:, j]:
        a.grid(alpha=.25)
        for s in ["top", "right"]:
            a.spines[s].set_visible(False)
ax[0, 0].legend(frameon=False, loc="lower right", fontsize=10)
fig.suptitle("Profile Velocity = 0：軌跡在命令當下就跳到 90°，所以慢的不是 Profile，而是 PWM 滿載約 100 ms",
             fontsize=13, x=0.02, ha="left")
fig.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(D("figures", "position_step_3curves_1106.png"), dpi=130)
print("saved figures")
