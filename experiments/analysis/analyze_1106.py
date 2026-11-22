"""
analyze_1106.py — 標籤 XL330 vs 標籤 XC330 比較（2026-11-06）

用法（在 analysis 資料夾的上一層執行）：
    python3 analysis/analyze_1106.py

讀 data/ 裡的三份 CSV，輸出：
    figures/XL_vs_XC_1106.png      三個實驗 × 四個量的對照圖
    data/summary_metrics_1106.csv  每份資料的關鍵指標

資料格式（舊版 8 欄程式）：exp,t_ms,tick,pwm,cur,vel,pos,err
    pwm: 885 = 100 %   cur: 約 1 mA/單位（輸入端電流）
    vel: 0.229 rpm/單位  pos: 4096 = 一圈   err != 0 = 通訊逾時（該列數值是上一筆的重複，丟掉）
命令在 t = 1.0 s 送出，圖上以命令時刻為 0。
"""
import os
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
D = lambda *p: os.path.join(ROOT, *p)

# 中文字型：Mac 用 PingFang/Heiti，Linux 用 Noto CJK
for f in ["PingFang TC", "Heiti TC", "Arial Unicode MS", "Noto Sans CJK TC", "Noto Sans CJK JP"]:
    if any(f == x.name for x in fm.fontManager.ttflist):
        plt.rcParams["font.family"] = f
        break
plt.rcParams["axes.unicode_minus"] = False

PWM_FULL, RPM, DEG = 885, 0.229, 360 / 4096


def load_new(path):
    d = pd.read_csv(path)
    n_err = int((d.err != 0).sum())
    d = d[d.err == 0].copy()
    d["t"] = d.t_ms / 1000 - 1.0
    return d, n_err, len(d) + n_err


def load_ref(path):  # 10/30 已整理過的格式
    d = pd.read_csv(path)
    n_err = int((d.err != 0).sum())
    d = d[d.err == 0].copy()
    d["t"] = d.t_s - 1.0
    d = d.rename(columns={"pwm_raw": "pwm", "current_raw": "cur",
                          "velocity_raw": "vel", "position_raw": "pos"})
    return d, n_err, len(d) + n_err


SETS = [
    ("標籤 XC330（10/30 舊資料）", load_ref(D("data", "reference_label-XC330_20261030.csv")), "#9a9a95", (0, (4, 3)), 1.5),
    ("標籤 XL330", load_new(D("data", "label-XL330_20261106.csv")), "#2a78d6", "-", 2),
    ("標籤 XC330", load_new(D("data", "label-XC330_20261106.csv")), "#eb6834", "-", 2),
]


def metrics(name, df, n_err, n_all):
    r = {"資料": name, "逾時列數": n_err, "總列數": n_all, "逾時比例_%": round(100 * n_err / n_all, 1)}
    for e, tag in [(1, "實驗1_80mA"), (3, "實驗3_11.5rpm")]:
        s = df[(df.exp == e) & (df.t >= 2.0)]           # 穩態：命令後 2–3 s
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
    return r


rows = [metrics(n, *data) for n, data, *_ in SETS]
pd.DataFrame(rows).to_csv(D("data", "summary_metrics_1106.csv"), index=False, encoding="utf-8-sig")
print(pd.DataFrame(rows).T.to_string(header=False))

titles = {1: "實驗 1：固定電流 80 mA", 2: "實驗 2：位置 0°→90°", 3: "實驗 3：固定速度 11.5 rpm"}
fig, ax = plt.subplots(3, 4, figsize=(16, 10.5), sharex=True)
for i, e in enumerate([1, 2, 3]):
    for name, (df, _, _), c, ls, lw in SETS:
        x = df[df.exp == e]
        p0 = x[x.t < 0].pos.mean()
        ax[i, 0].plot(x.t, (x.pos - p0) * DEG, c=c, lw=lw, ls=ls, label=name)
        ax[i, 1].plot(x.t, x.vel * RPM, c=c, lw=lw, ls=ls)
        ax[i, 2].plot(x.t, x.cur, c=c, lw=lw, ls=ls)
        ax[i, 3].plot(x.t, x.pwm / PWM_FULL * 100, c=c, lw=lw, ls=ls)
    ax[i, 0].set_ylabel(titles[e] + "\n\n位置變化 (°)", fontsize=11)
    for j, yl in enumerate(["", "速度 (rpm)", "回報電流 (mA)", "PWM (%)"]):
        a = ax[i, j]
        if yl:
            a.set_ylabel(yl)
        a.axvline(0, c="#bbb", lw=1, ls="--")
        a.grid(alpha=.25)
        for s in ["top", "right"]:
            a.spines[s].set_visible(False)
for a in ax[2]:
    a.set_xlabel("時間（送出命令 = 0）[s]")
ax[0, 0].legend(frameon=False, loc="upper left", fontsize=10)
fig.suptitle("兩顆馬達比較：標籤 XL330 vs 標籤 XC330（已移除通訊逾時的資料點）", fontsize=15, x=0.02, ha="left")
fig.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig(D("figures", "XL_vs_XC_1106.png"), dpi=130)
print("saved figures/XL_vs_XC_1106.png")
