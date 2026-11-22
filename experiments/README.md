# XL330-M288 測試程式（STM32 Nucleo-F446RE）

用 STM32 透過 DYNAMIXEL Protocol 2.0 控制 XL330-M288，並記錄三種實驗的電流、速度與位置資料（附時間戳）：
1. 固定電流命令
2. 固定位置命令（+90°）
3. 固定速度命令

## 檔案
| 檔案 | 放到哪裡 |
|---|---|
| `main.c` | **整份取代** `Core/Src/main.c`（推薦） |
| `dxl.h` | `Core/Inc/` |
| `dxl.c` | `Core/Src/` |
| `main_user_code.c` | 參考用，不要放進專案；只有想自己分段貼進 main.c 時才用 |

> `main.c` 會自己設定時脈（HSI 16 MHz）、PC13、USART2 與 USART1 半雙工（PA9 Open Drain），
> 所以只要專案本身有 HAL 的 UART 模組（例如從 Lab1 專案複製來的）就能直接用。
> 已用 arm-none-eabi-gcc + ST 官方 HAL 完整編譯、連結，無錯誤與警告。

## 馬達設定（先用 DYNAMIXEL Wizard）
- ID = 1、Baud Rate = 1 Mbps、Protocol 2.0、Return Delay Time = 0 µs
- 新馬達出廠是 57600 bps、Return Delay 500 µs，要先用 Wizard 改

## 接線
| 從 | 到 |
|---|---|
| 電源供應器 +5 V（限流 1 A） | 馬達 V |
| 電源供應器 GND | 馬達 G |
| 馬達 G | Nucleo GND（共地） |
| 馬達 D | Nucleo D8（PA9） |
| 4.7 kΩ 電阻 | D8 ↔ Nucleo 3V3（上拉） |

- 馬達**不要**接到 Nucleo 的 5 V，也**不需要**馬達驅動板
- 不要讓 OpenRB 和 STM32 同時接在同一條匯流排上

## CubeMX 設定
- PC13 = GPIO_Input（USER 按鈕）
- USART1：Single Wire (Half-Duplex)、PA9、1000000 bps、8N1；PA9 GPIO 設為 Open Drain、No pull
- USART2：115200（ST-LINK 虛擬序列埠，用來印資料）
- 時脈：HSI 16 MHz（沿用 Lab1 設定）

## 使用方式
1. 燒錄後打開序列埠（115200）。開機會印出：
   `# ping id=1 result=0 model=1200`（result=0 代表通訊成功）
2. 按藍色 USER 按鈕，依序執行實驗 1 → 2 → 3 → 1 …
3. 每個實驗：1 秒靜止基準 → 送出命令 → 共記錄 4 秒，100 Hz

## 輸出格式
```
exp,t_ms,tick,pwm,cur,vel,pos,vtraj,ptraj,goal,err
```
| 欄位 | 意義 | 換算 |
|---|---|---|
| exp | 實驗編號 1/2/3 | |
| t_ms | STM32 時間（從實驗開始算） | ms |
| tick | 馬達內部 Realtime Tick | ms（0～32767 循環） |
| pwm | Present PWM（124） | 885 = 100% |
| cur | Present Current（126） | 約 1 mA/單位（以 e-Manual 為準） |
| vel | Present Velocity（128） | 0.229 rpm/單位 |
| pos | Present Position（132） | 4096 = 一圈 |
| vtraj | Velocity Trajectory（136），Profile 產生的速度軌跡 | 0.229 rpm/單位 |
| ptraj | Position Trajectory（140），Profile 產生的位置軌跡 | 4096 = 一圈 |
| goal | 當下送進馬達的目標值（命令前 / 命令後） | 依模式 |
| err | 讀取結果 | 0 = 正常 |

以 `#` 開頭的行是說明與設定：
- `# cfg[boot]`、`# cfg[exp]`：開機與每次實驗前讀回的設定（Operating Mode、各項 Limit、PID 增益、Profile 等）
- `# mode readback`：切換模式後讀回確認

送出命令的瞬間，Nucleo 上的綠色 LED（PA5 / D13）會亮起，實驗結束時熄滅，可當示波器或影片的時間標記。

## 終端機指令
`c 80`（電流 mA）、`p 90`（位置 deg）、`v 12`（速度 rpm）、`h`（說明）；或按藍色按鈕跑預設實驗 1→2→3。

## Ping 失敗排查
| result | 意思 | 處理 |
|---|---|---|
| −2 | 逾時，沒收到回應 | 檢查 DATA、共地、上拉電阻、馬達電源；或調整 Return Delay Time |
| −3 / −4 | 封包內容錯誤 | 確認 baud rate 一致、PA9 為 Open Drain |
| 正數 | 馬達回報的錯誤碼 | 查 e-Manual 的 Status Packet Error |

> 封包格式與 CRC 已用模擬 UART 驗證（Ping 封包與 ROBOTIS 手冊範例一致），程式可完整編譯，但尚未在實機上完整測試。

## v3（2026-11-06）新增功能

### 新指令
| 指令 | 作用 | 範例 |
|---|---|---|
| `w <PWM%> [秒]` | PWM 模式：靜止 1 s → PWM 指定值持續 N 秒（預設 3，1～10）→ PWM 0 再記 1 s | `w 30 3` |
| `t <振幅%> <週期s> [次數]` | PWM 三角波：0 → +A → 0 → −A → 0，週期 2～30 s，1～5 次 | `t 30 10 2` |
| `i` | 印出目前電壓、溫度、位置、Hardware Error | `i` |

`w` 結束時會印出 `# summary (last 1 s of command): pwm=.. cur=.. vel=.. volt=..`，是命令最後 1 秒的平均值。

### 輸出多兩欄
```
exp,t_ms,tick,pwm,cur,vel,pos,vtraj,ptraj,goal,volt,temp,err
```
- `exp`：1 電流、2 位置、3 速度、**4 PWM 步階、5 PWM 三角波**
- `volt`：Present Input Voltage（位址 144），0.1 V/單位（50 = 5.0 V）
- `temp`：Present Temperature（位址 146），°C

### 安全機制
- 馬達 **≥ 50 °C 不開始**新的測試（會印出提示，等它降溫）
- 測試中 **≥ 60 °C 自動停止**並關扭矩（馬達上限 70 °C）
- 測試期間開啟 **Bus Watchdog 200 ms**：STM32 當機或線鬆掉，馬達會自己停
- 每次測試結束印出 `# status: ... hw_error=0x..`（0x00 = 正常）

### 其他
- `dxl.c` 的接收改成直接讀 USART 暫存器（`dxl_rx()`），解決新版 HAL 在 1 Mbps 下封包被截斷的問題

## v4（2026-11-08）：位置階躍 + 控制參數（老師 11/08 信第 2 點）

新指令 `s`：位置模式的小步階，500 Hz 記錄，可以指定 Position P / D / I gain。

```
s 10                 從 180° 附近往 +10° 階躍，預設 P=400 D=0 I=0
s -10                往 −10°
s 10 1600            P=1600（D、I 用預設 0）
s 10 1600 500        P=1600 D=500
s 10 1600 500 100    P=1600 D=500 I=100
```

流程：位置模式、預設增益移到 180° 停穩 → 寫入測試增益並讀回（`# gains readback`）→
目標設成目前位置（誤差歸零）等 300 ms → 開始記錄 → 200 ms 時送出 起點 + 步階 → 共記錄 2 s → 關扭力 → 印出全部資料 → 印 `# step summary`。

輸出欄位（9 欄）：`exp,t_ms,tick,pwm,cur,vel,pos,goal,err`，exp = 6，t_ms 每 2 ms 一筆，步階在 t_ms = 200。

`# step summary`：上升時間 10–90 %、超越量 %、安定時間（±5 % 步階，至少 ±2 raw）、
穩態誤差（最後 200 ms 平均）、PWM 最大值、PWM 飽和的時間、讀取錯誤數、`late_slots`（沒趕上 2 ms 的次數，應為 0）。

不飽和的條件（由 11/06 資料得到 PWM = P/256 × 位置誤差[raw]）：步階 < 885 × 256 / P raw，
P=400 約 50°、800 約 25°、1600 約 12°、3200 約 6°。

其他改動：`dxl.c` 的 CRC 改查表（快約 8 倍，結果和舊版逐位元計算完全相同）；新增 `dxl_read_motion()` 只讀 120–135（16 bytes）。
