# XL330 等效電阻 R 量測（堵轉）韌體

獨立的 STM32CubeIDE 專案（專案名稱 `XL330_Rmeasure`），不影響 `firmware/XL330_STM32`（v4）。
板子：NUCLEO-F446RE。DYNAMIXEL 通訊方式與 v4 相同：USART1 單線半雙工、PA9（D8）、Open-Drain、
1 Mbps、Protocol 2.0；電腦端走 USART2 → ST-LINK 虛擬 COM port，115200。

## 編譯與燒錄（STM32CubeIDE）
1. File → Import → General → Existing Projects into Workspace → 選 `experiments/R_measure`
   （**不要**勾 Copy projects into workspace）。可以和 `XL330_STM32` 放在同一個 workspace。
2. 專案上按右鍵 → Build Project（Debug）。
3. 接上 NUCLEO，Run → Run As → STM32 C/C++ Application（第一次會跳出設定視窗，直接 OK）。
4. 打開序列埠終端機：115200、8N1、行結尾送 LF 或 CR 都可以（建議 UTF-8，警告訊息有中文）。

> 不要從 `.ioc` 重新產生程式碼（Generate Code），`Src/main.c` 是手寫的。

## 接線（同 v4）
| XL330 | 接到 |
|---|---|
| DATA | PA9（D8），上拉電阻到 3V3 |
| VDD | 電源供應器 +5 V |
| GND | 電源供應器 GND 與 Nucleo GND |

馬達需先設定為 Protocol 2.0、1 Mbps、Return Delay 0。

## 指令
| 指令 | 作用 |
|---|---|
| `help` | 列出指令 |
| `ping` | 確認目前 ID 的馬達有回應 |
| `id <n>` | 切換馬達 ID（預設 1，0～252） |
| `info` | 讀 Operating Mode(11)、PWM Limit(36)、Current Limit(38, mA)、Present Temperature(146)（另附 Hardware Error(70)） |
| `pwmmode` | Goal PWM = 0、關扭力(64=0) → Operating Mode = 16 → 讀回確認 |
| `read` | 讀一次 Present PWM(124)、Current(126, mA)、Input Voltage(144, 0.1 V)、Temperature(146)、Position(132) |
| `stall <d> <ms>` | 堵轉一次（見下） |
| `off` | Goal PWM = 0、關扭力 |

開機時：Goal PWM = 0、關扭力，然後印出 help。

### stall <d> <ms>
- `-1 ≤ d ≤ 1`；`|d| > 0.6` 時 ms 限 400～1000，否則 400～5000；超出就拒絕。
- 開始前溫度 ≥ 60 °C 拒絕；Operating Mode 不是 16 也拒絕（請先 `pwmmode`）。
- 流程：Goal PWM = 0 → 開扭力 → Bus Watchdog 200 ms → Goal PWM = d×885 → 每 2 ms 讀一次 124～146，
  持續 ms（前 200 ms 不採計）→ Goal PWM = 0 → Watchdog 關 → 關扭力。
- 對採計的資料平均：`d = |Present PWM|/885`、`I = |Present Current|`（A）、`V`（V）；
  `T` = 期間最高溫度；`moved` = 整段期間 Position 最大減最小（pulse）。
- 印出 `V/I` 與 `d²·V/I`（Ω），以及一行可直接存成 CSV 的 `stall_csv,...`：
  ```
  stall_csv,id,d_cmd,ms,n,d,V,I,T,moved,VI,d2VI,err
  ```
- 警告：`moved > 5` →「馬達有轉動，這筆不能用」；`I > 0.98 × Current Limit` →「電流貼近 Current Limit，這筆不能用」。

### 額外的安全機制
- 堵轉期間開 Bus Watchdog 200 ms：板子當掉或線鬆掉，馬達會自己停。
- 堵轉期間溫度到 60 °C 就提前停止並關扭力（會印警告，這筆時間不足）。
- `d×885` 超過 PWM Limit 時印警告並限幅到 PWM Limit（馬達會拒絕超出範圍的 Goal PWM）。

## 檔案
- `Src/main.c`：指令列介面與量測流程（新寫）
- `Src/dxl.c`、`Inc/dxl.h`：Protocol 2.0 驅動，從 v4 原樣複製
- 其餘（HAL、CMSIS、startup、linker script、`.cproject`）從 `firmware/XL330_STM32` 複製，只改專案名稱
