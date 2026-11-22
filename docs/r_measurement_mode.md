# XL330 等效電阻 R 手動量測模式

在 `firmware/XL330_STM32` 裡用編譯開關 `MOTOR_R_TEST` 切換的手動量測模式。
在序列埠監控視窗一次輸入一個指令，STM32 執行後印出讀值，由人工記錄；**不會自動或連續量測**。

- 板子：NUCLEO-F446RE
- 馬達：DYNAMIXEL XL330-M288-T，Protocol 2.0，預設 ID 1、1 Mbps（Return Delay 建議 0）
- 通訊：沿用 `Src/dxl.c`（USART1 單線半雙工，PA9 / D8）
- 程式：`Inc/r_measure.h`、`Src/r_measure.c`；`main.c` 只在開機時多一個 `#if MOTOR_R_TEST` 分支

## 1. 切換到量測模式

兩種方法擇一：

**方法 A：改標頭檔**（最簡單）
1. 打開 `firmware/XL330_STM32/Inc/r_measure.h`
2. 把 `#define MOTOR_R_TEST 0` 改成 `#define MOTOR_R_TEST 1`
3. 在 STM32CubeIDE 按 Build，然後 Run 燒錄

**方法 B：CubeIDE 前置處理器符號**（不用改程式碼）
1. 專案按右鍵 → Properties → C/C++ Build → Settings → MCU GCC Compiler → Preprocessor
2. Define symbols 新增 `MOTOR_R_TEST=1`
3. Build、Run

要回到原本的 v4 韌體：改回 `0`（或刪掉符號），重新 Build 並燒錄。
`MOTOR_R_TEST = 0` 時 `r_measure.c` 不會產生任何程式碼，原本的功能完全不變。

> 不要從 `.ioc` 重新產生程式碼後覆蓋 `main.c`；新增的兩行在 `USER CODE` 區塊內，正常重新產生會保留。

## 2. 序列埠設定

| 項目 | 設定 |
|---|---|
| 連接埠 | ST-LINK Virtual COM Port（Windows 裝置管理員 → 連接埠 (COM 和 LPT)） |
| 鮑率 | 115200 |
| 格式 | 8 資料位元、無同位、1 停止位元（8N1） |
| 行結尾 | LF、CR 或 CRLF 都可以 |
| 文字編碼 | UTF-8（輸出有中文） |
| 大小寫 | 不分（`STALL 0.3 2000` 也可以） |

可用 VS Code Serial Monitor、PuTTY、Tera Term、Arduino IDE 序列埠監控視窗等。
輸入的字元會回顯；Backspace 可刪字。

## 3. 指令

開機時會對目前 ID 執行 Goal PWM = 0、關扭力，然後印出指令列表，不會自動開始量測。

| 指令 | 作用 |
|---|---|
| `help` | 列出所有指令 |
| `ping` | 確認馬達有回應 |
| `id <n>` | 切換要操作的馬達 ID（0～252） |
| `info` | 印 Operating Mode(11)、PWM Limit(36)、Current Limit(38)、溫度(146) |
| `pwmmode` | 關扭力 → Operating Mode 設為 16（PWM）→ 讀回確認 |
| `read` | 讀一次 d、電流、電壓、溫度、位置 |
| `stall <d> <ms>` | 堵轉一次（見下） |
| `off` | Goal PWM 歸零並關扭力 |

### stall <d> <ms>

**執行前馬達輸出軸一定要固定在夾具上。**

- 限制：`-1 ≤ d ≤ 1`；`|d| > 0.6` 時 `ms` 為 400～1000，`|d| ≤ 0.6` 時 `ms` 為 400～5000，超出就拒絕
- 執行前檢查：Operating Mode 必須是 16（不是就請先 `pwmmode`）；溫度 ≥ 60 °C 拒絕並提示先冷卻
- 流程：Goal PWM = 0 → 開扭力 → Bus Watchdog 200 ms → Goal PWM = d × 885 → 每 2 ms 讀一次狀態，共 `ms` 毫秒 → Goal PWM = 0 → 關扭力
- 計算：前 200 ms 的資料丟掉；其餘取 |d| = |Present PWM| / 885、|電流|、電壓的平均，以及溫度最大值；整段期間位置的最大減最小 = moved
- 輸出：n、d、V、I、T、moved，以及參考值 V/I、d²·V/I（Ω）
- 警告：
  - moved > 5 pulse →「馬達有轉動，這筆不能用」
  - 平均電流 > 0.98 × Current Limit →「電流貼近 Current Limit，這筆不能用」
  - 中途溫度到 60 °C → 提前停止，這筆不能用

### 安全機制
- 通訊錯誤（逾時、封包錯誤、CRC 錯誤、馬達回報錯誤）：印出錯誤原因並關扭力
- 馬達回報 Hardware Error（狀態封包的警示位元）：讀位址 70 印出原因（電壓、過熱、編碼器、電氣衝擊、過載）並關扭力；要清除需重新上電或用 Wizard Reboot
- 堵轉期間開 Bus Watchdog 200 ms：板子當掉或線鬆掉，馬達會自己停
- `d × 885` 超過 PWM Limit 時印警告並改用 PWM Limit

## 4. 輸出範例

> 以下數字只是格式示意，不是實際量測值。

```
# XL330 等效電阻 R 手動量測模式（MOTOR_R_TEST）
# DYNAMIXEL Protocol 2.0、1 Mbps、馬達 ID 1
# 開機：Goal PWM = 0，扭力已關閉
# 指令（115200 baud，一行一個指令，以換行結尾，不分大小寫）：
...
> ping
# ping ID 1：有回應，型號 1200（XL330-M288）
> info
# info ID 1：Operating Mode = 3、PWM Limit = 885（100.0 %）、Current Limit = 1750 mA、溫度 = 28 °C
> pwmmode
# pwmmode：已關扭力，Operating Mode 寫入 16、讀回 16 → 成功（PWM 模式）
> read
# read ID 1：d = 0.0000、電流 = 0.000 A、電壓 = 5.0 V、溫度 = 28 °C、位置 = 2051 pulse（速度 = 0 × 0.229 rpm）
> stall 0.3 2000
# stall ID 1：d = 0.300（Goal PWM = 266），2000 ms，前 200 ms 不採計，開始溫度 28 °C
# 結果 ID 1：n = 900、d = 0.3006、V = 4.982 V、I = 0.5871 A、T = 29 °C、moved = 1 pulse
# 參考值：V/I = 8.486 Ω、d²·V/I = 0.767 Ω（實際執行 2000 ms）
> stall 0.8 3000
# 拒絕執行：|d| > 0.6 時，ms 必須在 400～1000 之間
> off
# off：Goal PWM = 0，扭力已關閉
```

錯誤範例：
```
> read
# 錯誤：讀取狀態失敗，逾時，馬達沒有回應（代碼 -2）。執行關扭力。
# 警告：關閉程序結果 Goal PWM=-2、Watchdog=-2、關扭力=-2（0 = 成功）
# 關扭力失敗，請檢查接線與電源後輸入 off
```
