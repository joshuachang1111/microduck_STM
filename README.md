# microduck_STM

DYNAMIXEL XL330-M288 actuator characterization for the Microduck replication project (NTHU).
STM32 NUCLEO-F446RE drives the servo over DYNAMIXEL Protocol 2.0 and logs data to a PC.

## Folders
| Folder | Content |
|---|---|
| `firmware/XL330_STM32/` | STM32CubeIDE project (firmware v4). Open in STM32CubeIDE: File → Import → Existing Projects into Workspace |
| `experiments/` | Lab notes (`實驗紀錄_*.md`, `實驗結果總覽.md`), raw logs and summaries (`data/`), analysis scripts (`analysis/`), figures (`figures/`) |

## Wiring
| XL330 | Connect to |
|---|---|
| DATA | PA9 (Arduino D8), 1.6 kΩ pull-up to 3V3 |
| VDD | bench supply +5 V |
| GND | supply GND and Nucleo GND |

Console: ST-LINK virtual COM port, 115200 baud (`screen -L /dev/tty.usbmodemXXXX 115200`). Type `h` for commands.

## Firmware commands (v4)
`c <mA>` current · `p <deg>` position · `v <rpm>` velocity · `w <%> [s]` PWM · `t <%> <period s> [cycles]` PWM triangle · `s <deg> [P] [D] [I]` position step (500 Hz) · `i` status · `h` help

## Key results so far (label XL330, 5 V)
- Mode 3 position control: PWM = P/256 × position error (no inner current loop)
- Coil R = 2.55 Ω (4-wire), L ≈ 0.59 mH (0.5–2 kHz), L/R ≈ 0.23 ms
- PWM frequency 33.36 kHz
- Constant-speed voltage: ν₀ = 0.428 V·s/rad × ω (output side, 0.23–20 rpm)

See `experiments/實驗結果總覽.md` for details.
