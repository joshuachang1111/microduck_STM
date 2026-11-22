/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : XL330-M288 test on NUCLEO-F446RE (DYNAMIXEL Protocol 2.0)
  ******************************************************************************
  * 用法：整份取代 Src/main.c（dxl.h 放 Inc，dxl.c 放 Src）
  *
  * 接線：
  *   XL330 DATA ── PA9 (Arduino D8) ── 1.6kΩ ── 3V3
  *   XL330 V    ── 電源供應器 +5V
  *   XL330 G    ── 電源供應器 GND ── Nucleo GND
  *
  * 在終端機輸入指令（按 Enter）：c 80 = 電流 80 mA、p 90 = 位置 90°、v 12 = 速度 12 rpm、
  *   w 30 3 = PWM 30 % 持續 3 秒、t 30 10 2 = PWM 三角波 ±30 % 週期 10 秒 2 次、i = 狀態、h = 說明
  * v3（2026-11-06）：加 PWM 指令（exp 4）、PWM 三角波（exp 5）、記錄電壓與溫度、
  *   溫度保護（50 °C 以上不開始、60 °C 自動停止）、測試期間 Bus Watchdog 200 ms
  * 或按藍色 USER 按鈕：依序執行預設實驗 1 → 2 → 3 → 1 …
  *   1 = 固定電流、2 = 固定位置（+90°）、3 = 固定速度
  *   每個實驗：先 1 秒靜止基準，再送命令，共記錄 4 秒（100 Hz）
  * 輸出（UART2 = ST-LINK 虛擬序列埠，115200）：
  *   exp,t_ms,tick,pwm,cur,vel,pos,vtraj,ptraj,goal,volt,temp,err   (lines starting with # = info/settings)
  *   volt = 0.1 V/unit, temp = deg C
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "dxl.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

#define DXL_ID         1
#define DXL_BAUD       1000000
#define GOAL_CURRENT   80     /* raw (XL330: ~1 mA/unit), keep well under Current Limit */
#define GOAL_VELOCITY  50     /* raw, 0.229 rpm/unit -> ~11.5 rpm */
#define POS_START      0      /* raw, exp 2 pre-positions here first (0 deg)       */
#define POS_GOAL       1024   /* raw, exp 2 fixed position command: 1024 = +90 deg  */
                              /* (0.088 deg/unit, 4096/rev, same angle as Wizard)   */
#define CUR_MAX        300    /* clamp for typed current command [mA] (= Current Limit) */
#define VEL_MAX_RAW    445    /* clamp for typed velocity command (445*0.229 = ~102 rpm) */
#define T_IDLE_MS      1000   /* baseline before the command */
#define T_TOTAL_MS     4000   /* total logging time per experiment */
#define TS_MS          10     /* 100 Hz logging */
#define T_TAIL_MS      1000   /* PWM tests: logging after the command goes back to 0 */
#define PWM_FULL       885    /* Goal/Present PWM raw value for 100 % */
#define TEMP_START_MAX_C 50   /* refuse to start a test above this temperature */
#define TEMP_STOP_C    60     /* stop a running test at this temperature (limit is 70) */
#define WATCHDOG_20MS  10     /* Bus Watchdog during a test: 10 x 20 ms = 200 ms */
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;    /* DYNAMIXEL bus (half-duplex, PA9) */
UART_HandleTypeDef huart2;    /* printf -> ST-LINK virtual COM */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

static uint32_t le_u(const uint8_t *b, int n)
{
  uint32_t v = 0;
  for (int i = n - 1; i >= 0; i--) v = (v << 8) | b[i];
  return v;
}

/* Read back and print the servo settings (lines start with '#', so CSV tools skip them).
 * EEPROM 9..55 and RAM 76..115 are read in two packets. */
static void print_config(const char *tag)
{
  uint8_t e[47], m[40];
  int r1 = dxl_read(DXL_ID, 9, e, 47);    /* addr 9..55   */
  int r2 = dxl_read(DXL_ID, 76, m, 40);   /* addr 76..115 */
  if (r1 != DXL_OK || r2 != DXL_OK) {
    printf("# cfg[%s] read failed: %d %d\r\n", tag, r1, r2);
    return;
  }
#define E(a, n) le_u(&e[(a) - 9], (n))
#define M(a, n) le_u(&m[(a) - 76], (n))
  printf("# cfg[%s] ReturnDelay=%lu(x2us) DriveMode=%lu OperatingMode=%lu TempLimit=%lu "
         "MaxV=%lu MinV=%lu PWMLimit=%lu CurrentLimit=%lu VelocityLimit=%lu "
         "MaxPos=%lu MinPos=%lu\r\n", tag,
         E(9,1), E(10,1), E(11,1), E(31,1), E(32,2), E(34,2), E(36,2), E(38,2), E(44,4),
         E(48,4), E(52,4));
  printf("# cfg[%s] VelI=%lu VelP=%lu PosD=%lu PosI=%lu PosP=%lu FF2=%lu FF1=%lu "
         "GoalPWM=%ld GoalCurrent=%ld GoalVelocity=%ld ProfileAcc=%lu ProfileVel=%lu\r\n", tag,
         M(76,2), M(78,2), M(80,2), M(82,2), M(84,2), M(88,2), M(90,2),
         (long)(int16_t)M(100,2), (long)(int16_t)M(102,2), (long)(int32_t)M(104,4),
         M(108,4), M(112,4));
#undef E
#undef M
}

static int32_t round_f(float x) { return (int32_t)(x >= 0 ? x + 0.5f : x - 0.5f); }

/* ---------------- one test = one logged run ----------------
 * exp 1 = current, 2 = position, 3 = velocity  (step command, 4 s total)
 * exp 4 = PWM step      (1 s idle, PWM for active_ms, then PWM 0 for tail_ms)
 * exp 5 = PWM triangle  (1 s idle, triangle 0 -> +A -> 0 -> -A -> 0 per period, then 0) */
typedef struct {
  int      exp;
  int32_t  goal;        /* step value (raw), or triangle amplitude (raw PWM) */
  uint32_t active_ms;   /* how long the command is applied */
  uint32_t tail_ms;     /* logging after the command is set back to 0 */
  uint32_t period_ms;   /* exp 5 only */
  int      preposition; /* exp 2 only: move to POS_START first (not logged) */
} test_t;

static uint8_t exp_mode(int e)
{
  switch (e) {
    case 1:  return DXL_MODE_CURRENT;
    case 2:  return DXL_MODE_POSITION;
    case 3:  return DXL_MODE_VELOCITY;
    default: return DXL_MODE_PWM;
  }
}

static int write_goal(int e, int32_t g)
{
  switch (e) {
    case 1:  return dxl_write2(DXL_ID, DXL_ADDR_GOAL_CURRENT,  (int16_t)g);
    case 2:  return dxl_write4(DXL_ID, DXL_ADDR_GOAL_POSITION, g);
    case 3:  return dxl_write4(DXL_ID, DXL_ADDR_GOAL_VELOCITY, g);
    default: return dxl_write2(DXL_ID, DXL_ADDR_GOAL_PWM,      (int16_t)g);
  }
}

/* triangle 0 -> +amp -> 0 -> -amp -> 0 over one period */
static int32_t tri_goal(int32_t amp, uint32_t period_ms, uint32_t t_ms)
{
  float u = (float)(t_ms % period_ms) / (float)period_ms;
  float y = (u < 0.25f) ? 4.0f * u : (u < 0.75f) ? 2.0f - 4.0f * u : 4.0f * u - 4.0f;
  return round_f((float)amp * y);
}

static int read_state_retry(dxl_state_t *s)
{
  int r = DXL_ERR_TIMEOUT;
  for (int k = 0; k < 5 && r != DXL_OK; k++) {
    r = dxl_read_state(DXL_ID, s);
    if (r != DXL_OK) HAL_Delay(10);
  }
  return r;
}

/* one line: voltage, temperature, position, hardware error */
static void print_status(void)
{
  dxl_state_t s = {0};
  uint8_t hw = 0xFF;
  int r1 = read_state_retry(&s);
  int r2 = dxl_read(DXL_ID, DXL_ADDR_HW_ERROR, &hw, 1);
  printf("# status: volt=%u.%u V temp=%u C pos=%ld hw_error=0x%02X (read %d %d)\r\n",
         s.volt / 10, s.volt % 10, s.temp, (long)s.position, hw, r1, r2);
}

static void run_experiment(const test_t *t)
{
  dxl_state_t s = {0};
  int r;
  const int e = t->exp;

  /* 0) do not start on a hot motor */
  r = read_state_retry(&s);
  if (r != DXL_OK) { printf("# read failed: %d\r\n", r); return; }
  if (s.temp >= TEMP_START_MAX_C) {
    printf("# motor is %u C (>= %d C): let it cool down first\r\n", s.temp, TEMP_START_MAX_C);
    return;
  }

  /* 1) torque off -> select mode (mode can only change with torque off) */
  dxl_write1(DXL_ID, DXL_ADDR_TORQUE_ENABLE, 0);
  HAL_Delay(10);
  uint8_t mode = exp_mode(e);
  r = dxl_write1(DXL_ID, DXL_ADDR_OPERATING_MODE, mode);
  if (r != DXL_OK) { printf("# set mode failed: %d\r\n", r); return; }
  /* Operating Mode is stored in EEPROM: give the servo time before the next packet */
  HAL_Delay(50);
  {                                        /* read back: did the mode really change? */
    uint8_t mr = 0xFF;
    int rr = DXL_ERR_TIMEOUT;
    for (int k = 0; k < 5 && rr != DXL_OK; k++) {
      rr = dxl_read(DXL_ID, DXL_ADDR_OPERATING_MODE, &mr, 1);
      if (rr != DXL_OK) HAL_Delay(10);
    }
    printf("# mode readback: wrote %u, read %u (result %d)\r\n", mode, mr, rr);
    if (rr != DXL_OK || mr != mode) { printf("# mode mismatch, abort\r\n"); return; }
  }
  print_config("exp");

  /* 2) make the goal "do nothing" before enabling torque */
  int32_t hold_pos = 0;                    /* exp 2: goal before the step */
  if (e == 2) {
    r = read_state_retry(&s);
    if (r != DXL_OK) { printf("# read failed: %d\r\n", r); return; }
    /* Position Control Mode only accepts goals in 0..4095 (one turn) */
    if (s.position < 0 || s.position > 4095) {
      printf("# present position %ld is outside 0..4095 in position mode, abort\r\n",
             (long)s.position);
      return;
    }
    hold_pos = s.position;
    r = write_goal(2, hold_pos);           /* hold where it is */
  } else {
    r = write_goal(e, 0);
  }
  if (r != DXL_OK) { printf("# zero goal failed: %d\r\n", r); return; }
  for (int k = 0; k < 5; k++) {            /* torque on (retry) */
    r = dxl_write1(DXL_ID, DXL_ADDR_TORQUE_ENABLE, 1);
    if (r == DXL_OK) break;
    HAL_Delay(10);
  }
  if (r != DXL_OK) { printf("# torque on failed: %d\r\n", r); return; }

  /* exp 2: move to POS_START first (not logged), so the logged part is a clean
   * POS_START -> goal step, e.g. 0 deg -> +90 deg */
  if (e == 2 && t->preposition) {
    printf("# position: moving to start %d (%d deg) ...\r\n", POS_START, POS_START * 88 / 1000);
    hold_pos = POS_START;
    r = write_goal(2, POS_START);
    if (r != DXL_OK) { printf("# start goal failed: %d\r\n", r); return; }
    uint32_t tw = HAL_GetTick();
    while (HAL_GetTick() - tw < 3000) {           /* wait until settled (max 3 s) */
      if (dxl_read_state(DXL_ID, &s) == DXL_OK &&
          s.position > POS_START - 10 && s.position < POS_START + 10 &&
          s.velocity == 0) break;
      HAL_Delay(20);
    }
    HAL_Delay(300);
    printf("# position: start=%ld, command at t=1s -> goal=%ld (%ld deg)\r\n",
           (long)s.position, (long)t->goal, (long)(t->goal * 88 / 1000));
  }

  /* 3) safety: if the STM32 stops talking for 200 ms, the servo stops by itself */
  r = dxl_write1(DXL_ID, DXL_ADDR_BUS_WATCHDOG, WATCHDOG_20MS);
  if (r != DXL_OK) printf("# bus watchdog on failed: %d\r\n", r);

  /* 4) log at 100 Hz */
  const uint32_t t_cmd = T_IDLE_MS, t_rel = T_IDLE_MS + t->active_ms;
  const uint32_t t_end = t_rel + t->tail_ms;
  printf("# exp %d start\r\n", e);
  printf("exp,t_ms,tick,pwm,cur,vel,pos,vtraj,ptraj,goal,volt,temp,err\r\n");
  uint32_t t0 = HAL_GetTick(), next = t0;
  int cmd_sent = 0, released = 0, hot = 0;
  int32_t goal_now = (e == 2) ? hold_pos : 0;   /* goal currently in the servo */
  int64_t sum_pwm = 0, sum_cur = 0, sum_vel = 0, sum_volt = 0;
  int32_t n_sum = 0;
  while (HAL_GetTick() - t0 < t_end) {
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - next) < 0) continue;
    next += TS_MS;
    uint32_t tin = now - t0;
    int wr = DXL_OK;

    if (tin >= t_cmd && tin < t_rel) {
      if (e == 5) {                                    /* triangle: new goal every sample */
        goal_now = tri_goal(t->goal, t->period_ms, tin - t_cmd);
        wr = write_goal(e, goal_now);
        cmd_sent = 1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      } else if (!cmd_sent) {                          /* step: send once */
        wr = write_goal(e, t->goal);
        goal_now = t->goal;
        cmd_sent = 1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   /* LED / scope marker */
      }
    } else if (tin >= t_rel && t->tail_ms > 0 && !released) {
      wr = write_goal(e, 0);                           /* PWM tests: back to 0 */
      goal_now = 0;
      released = 1;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    if (wr != DXL_OK) printf("# command write failed: %d\r\n", wr);

    r = dxl_read_state(DXL_ID, &s);
    printf("%d,%lu,%u,%d,%d,%ld,%ld,%ld,%ld,%ld,%u,%u,%d\r\n", e, (unsigned long)tin,
           (unsigned)s.tick, s.pwm, s.current, (long)s.velocity, (long)s.position,
           (long)s.vel_traj, (long)s.pos_traj, (long)goal_now, (unsigned)s.volt,
           (unsigned)s.temp, r);

    /* exp 4: average of the last 1 s of the command, printed at the end */
    if (e == 4 && r == DXL_OK && tin + 1000 >= t_rel && tin < t_rel) {
      sum_pwm += s.pwm; sum_cur += s.current; sum_vel += s.velocity; sum_volt += s.volt;
      n_sum++;
    }
    if (r == DXL_OK && s.temp >= TEMP_STOP_C) { hot = 1; break; }
  }

  /* 5) stop */
  write_goal(e, (e == 2) ? goal_now : 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  dxl_write1(DXL_ID, DXL_ADDR_BUS_WATCHDOG, 0);
  dxl_write1(DXL_ID, DXL_ADDR_TORQUE_ENABLE, 0);
  printf("# exp %d end\r\n", e);
  if (hot) printf("# STOPPED: temperature %u C >= %d C\r\n", s.temp, TEMP_STOP_C);
  if (e == 4 && n_sum > 0) {
    int32_t pwm_x10  = (int32_t)(sum_pwm * 1000 / PWM_FULL / n_sum);   /* 0.1 % */
    int32_t cur      = (int32_t)(sum_cur / n_sum);                     /* mA    */
    int32_t vel_x100 = (int32_t)(sum_vel * 229 / 10 / n_sum);          /* 0.01 rpm */
    int32_t volt     = (int32_t)(sum_volt / n_sum);                    /* 0.1 V */
    printf("# summary (last 1 s of command): pwm=%ld.%ld %% cur=%ld mA vel=%s%ld.%02ld rpm volt=%ld.%ld V\r\n",
           (long)(pwm_x10 / 10), (long)labs(pwm_x10 % 10), (long)cur,
           (vel_x100 < 0) ? "-" : "", (long)labs(vel_x100 / 100), (long)labs(vel_x100 % 100),
           (long)(volt / 10), (long)(volt % 10));
  }
  print_status();
}

/* ---------------- terminal commands (type in screen, press Enter) ---------------- */
static char line[32];
static int  line_len = 0;

static void print_help(void)
{
  printf("# commands (type then Enter):\r\n");
  printf("#   c 80        constant current  [mA]   (-%d..%d)\r\n", CUR_MAX, CUR_MAX);
  printf("#   p 90        fixed position    [deg]  (0..359, absolute, same angle as Wizard)\r\n");
  printf("#   v 12        constant velocity [rpm]  (negative = reverse)\r\n");
  printf("#   w 30 3      PWM 30 %% for 3 s, then PWM 0   (-100..100 %%, 1..10 s, default 3 s)\r\n");
  printf("#   t 30 10 2   PWM triangle +-30 %%, period 10 s, 2 cycles (2..30 s, 1..5 cycles)\r\n");
  printf("#   i           status: voltage, temperature, position, hardware error\r\n");
  printf("#   h           this help\r\n");
  printf("# or press USER button: preset experiments 1 -> 2 -> 3\r\n");
}

/* returns 1 when a full line has been typed into 'line' */
static int console_getline(void)
{
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) ||
         __HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
    uint8_t c = (uint8_t)(huart2.Instance->DR & 0xFF);   /* reading DR also clears ORE */
    if (c == '\r' || c == '\n') {
      if (line_len == 0) continue;
      line[line_len] = 0;
      line_len = 0;
      printf("\r\n");
      return 1;
    }
    if ((c == 0x7F || c == 0x08) && line_len > 0) { line_len--; printf("\b \b"); continue; }
    if (c >= 32 && c < 127 && line_len < (int)sizeof(line) - 1) {
      line[line_len++] = (char)c;
      HAL_UART_Transmit(&huart2, &c, 1, 10);              /* echo */
    }
  }
  return 0;
}

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

static void handle_command(const char *cmd)
{
  while (*cmd == ' ') cmd++;
  char k = (char)tolower((unsigned char)cmd[0]);
  const char *p = cmd + 1;
  char *end;
  float a[3] = {0, 0, 0};
  int n = 0;
  while (n < 3) {                          /* up to 3 numbers after the letter */
    float x = strtof(p, &end);
    if (end == p) break;
    a[n++] = x;
    p = end;
  }
  test_t t = { 0, 0, T_TOTAL_MS - T_IDLE_MS, 0, 0, 0 };

  if (k == 'h' || k == '?') { print_help(); return; }
  if (k == 'i') { print_status(); return; }
  if (n == 0 || (k != 'c' && k != 'p' && k != 'v' && k != 'w' && k != 't')) {
    printf("# unknown command: \"%s\"  (type h for help)\r\n", cmd);
    return;
  }
  if (k == 'c') {
    t.exp = 1;
    t.goal = round_f(clampf(a[0], -CUR_MAX, CUR_MAX));
    printf("# command: current %ld mA (raw %ld)\r\n", (long)t.goal, (long)t.goal);
  } else if (k == 'p') {
    if (a[0] < 0 || a[0] >= 360) { printf("# position must be 0..359 deg\r\n"); return; }
    t.exp = 2;
    t.goal = round_f(a[0] * 4096.0f / 360.0f);   /* 90 deg -> 1024 */
    if (t.goal > 4095) t.goal = 4095;
    printf("# command: position %ld deg (raw %ld)\r\n", (long)round_f(a[0]), (long)t.goal);
  } else if (k == 'v') {
    t.exp = 3;
    t.goal = round_f(clampf(a[0] / 0.229f, -VEL_MAX_RAW, VEL_MAX_RAW));
    printf("# command: velocity %ld rpm (raw %ld, 0.229 rpm/unit)\r\n", (long)round_f(a[0]), (long)t.goal);
  } else if (k == 'w') {
    float pct = clampf(a[0], -100.0f, 100.0f);
    float sec = (n >= 2) ? clampf(a[1], 1.0f, 10.0f) : 3.0f;
    t.exp = 4;
    t.goal = round_f(pct * PWM_FULL / 100.0f);
    t.active_ms = (uint32_t)round_f(sec * 1000.0f);
    t.tail_ms = T_TAIL_MS;
    printf("# command: PWM %ld %% (raw %ld) for %ld ms\r\n",
           (long)round_f(pct), (long)t.goal, (long)t.active_ms);
  } else {                                 /* 't' */
    if (n < 2) { printf("# usage: t <amplitude %%> <period s> [cycles]\r\n"); return; }
    float amp = clampf(a[0], 1.0f, 100.0f);
    float per = clampf(a[1], 2.0f, 30.0f);
    int cyc = (n >= 3) ? (int)clampf(a[2], 1.0f, 5.0f) : 1;
    t.exp = 5;
    t.goal = round_f(amp * PWM_FULL / 100.0f);
    t.period_ms = (uint32_t)round_f(per * 1000.0f);
    t.active_ms = t.period_ms * (uint32_t)cyc;
    t.tail_ms = T_TAIL_MS;
    printf("# command: PWM triangle +-%ld %% (raw %ld), period %ld ms, %d cycle(s)\r\n",
           (long)round_f(amp), (long)t.goal, (long)t.period_ms, cyc);
  }
  run_experiment(&t);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);       /* print immediately (prompt, echo) */
  dxl_init(&huart1);
  HAL_Delay(200);                         /* let the servo boot */
  printf("\r\n# XL330 test start\r\n");

  uint16_t model = 0;
  int pr = DXL_ERR_TIMEOUT;
  /* keep pinging every 100 ms until the servo answers (easy to see on a scope) */
  for (int tries = 1; pr != DXL_OK; tries++) {
    pr = dxl_ping(DXL_ID, &model);
    printf("# ping try %d: result=%d model=%u, rx bytes=%u :", tries, pr,
           (unsigned)model, (unsigned)dxl_dbg_rx_got);
    for (int i = 0; i < dxl_dbg_rx_got && i < 14; i++) printf(" %02X", dxl_dbg_rx[i]);
    printf("\r\n");
    if (pr != DXL_OK) HAL_Delay(100);
  }
  if (pr != DXL_OK)
    printf("# ping failed: check wiring / 5V power / baud / ID, then press RESET\r\n");
  print_config("boot");
  print_status();
  print_help();
  printf("> ");

  int exp_no = 1;
  GPIO_PinState btn_prev = GPIO_PIN_SET;
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN 3 */
    GPIO_PinState btn = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
    if (btn_prev == GPIO_PIN_SET && btn == GPIO_PIN_RESET) {
      test_t t = { exp_no, (exp_no == 1) ? GOAL_CURRENT : (exp_no == 2) ? POS_GOAL : GOAL_VELOCITY,
                   T_TOTAL_MS - T_IDLE_MS, 0, 0, 1 };
      run_experiment(&t);
      exp_no = exp_no % 3 + 1;
      printf("# next: experiment %d (press USER button) or type a command\r\n> ", exp_no);
    }
    btn_prev = btn;

    if (console_getline()) {          /* a full line was typed */
      handle_command(line);
      printf("> ");
    }
    HAL_Delay(5);
    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration: HSI 16 MHz, no PLL (same as Lab1)
  *        USART1/USART2 clock = 16 MHz -> 1 Mbps divides exactly
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1: DYNAMIXEL bus, single-wire half-duplex on PA9 (D8)
  *        Pin is configured here directly (open-drain, external 4.7k pull-up),
  *        so it works even if the .ioc / msp file has no USART1 settings.
  */
static void MX_USART1_UART_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  huart1.Instance = USART1;
  huart1.Init.BaudRate = DXL_BAUD;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

  /* (re)configure PA9 after init, in case the msp file set it differently */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief USART2: printf output via ST-LINK virtual COM port (PA2/PA3)
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO: PC13 = USER button (input, board has external pull-up)
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PA5 = LD2 green LED (Arduino D13): lights up when the step command is sent */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
