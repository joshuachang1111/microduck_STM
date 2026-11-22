/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : XL330 equivalent resistance R measurement (stall test)
  *                   NUCLEO-F446RE, DYNAMIXEL Protocol 2.0, 1 Mbps
  ******************************************************************************
  * Standalone experiment firmware (2026-11-22). Does not change firmware/XL330_STM32.
  *
  * Wiring (same as firmware v4):
  *   XL330 DATA -- PA9 (Arduino D8, USART1 single-wire half-duplex) -- pull-up -- 3V3
  *   XL330 V    -- bench supply +5 V,  XL330 G -- supply GND -- Nucleo GND
  * Console: USART2 = ST-LINK virtual COM port, 115200 8N1, one command per line.
  *
  * Commands: help | ping | id <n> | info | pwmmode | read | stall <d> <ms> | off
  *
  * stall <d> <ms>:
  *   -1 <= d <= 1; |d| > 0.6 -> 400..1000 ms, else 400..5000 ms; refused at >= 60 C
  *   Goal PWM = 0 -> torque on -> Goal PWM = d*885 -> sample for ms (first 200 ms
  *   not averaged) -> Goal PWM = 0 -> torque off.
  *   Extra safety: Bus Watchdog 200 ms during the run, early stop at >= 60 C,
  *   Goal PWM clamped to PWM Limit(36) with a warning, refused unless Operating Mode = 16.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "dxl.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

#define FW_NAME            "XL330 R measurement (2026-11-22)"
#define DXL_BAUD           1000000
#define DXL_ADDR_PWM_LIMIT     36   /* 2 bytes, EEPROM */
#define DXL_ADDR_CURRENT_LIMIT 38   /* 2 bytes, EEPROM, mA (XL330: 1 mA/unit) */

#define PWM_FULL           885      /* Goal/Present PWM raw value for 100 % */
#define TEMP_LIMIT_C       60       /* refuse to start / stop a running stall at this temperature */
#define WATCHDOG_20MS      10       /* Bus Watchdog during stall: 10 x 20 ms = 200 ms */
#define STALL_SKIP_MS      200      /* first 200 ms after Goal PWM is set are not averaged */
#define STALL_TS_MS        2        /* sampling period during stall */
#define STALL_D_HIGH       0.6f     /* |d| above this -> shorter time window */
#define STALL_MS_MIN       400
#define STALL_MS_MAX_HIGH  1000
#define STALL_MS_MAX_LOW   5000
#define MOVED_MAX_PULSE    5        /* position change above this -> data not usable */
#define CUR_NEAR_LIMIT     0.98f    /* I above this fraction of Current Limit -> not usable */
#define WR_RETRY           3
/* USER CODE END PD */

UART_HandleTypeDef huart1;    /* DYNAMIXEL bus (half-duplex, PA9) */
UART_HandleTypeDef huart2;    /* console -> ST-LINK virtual COM */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN 0 */
static uint8_t g_id = 1;
static char line[64];
static int line_len = 0;

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/* Fixed-point float formatting (newlib-nano printf has no %f by default).
 * Returns one of 8 rotating static buffers, so several can be used in one printf. */
static const char *ff(float x, int dec)
{
  static char buf[8][24];
  static int k = 0;
  char *b = buf[k++ & 7];
  if (!isfinite(x)) { strcpy(b, "nan"); return b; }
  long scale = 1;
  for (int i = 0; i < dec; i++) scale *= 10;
  int neg = x < 0;
  long v = (long)(fabsf(x) * (float)scale + 0.5f);
  if (dec > 0)
    snprintf(b, 24, "%s%ld.%0*ld", (neg && v) ? "-" : "", v / scale, dec, v % scale);
  else
    snprintf(b, 24, "%s%ld", (neg && v) ? "-" : "", v);
  return b;
}

/* ---- DYNAMIXEL helpers (all use the current ID g_id) ---- */
typedef struct {
  int16_t  pwm;       /* 124, raw, 885 = 100 % */
  int16_t  cur;       /* 126, mA */
  int32_t  pos;       /* 132, pulse */
  uint16_t volt;      /* 144, 0.1 V */
  uint8_t  temp;      /* 146, deg C */
} meas_t;

/* one READ of addr 124..146 (23 bytes) */
static int read_meas(meas_t *m)
{
  uint8_t d[23];
  int r = dxl_read(g_id, DXL_ADDR_PRESENT_PWM, d, sizeof(d));
  if (r != DXL_OK) return r;
  m->pwm  = (int16_t)(d[0] | (d[1] << 8));                       /* 124 */
  m->cur  = (int16_t)(d[2] | (d[3] << 8));                       /* 126 */
  m->pos  = (int32_t)((uint32_t)d[8] | ((uint32_t)d[9] << 8) |
                      ((uint32_t)d[10] << 16) | ((uint32_t)d[11] << 24));   /* 132 */
  m->volt = (uint16_t)(d[20] | (d[21] << 8));                    /* 144 */
  m->temp = d[22];                                               /* 146 */
  return DXL_OK;
}

static int read_meas_retry(meas_t *m)
{
  int r = DXL_ERR_TIMEOUT;
  for (int k = 0; k < WR_RETRY && r != DXL_OK; k++) {
    r = read_meas(m);
    if (r != DXL_OK) HAL_Delay(5);
  }
  return r;
}

static int rd1(uint16_t addr, uint8_t *v)
{
  int r = DXL_ERR_TIMEOUT;
  for (int k = 0; k < WR_RETRY && r != DXL_OK; k++) r = dxl_read(g_id, addr, v, 1);
  return r;
}

static int rd2(uint16_t addr, uint16_t *v)
{
  uint8_t d[2] = {0, 0};
  int r = DXL_ERR_TIMEOUT;
  for (int k = 0; k < WR_RETRY && r != DXL_OK; k++) r = dxl_read(g_id, addr, d, 2);
  *v = (uint16_t)(d[0] | (d[1] << 8));
  return r;
}

static int wr1(uint16_t addr, uint8_t v)
{
  int r = DXL_ERR_TIMEOUT;
  for (int k = 0; k < WR_RETRY && r != DXL_OK; k++) r = dxl_write1(g_id, addr, v);
  return r;
}

static int wr2(uint16_t addr, int16_t v)
{
  int r = DXL_ERR_TIMEOUT;
  for (int k = 0; k < WR_RETRY && r != DXL_OK; k++) r = dxl_write2(g_id, addr, v);
  return r;
}

/* Goal PWM = 0, watchdog off, torque off. Returns the torque-off result. */
static int motor_off(void)
{
  int r1 = wr2(DXL_ADDR_GOAL_PWM, 0);
  int r2 = wr1(DXL_ADDR_BUS_WATCHDOG, 0);
  int r3 = wr1(DXL_ADDR_TORQUE_ENABLE, 0);
  if (r1 != DXL_OK || r2 != DXL_OK || r3 != DXL_OK)
    printf("# WARNING: off sequence results: goal_pwm=%d watchdog=%d torque=%d\r\n", r1, r2, r3);
  return r3;
}

/* ---- commands ---- */
static void cmd_help(void)
{
  printf("# commands (115200 baud, one per line):\r\n");
  printf("#   help              this list\r\n");
  printf("#   ping              check that motor ID %u answers\r\n", g_id);
  printf("#   id <n>            switch motor ID (0..252, now %u)\r\n", g_id);
  printf("#   info              Operating Mode(11), PWM Limit(36), Current Limit(38), Temperature(146)\r\n");
  printf("#   pwmmode           torque off, Operating Mode = 16 (PWM), read back\r\n");
  printf("#   read              Present PWM, Current, Input Voltage, Temperature, Position\r\n");
  printf("#   stall <d> <ms>    stall test, -1<=d<=1; |d|>0.6: %d..%d ms, else %d..%d ms\r\n",
         STALL_MS_MIN, STALL_MS_MAX_HIGH, STALL_MS_MIN, STALL_MS_MAX_LOW);
  printf("#                     (motor must be clamped; refused at >= %d C)\r\n", TEMP_LIMIT_C);
  printf("#   off               Goal PWM = 0, torque off\r\n");
}

static void cmd_ping(void)
{
  uint16_t model = 0;
  int r = DXL_ERR_TIMEOUT;
  for (int k = 0; k < WR_RETRY && r != DXL_OK; k++) r = dxl_ping(g_id, &model);
  if (r == DXL_OK) printf("# ping id=%u: OK, model=%u\r\n", g_id, (unsigned)model);
  else             printf("# ping id=%u: FAILED (result %d, rx bytes %u)\r\n", g_id, r,
                          (unsigned)dxl_dbg_rx_got);
}

static void cmd_id(const char *arg)
{
  char *end;
  long n = arg ? strtol(arg, &end, 10) : -1;
  if (!arg || end == arg || *end != 0 || n < 0 || n > 252) {
    printf("# usage: id <0..252>\r\n");
    return;
  }
  g_id = (uint8_t)n;
  printf("# motor ID = %u\r\n", g_id);
}

static void cmd_info(void)
{
  uint8_t mode = 0xFF, temp = 0, hw = 0xFF;
  uint16_t pwm_lim = 0, cur_lim = 0;
  int r1 = rd1(DXL_ADDR_OPERATING_MODE, &mode);
  int r2 = rd2(DXL_ADDR_PWM_LIMIT, &pwm_lim);
  int r3 = rd2(DXL_ADDR_CURRENT_LIMIT, &cur_lim);
  int r4 = rd1(DXL_ADDR_PRESENT_TEMP, &temp);
  int r5 = rd1(DXL_ADDR_HW_ERROR, &hw);
  printf("# info id=%u: OperatingMode=%u%s PWMLimit=%u (%s %%) CurrentLimit=%u mA "
         "Temperature=%u C HardwareError=0x%02X (read %d %d %d %d %d)\r\n",
         g_id, mode, mode == DXL_MODE_PWM ? "(PWM)" : "", pwm_lim,
         ff(100.0f * pwm_lim / PWM_FULL, 1), cur_lim, temp, hw, r1, r2, r3, r4, r5);
}

static void cmd_pwmmode(void)
{
  int r = wr2(DXL_ADDR_GOAL_PWM, 0);
  if (r != DXL_OK) printf("# Goal PWM = 0 failed: %d (continuing)\r\n", r);
  r = wr1(DXL_ADDR_TORQUE_ENABLE, 0);
  if (r != DXL_OK) { printf("# torque off failed: %d, abort\r\n", r); return; }
  HAL_Delay(10);
  r = wr1(DXL_ADDR_OPERATING_MODE, DXL_MODE_PWM);
  if (r != DXL_OK) { printf("# write Operating Mode failed: %d\r\n", r); return; }
  HAL_Delay(10);
  uint8_t m = 0xFF;
  int rr = rd1(DXL_ADDR_OPERATING_MODE, &m);
  printf("# pwmmode: torque off, wrote Operating Mode %u, read back %u (result %d) -> %s\r\n",
         DXL_MODE_PWM, m, rr, (rr == DXL_OK && m == DXL_MODE_PWM) ? "OK" : "MISMATCH");
}

static void cmd_read(void)
{
  meas_t m;
  int r = read_meas_retry(&m);
  if (r != DXL_OK) { printf("# read failed: %d\r\n", r); return; }
  printf("# read id=%u: PWM=%d (%s %%) Current=%d mA Voltage=%s V Temperature=%u C Position=%ld\r\n",
         g_id, m.pwm, ff(100.0f * m.pwm / PWM_FULL, 1), m.cur, ff(m.volt * 0.1f, 1), m.temp,
         (long)m.pos);
}

static void cmd_off(void)
{
  int r = motor_off();
  printf("# off: Goal PWM = 0, torque off (result %d)\r\n", r);
}

static void cmd_stall(const char *a1, const char *a2)
{
  char *e1, *e2;
  float d = a1 ? strtof(a1, &e1) : 0;
  long ms = a2 ? strtol(a2, &e2, 10) : 0;
  if (!a1 || !a2 || e1 == a1 || *e1 != 0 || e2 == a2 || *e2 != 0) {
    printf("# usage: stall <d -1..1> <ms>\r\n");
    return;
  }
  if (!(d >= -1.0f && d <= 1.0f)) { printf("# REFUSED: d must be -1..1\r\n"); return; }
  long ms_max = (fabsf(d) > STALL_D_HIGH) ? STALL_MS_MAX_HIGH : STALL_MS_MAX_LOW;
  if (ms < STALL_MS_MIN || ms > ms_max) {
    printf("# REFUSED: for |d| %s %s, ms must be %d..%ld\r\n",
           fabsf(d) > STALL_D_HIGH ? ">" : "<=", ff(STALL_D_HIGH, 1), STALL_MS_MIN, ms_max);
    return;
  }

  /* pre-checks: mode, limits, temperature */
  uint8_t mode = 0xFF;
  uint16_t pwm_lim = 0, cur_lim = 0;
  int r = rd1(DXL_ADDR_OPERATING_MODE, &mode);
  if (r == DXL_OK) r = rd2(DXL_ADDR_PWM_LIMIT, &pwm_lim);
  if (r == DXL_OK) r = rd2(DXL_ADDR_CURRENT_LIMIT, &cur_lim);
  if (r != DXL_OK) { printf("# REFUSED: cannot read mode/limits (%d)\r\n", r); return; }
  if (mode != DXL_MODE_PWM) {
    printf("# REFUSED: Operating Mode is %u, not 16 (PWM). Run pwmmode first.\r\n", mode);
    return;
  }
  meas_t m;
  r = read_meas_retry(&m);
  if (r != DXL_OK) { printf("# REFUSED: read failed (%d)\r\n", r); return; }
  if (m.temp >= TEMP_LIMIT_C) {
    printf("# REFUSED: temperature %u C >= %d C, let it cool down\r\n", m.temp, TEMP_LIMIT_C);
    return;
  }

  int32_t goal = (int32_t)lroundf(d * PWM_FULL);
  if (goal > (int32_t)pwm_lim || goal < -(int32_t)pwm_lim) {
    int32_t g2 = goal > 0 ? (int32_t)pwm_lim : -(int32_t)pwm_lim;
    printf("# WARNING: Goal PWM %ld exceeds PWM Limit %u, clamped to %ld\r\n",
           (long)goal, pwm_lim, (long)g2);
    goal = g2;
  }
  printf("# stall id=%u: d=%s goal_pwm=%ld for %ld ms (first %d ms not averaged), "
         "start temp %u C\r\n", g_id, ff(d, 3), (long)goal, ms, STALL_SKIP_MS, m.temp);

  /* Goal PWM = 0 -> torque on -> watchdog -> Goal PWM = d*885 */
  r = wr2(DXL_ADDR_GOAL_PWM, 0);
  if (r == DXL_OK) r = wr1(DXL_ADDR_TORQUE_ENABLE, 1);
  if (r == DXL_OK) {
    int rw = wr1(DXL_ADDR_BUS_WATCHDOG, WATCHDOG_20MS);
    if (rw != DXL_OK) printf("# WARNING: bus watchdog on failed: %d\r\n", rw);
  }
  if (r == DXL_OK) r = wr2(DXL_ADDR_GOAL_PWM, (int16_t)goal);
  if (r != DXL_OK) {
    printf("# start sequence failed: %d, turning off\r\n", r);
    motor_off();
    return;
  }
  uint32_t t0 = HAL_GetTick();

  /* sampling */
  uint32_t n = 0, nall = 0, nerr = 0;
  float sum_pwm = 0, sum_cur = 0, sum_volt = 0;
  int32_t pmin = INT32_MAX, pmax = INT32_MIN;
  uint8_t tmax = m.temp;
  int hot = 0;
  uint32_t next = t0;
  for (;;) {
    uint32_t now = HAL_GetTick();
    uint32_t el = now - t0;
    if (el >= (uint32_t)ms) break;
    if ((int32_t)(now - next) < 0) continue;
    next += STALL_TS_MS;
    meas_t s;
    if (read_meas(&s) != DXL_OK) { nerr++; continue; }
    nall++;
    if (s.pos < pmin) pmin = s.pos;
    if (s.pos > pmax) pmax = s.pos;
    if (s.temp > tmax) tmax = s.temp;
    if (el >= STALL_SKIP_MS) {
      sum_pwm  += fabsf((float)s.pwm);
      sum_cur  += fabsf((float)s.cur);
      sum_volt += (float)s.volt;
      n++;
    }
    if (s.temp >= TEMP_LIMIT_C) { hot = 1; break; }
  }
  uint32_t dur = HAL_GetTick() - t0;

  /* Goal PWM = 0 -> torque off */
  int roff = motor_off();
  if (roff != DXL_OK) printf("# WARNING: torque off failed (%d): send off again!\r\n", roff);

  if (n == 0) {
    printf("# stall: no valid samples after %d ms (samples %lu, read errors %lu)\r\n",
           STALL_SKIP_MS, (unsigned long)nall, (unsigned long)nerr);
    return;
  }
  float dm = sum_pwm / n / PWM_FULL;        /* |d| */
  float I  = sum_cur / n / 1000.0f;         /* A  */
  float V  = sum_volt / n * 0.1f;           /* V  */
  int32_t moved = (nall > 0) ? (pmax - pmin) : 0;
  float vi   = (I > 0) ? V / I : NAN;
  float d2vi = (I > 0) ? dm * dm * V / I : NAN;

  printf("# stall result id=%u: n=%lu d=%s V=%s V I=%s A T=%u C moved=%ld pulse  "
         "V/I=%s ohm  d^2*V/I=%s ohm  (run %lu ms, read errors %lu)\r\n",
         g_id, (unsigned long)n, ff(dm, 4), ff(V, 3), ff(I, 4), tmax, (long)moved,
         ff(vi, 3), ff(d2vi, 3), (unsigned long)dur, (unsigned long)nerr);
  printf("stall_csv,id,d_cmd,ms,n,d,V,I,T,moved,VI,d2VI,err\r\n");
  printf("stall_csv,%u,%s,%ld,%lu,%s,%s,%s,%u,%ld,%s,%s,%lu\r\n",
         g_id, ff(d, 3), ms, (unsigned long)n, ff(dm, 4), ff(V, 3), ff(I, 4), tmax,
         (long)moved, ff(vi, 3), ff(d2vi, 3), (unsigned long)nerr);

  if (moved > MOVED_MAX_PULSE)
    printf("# WARNING: 馬達有轉動，這筆不能用 (moved=%ld > %d pulse)\r\n", (long)moved, MOVED_MAX_PULSE);
  if (I * 1000.0f > CUR_NEAR_LIMIT * cur_lim)
    printf("# WARNING: 電流貼近 Current Limit，這筆不能用 (I=%s mA > 0.98 x %u mA)\r\n",
           ff(I * 1000.0f, 0), cur_lim);
  if (hot)
    printf("# WARNING: STOPPED EARLY, temperature %u C >= %d C; 這筆時間不足\r\n", tmax, TEMP_LIMIT_C);
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

static void handle_command(char *s)
{
  for (char *p = s; *p; p++) *p = (char)tolower((unsigned char)*p);
  char *cmd = strtok(s, " \t");
  char *a1 = strtok(NULL, " \t");
  char *a2 = strtok(NULL, " \t");
  char *a3 = strtok(NULL, " \t");
  if (!cmd) return;
  if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) cmd_help();
  else if (!strcmp(cmd, "ping"))    cmd_ping();
  else if (!strcmp(cmd, "id"))      cmd_id(a1);
  else if (!strcmp(cmd, "info"))    cmd_info();
  else if (!strcmp(cmd, "pwmmode")) cmd_pwmmode();
  else if (!strcmp(cmd, "read"))    cmd_read();
  else if (!strcmp(cmd, "stall")) {
    if (a3) printf("# usage: stall <d -1..1> <ms>\r\n");
    else    cmd_stall(a1, a2);
  }
  else if (!strcmp(cmd, "off"))     cmd_off();
  else printf("# unknown command \"%s\" (type help)\r\n", cmd);
}
/* USER CODE END 0 */

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
  printf("\r\n# %s\r\n", FW_NAME);
  printf("# DYNAMIXEL Protocol 2.0, %lu bps, motor ID %u\r\n", (unsigned long)DXL_BAUD, g_id);
  int r = motor_off();
  printf("# boot: Goal PWM = 0, torque off (result %d%s)\r\n", r,
         r == DXL_OK ? "" : ", motor not answering? try ping");
  cmd_help();
  printf("> ");
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    if (console_getline()) {
      handle_command(line);
      printf("> ");
    }
    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration: HSI 16 MHz, no PLL (same as firmware v4)
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
  * @brief USART1: DYNAMIXEL bus, single-wire half-duplex on PA9 (D8), open-drain
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

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief USART2: console via ST-LINK virtual COM port (PA2/PA3), 115200
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
  * @brief GPIO: PC13 = USER button input (unused), PA5 = LD2 off
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

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

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
