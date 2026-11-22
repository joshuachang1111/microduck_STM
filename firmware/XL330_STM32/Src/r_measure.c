/*
 * r_measure.c — XL330 equivalent resistance R, manual measurement mode
 *
 * Built only when MOTOR_R_TEST = 1 (see r_measure.h). One command per line on the
 * ST-LINK virtual COM port (USART2, 115200); results are printed for manual logging.
 * No automatic or continuous measurement. Uses the existing driver in dxl.c.
 *
 * Commands: help | ping | id <n> | info | pwmmode | read | stall <d> <ms> | off
 */
#include "r_measure.h"

#if MOTOR_R_TEST

#include "main.h"
#include "dxl.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

extern UART_HandleTypeDef huart2;          /* console, defined in main.c */

#define ADDR_PWM_LIMIT      36             /* 2 bytes */
#define ADDR_CURRENT_LIMIT  38             /* 2 bytes, 1 mA/unit */

#define PWM_FULL            885            /* 885 = 100 % */
#define TEMP_LIMIT_C        60             /* refuse to start / stop a running stall */
#define WATCHDOG_20MS       10             /* Bus Watchdog during stall: 200 ms */
#define STALL_SKIP_MS       200            /* first 200 ms not averaged */
#define STALL_TS_MS         2              /* sampling period during stall */
#define STALL_D_HIGH        0.6f
#define STALL_MS_MIN        400
#define STALL_MS_MAX_HIGH   1000           /* |d| >  0.6 */
#define STALL_MS_MAX_LOW    5000           /* |d| <= 0.6 */
#define MOVED_MAX_PULSE     5
#define CUR_NEAR_LIMIT      0.98f
#define RETRY               3

static uint8_t g_id = 1;
static char line[64];
static int line_len = 0;

/* ---------- formatting (newlib-nano printf has no %f) ---------- */
static const char *ff(float x, int dec)
{
  static char buf[8][24];
  static int k = 0;
  char *b = buf[k++ & 7];
  if (!isfinite(x)) { strcpy(b, "—"); return b; }
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

static const char *err_text(int r)
{
  switch (r) {
    case DXL_ERR_TX:      return "傳送失敗";
    case DXL_ERR_TIMEOUT: return "逾時，馬達沒有回應";
    case DXL_ERR_PACKET:  return "回應封包格式錯誤";
    case DXL_ERR_CRC:     return "回應封包 CRC 錯誤";
    case DXL_ERR_SIZE:    return "封包長度超出緩衝區";
    case 1:               return "馬達回報：Result Fail";
    case 2:               return "馬達回報：Instruction Error";
    case 3:               return "馬達回報：CRC Error";
    case 4:               return "馬達回報：Data Range Error（數值超出範圍）";
    case 5:               return "馬達回報：Data Length Error";
    case 6:               return "馬達回報：Data Limit Error";
    case 7:               return "馬達回報：Access Error（例如扭力開著時寫 EEPROM）";
    default:              return "未知錯誤";
  }
}

/* ---------- DYNAMIXEL helpers (current ID g_id) ---------- */
typedef struct {
  int16_t  pwm;    /* 124 */
  int16_t  cur;    /* 126, mA */
  int32_t  vel;    /* 128, 0.229 rpm */
  int32_t  pos;    /* 132, pulse */
  uint16_t volt;   /* 144, 0.1 V */
  uint8_t  temp;   /* 146, C */
} meas_t;

static int32_t le32(const uint8_t *b)
{
  return (int32_t)((uint32_t)b[0] | ((uint32_t)b[1] << 8) |
                   ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24));
}

/* one READ of addr 124..146 (23 bytes) */
static int read_meas(meas_t *m)
{
  uint8_t d[23];
  int r = dxl_read(g_id, DXL_ADDR_PRESENT_PWM, d, sizeof(d));
  if (r != DXL_OK) return r;
  m->pwm  = (int16_t)(d[0] | (d[1] << 8));       /* 124 */
  m->cur  = (int16_t)(d[2] | (d[3] << 8));       /* 126 */
  m->vel  = le32(&d[4]);                         /* 128 */
  m->pos  = le32(&d[8]);                         /* 132 */
  m->volt = (uint16_t)(d[20] | (d[21] << 8));    /* 144 */
  m->temp = d[22];                               /* 146 */
  return DXL_OK;
}

static int read_meas_retry(meas_t *m)
{
  int r = DXL_ERR_TIMEOUT;
  for (int k = 0; k < RETRY && r != DXL_OK; k++) {
    r = read_meas(m);
    if (r != DXL_OK) HAL_Delay(5);
  }
  return r;
}

static int rd(uint16_t addr, uint16_t len, uint16_t *v)
{
  uint8_t d[2] = {0, 0};
  int r = DXL_ERR_TIMEOUT;
  for (int k = 0; k < RETRY && r != DXL_OK; k++) r = dxl_read(g_id, addr, d, len);
  *v = (len == 1) ? d[0] : (uint16_t)(d[0] | (d[1] << 8));
  return r;
}

static int wr1(uint16_t addr, uint8_t v)
{
  int r = DXL_ERR_TIMEOUT;
  for (int k = 0; k < RETRY && r != DXL_OK; k++) r = dxl_write1(g_id, addr, v);
  return r;
}

static int wr2(uint16_t addr, int16_t v)
{
  int r = DXL_ERR_TIMEOUT;
  for (int k = 0; k < RETRY && r != DXL_OK; k++) r = dxl_write2(g_id, addr, v);
  return r;
}

/* Goal PWM = 0, Bus Watchdog off, torque off. Returns the torque-off result. */
static int motor_off(void)
{
  int r1 = wr2(DXL_ADDR_GOAL_PWM, 0);
  int r2 = wr1(DXL_ADDR_BUS_WATCHDOG, 0);
  int r3 = wr1(DXL_ADDR_TORQUE_ENABLE, 0);
  if (r1 != DXL_OK || r2 != DXL_OK || r3 != DXL_OK)
    printf("# 警告：關閉程序結果 Goal PWM=%d、Watchdog=%d、關扭力=%d（0 = 成功）\r\n", r1, r2, r3);
  return r3;
}

/* communication error: print it and turn the torque off */
static void comm_fail(const char *what, int r)
{
  printf("# 錯誤：%s失敗，%s（代碼 %d）。執行關扭力。\r\n", what, err_text(r), r);
  int ro = motor_off();
  printf("# 關扭力%s\r\n", ro == DXL_OK ? "完成" : "失敗，請檢查接線與電源後輸入 off");
}

/* Hardware Error alert in the last reply: print addr 70 and turn the torque off.
 * Returns 1 if there was an alert. */
static int check_alert(void)
{
  if (!dxl_last_alert) return 0;
  uint16_t hw = 0xFF;
  int r = rd(DXL_ADDR_HW_ERROR, 1, &hw);
  printf("# 錯誤：馬達回報 Hardware Error（位址 70 = 0x%02X，讀取結果 %d）", hw & 0xFF, r);
  if (r == DXL_OK) {
    if (hw & 0x01) printf("：輸入電壓異常");
    if (hw & 0x04) printf("：過熱");
    if (hw & 0x08) printf("：馬達編碼器異常");
    if (hw & 0x10) printf("：電氣衝擊");
    if (hw & 0x20) printf("：過載");
  }
  printf("\r\n# 執行關扭力。要清除 Hardware Error 需重新上電或用 Wizard Reboot 馬達。\r\n");
  motor_off();
  return 1;
}

/* ---------- commands ---------- */
static void cmd_help(void)
{
  printf("# 指令（115200 baud，一行一個指令，以換行結尾，不分大小寫）：\r\n");
  printf("#   help              列出所有指令\r\n");
  printf("#   ping              確認馬達有回應（目前 ID %u）\r\n", g_id);
  printf("#   id <n>            切換要操作的馬達 ID（0～252）\r\n");
  printf("#   info              印 Operating Mode、PWM Limit、Current Limit、溫度\r\n");
  printf("#   pwmmode           關扭力 → 設為 PWM 模式（16）→ 讀回確認\r\n");
  printf("#   read              讀一次 d、電流、電壓、溫度、位置\r\n");
  printf("#   stall <d> <ms>    堵轉一次：-1 ≤ d ≤ 1；|d| > 0.6 時 ms 為 %d～%d，否則 %d～%d\r\n",
         STALL_MS_MIN, STALL_MS_MAX_HIGH, STALL_MS_MIN, STALL_MS_MAX_LOW);
  printf("#                     （馬達須先固定在夾具上；溫度 ≥ %d °C 不執行）\r\n", TEMP_LIMIT_C);
  printf("#   off               Goal PWM 歸零並關扭力\r\n");
}

static void cmd_ping(void)
{
  uint16_t model = 0;
  int r = DXL_ERR_TIMEOUT;
  for (int k = 0; k < RETRY && r != DXL_OK; k++) r = dxl_ping(g_id, &model);
  if (r != DXL_OK) { comm_fail("ping", r); return; }
  printf("# ping ID %u：有回應，型號 %u%s\r\n", g_id, (unsigned)model,
         model == 1200 ? "（XL330-M288）" : "");
  check_alert();
}

static void cmd_id(const char *arg)
{
  char *end;
  long n = arg ? strtol(arg, &end, 10) : -1;
  if (!arg || end == arg || *end != 0 || n < 0 || n > 252) {
    printf("# 用法：id <0～252>\r\n");
    return;
  }
  g_id = (uint8_t)n;
  printf("# 目前操作的馬達 ID = %u（可用 ping 確認）\r\n", g_id);
}

static void cmd_info(void)
{
  uint16_t mode, pwm_lim, cur_lim, temp;
  int r = rd(DXL_ADDR_OPERATING_MODE, 1, &mode);
  if (r == DXL_OK && check_alert()) return;
  if (r == DXL_OK) r = rd(ADDR_PWM_LIMIT, 2, &pwm_lim);
  if (r == DXL_OK) r = rd(ADDR_CURRENT_LIMIT, 2, &cur_lim);
  if (r == DXL_OK) r = rd(DXL_ADDR_PRESENT_TEMP, 1, &temp);
  if (r != DXL_OK) { comm_fail("讀取設定", r); return; }
  if (check_alert()) return;
  printf("# info ID %u：Operating Mode = %u%s、PWM Limit = %u（%s %%）、"
         "Current Limit = %u mA、溫度 = %u °C\r\n",
         g_id, mode, mode == DXL_MODE_PWM ? "（PWM 模式）" : "", pwm_lim,
         ff(100.0f * pwm_lim / PWM_FULL, 1), cur_lim, temp);
}

static void cmd_pwmmode(void)
{
  int r = wr2(DXL_ADDR_GOAL_PWM, 0);
  if (r == DXL_OK) r = wr1(DXL_ADDR_TORQUE_ENABLE, 0);
  if (r != DXL_OK) { comm_fail("關扭力", r); return; }
  if (check_alert()) return;
  HAL_Delay(10);
  r = wr1(DXL_ADDR_OPERATING_MODE, DXL_MODE_PWM);
  if (r != DXL_OK) { comm_fail("寫入 Operating Mode", r); return; }
  HAL_Delay(10);
  uint16_t m = 0xFF;
  r = rd(DXL_ADDR_OPERATING_MODE, 1, &m);
  if (r != DXL_OK) { comm_fail("讀回 Operating Mode", r); return; }
  if (check_alert()) return;
  if (m == DXL_MODE_PWM)
    printf("# pwmmode：已關扭力，Operating Mode 寫入 16、讀回 %u → 成功（PWM 模式）\r\n", m);
  else
    printf("# pwmmode：寫入 16 但讀回 %u → 失敗，請再試一次\r\n", m);
}

static void cmd_read(void)
{
  meas_t m;
  int r = read_meas_retry(&m);
  if (r != DXL_OK) { comm_fail("讀取狀態", r); return; }
  if (check_alert()) return;
  printf("# read ID %u：d = %s、電流 = %s A、電壓 = %s V、溫度 = %u °C、位置 = %ld pulse"
         "（速度 = %ld × 0.229 rpm）\r\n",
         g_id, ff((float)m.pwm / PWM_FULL, 4), ff(m.cur / 1000.0f, 3), ff(m.volt * 0.1f, 1),
         m.temp, (long)m.pos, (long)m.vel);
}

static void cmd_off(void)
{
  int r = motor_off();
  if (r != DXL_OK) { printf("# 錯誤：關扭力失敗，%s（代碼 %d）\r\n", err_text(r), r); return; }
  printf("# off：Goal PWM = 0，扭力已關閉\r\n");
}

static void cmd_stall(const char *a1, const char *a2)
{
  char *e1, *e2;
  float d = a1 ? strtof(a1, &e1) : 0;
  long ms = a2 ? strtol(a2, &e2, 10) : 0;
  if (!a1 || !a2 || e1 == a1 || *e1 != 0 || e2 == a2 || *e2 != 0) {
    printf("# 用法：stall <d，-1～1> <ms>，例如 stall 0.3 2000\r\n");
    return;
  }
  if (!(d >= -1.0f && d <= 1.0f)) { printf("# 拒絕執行：d 必須在 -1～1 之間\r\n"); return; }
  long ms_max = (fabsf(d) > STALL_D_HIGH) ? STALL_MS_MAX_HIGH : STALL_MS_MAX_LOW;
  if (ms < STALL_MS_MIN || ms > ms_max) {
    printf("# 拒絕執行：|d| %s 0.6 時，ms 必須在 %d～%ld 之間\r\n",
           fabsf(d) > STALL_D_HIGH ? ">" : "≤", STALL_MS_MIN, ms_max);
    return;
  }

  /* pre-checks */
  uint16_t mode, pwm_lim, cur_lim;
  int r = rd(DXL_ADDR_OPERATING_MODE, 1, &mode);
  if (r == DXL_OK) r = rd(ADDR_PWM_LIMIT, 2, &pwm_lim);
  if (r == DXL_OK) r = rd(ADDR_CURRENT_LIMIT, 2, &cur_lim);
  if (r != DXL_OK) { comm_fail("讀取設定", r); return; }
  if (check_alert()) return;
  if (mode != DXL_MODE_PWM) {
    printf("# 拒絕執行：Operating Mode 是 %u，不是 16（PWM 模式），請先輸入 pwmmode\r\n", mode);
    return;
  }
  meas_t m;
  r = read_meas_retry(&m);
  if (r != DXL_OK) { comm_fail("讀取狀態", r); return; }
  if (check_alert()) return;
  if (m.temp >= TEMP_LIMIT_C) {
    printf("# 拒絕執行：馬達溫度 %u °C ≥ %d °C，請先冷卻\r\n", m.temp, TEMP_LIMIT_C);
    return;
  }

  int32_t goal = (int32_t)lroundf(d * PWM_FULL);
  if (goal > (int32_t)pwm_lim || goal < -(int32_t)pwm_lim) {
    int32_t g2 = goal > 0 ? (int32_t)pwm_lim : -(int32_t)pwm_lim;
    printf("# 警告：Goal PWM %ld 超過 PWM Limit %u，改用 %ld\r\n", (long)goal, pwm_lim, (long)g2);
    goal = g2;
  }
  printf("# stall ID %u：d = %s（Goal PWM = %ld），%ld ms，前 %d ms 不採計，開始溫度 %u °C\r\n",
         g_id, ff(d, 3), (long)goal, ms, STALL_SKIP_MS, m.temp);

  /* Goal PWM = 0 -> torque on -> watchdog -> Goal PWM = d x 885 */
  r = wr2(DXL_ADDR_GOAL_PWM, 0);
  if (r == DXL_OK) r = wr1(DXL_ADDR_TORQUE_ENABLE, 1);
  if (r == DXL_OK) r = wr1(DXL_ADDR_BUS_WATCHDOG, WATCHDOG_20MS);
  if (r == DXL_OK) r = wr2(DXL_ADDR_GOAL_PWM, (int16_t)goal);
  if (r != DXL_OK) { comm_fail("開始堵轉", r); return; }
  if (check_alert()) return;
  uint32_t t0 = HAL_GetTick();

  uint32_t n = 0;
  float sum_d = 0, sum_i = 0, sum_v = 0;
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
    r = read_meas(&s);
    if (r != DXL_OK) r = read_meas(&s);                 /* one retry */
    if (r != DXL_OK) { comm_fail("堵轉中讀取狀態", r); return; }
    if (check_alert()) return;
    if (s.pos < pmin) pmin = s.pos;
    if (s.pos > pmax) pmax = s.pos;
    if (s.temp > tmax) tmax = s.temp;
    if (el >= STALL_SKIP_MS) {
      sum_d += fabsf((float)s.pwm) / PWM_FULL;
      sum_i += fabsf((float)s.cur) / 1000.0f;
      sum_v += s.volt * 0.1f;
      n++;
    }
    if (s.temp >= TEMP_LIMIT_C) { hot = 1; break; }
  }
  uint32_t dur = HAL_GetTick() - t0;

  /* Goal PWM = 0 -> torque off */
  r = motor_off();
  if (r != DXL_OK) printf("# 警告：關扭力失敗（%s），請立刻輸入 off！\r\n", err_text(r));

  if (n == 0) { printf("# 沒有採計到任何資料（實際執行 %lu ms）\r\n", (unsigned long)dur); return; }
  float dm = sum_d / n, I = sum_i / n, V = sum_v / n;
  int32_t moved = pmax - pmin;
  float vi = (I > 0) ? V / I : NAN;
  float d2vi = (I > 0) ? dm * dm * V / I : NAN;

  printf("# 結果 ID %u：n = %lu、d = %s、V = %s V、I = %s A、T = %u °C、moved = %ld pulse\r\n",
         g_id, (unsigned long)n, ff(dm, 4), ff(V, 3), ff(I, 4), tmax, (long)moved);
  printf("# 參考值：V/I = %s Ω、d²·V/I = %s Ω（實際執行 %lu ms）\r\n",
         ff(vi, 3), ff(d2vi, 3), (unsigned long)dur);
  if (moved > MOVED_MAX_PULSE)
    printf("# 警告：馬達有轉動，這筆不能用（moved = %ld > %d pulse）\r\n", (long)moved, MOVED_MAX_PULSE);
  if (I * 1000.0f > CUR_NEAR_LIMIT * cur_lim)
    printf("# 警告：電流貼近 Current Limit，這筆不能用（I = %s mA > 0.98 × %u mA）\r\n",
           ff(I * 1000.0f, 0), cur_lim);
  if (hot)
    printf("# 警告：溫度達 %u °C，提前停止，時間不足，這筆不能用\r\n", tmax);
}

/* ---------- console ---------- */
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
    if (a3) printf("# 用法：stall <d，-1～1> <ms>\r\n");
    else    cmd_stall(a1, a2);
  }
  else if (!strcmp(cmd, "off"))     cmd_off();
  else printf("# 不認識的指令「%s」，輸入 help 查看指令\r\n", cmd);
}

void r_measure_run(void)
{
  HAL_Delay(200);                          /* let the servo boot */
  printf("\r\n# XL330 等效電阻 R 手動量測模式（MOTOR_R_TEST）\r\n");
  printf("# DYNAMIXEL Protocol 2.0、1 Mbps、馬達 ID %u\r\n", g_id);
  int r = motor_off();
  if (r == DXL_OK) printf("# 開機：Goal PWM = 0，扭力已關閉\r\n");
  else printf("# 開機：關扭力失敗（%s），馬達可能沒接好或沒上電，請用 ping 確認\r\n", err_text(r));
  check_alert();
  cmd_help();
  printf("> ");
  for (;;) {
    if (console_getline()) {
      handle_command(line);
      printf("> ");
    }
  }
}

#else
typedef int r_measure_disabled_t;   /* avoid an empty translation unit */
#endif /* MOTOR_R_TEST */
