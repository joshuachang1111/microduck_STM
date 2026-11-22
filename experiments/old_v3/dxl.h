/*
 * dxl.h — Minimal DYNAMIXEL Protocol 2.0 driver for STM32 HAL
 * Single-wire half-duplex UART (e.g. USART1 TX = PA9)
 * Control table addresses: DYNAMIXEL X-series (XL330-M288)
 */
#ifndef DXL_H
#define DXL_H

#include "main.h"
#include <stdint.h>

/* ---- Control table (X-series, RAM area) ---- */
#define DXL_ADDR_OPERATING_MODE    11   /* 1 byte, EEPROM: write only when torque is off */
#define DXL_ADDR_TORQUE_ENABLE     64   /* 1 byte */
#define DXL_ADDR_HW_ERROR          70   /* 1 byte, Hardware Error Status */
#define DXL_ADDR_BUS_WATCHDOG      98   /* 1 byte, 20 ms/unit, 0 = off */
#define DXL_ADDR_GOAL_PWM          100  /* 2 bytes, 885 = 100 % */
#define DXL_ADDR_GOAL_CURRENT      102  /* 2 bytes */
#define DXL_ADDR_GOAL_VELOCITY     104  /* 4 bytes */
#define DXL_ADDR_GOAL_POSITION     116  /* 4 bytes */
#define DXL_ADDR_REALTIME_TICK     120  /* 2 bytes, servo-side ms counter (0~32767) */
#define DXL_ADDR_PRESENT_PWM       124  /* 2 bytes */
#define DXL_ADDR_PRESENT_CURRENT   126  /* 2 bytes */
#define DXL_ADDR_PRESENT_VELOCITY  128  /* 4 bytes */
#define DXL_ADDR_PRESENT_POSITION  132  /* 4 bytes */
#define DXL_ADDR_VELOCITY_TRAJ     136  /* 4 bytes, profile-generated velocity */
#define DXL_ADDR_POSITION_TRAJ     140  /* 4 bytes, profile-generated position */
#define DXL_ADDR_PRESENT_VOLTAGE   144  /* 2 bytes, 0.1 V/unit */
#define DXL_ADDR_PRESENT_TEMP      146  /* 1 byte, deg C */

/* ---- Operating modes ---- */
#define DXL_MODE_CURRENT   0
#define DXL_MODE_VELOCITY  1
#define DXL_MODE_POSITION  3
#define DXL_MODE_PWM       16

/* ---- Return codes (>0 = error byte reported by the servo) ---- */
#define DXL_OK           0
#define DXL_ERR_TX      -1
#define DXL_ERR_TIMEOUT -2
#define DXL_ERR_PACKET  -3
#define DXL_ERR_CRC     -4
#define DXL_ERR_SIZE    -5

/* Raw values; convert units in MATLAB/Python (see README) */
typedef struct {
  uint16_t tick;      /* servo realtime tick [ms], wraps at 32767 */
  int16_t  pwm;       /* present PWM, raw */
  int16_t  current;   /* present current, raw */
  int32_t  velocity;  /* present velocity, raw */
  int32_t  position;  /* present position, raw (4096 per rev) */
  int32_t  vel_traj;  /* velocity trajectory (profile output), raw */
  int32_t  pos_traj;  /* position trajectory (profile output), raw */
  uint16_t volt;      /* present input voltage, 0.1 V/unit */
  uint8_t  temp;      /* present temperature, deg C */
} dxl_state_t;

/* debug info about the last transaction */
extern uint16_t dxl_dbg_rx_got;
extern uint8_t  dxl_dbg_rx[16];

void dxl_init(UART_HandleTypeDef *huart);
int  dxl_ping(uint8_t id, uint16_t *model);
int  dxl_write(uint8_t id, uint16_t addr, const uint8_t *data, uint16_t len);
int  dxl_write1(uint8_t id, uint16_t addr, uint8_t v);
int  dxl_write2(uint8_t id, uint16_t addr, int16_t v);
int  dxl_write4(uint8_t id, uint16_t addr, int32_t v);
int  dxl_read(uint8_t id, uint16_t addr, uint8_t *data, uint16_t len);
/* One READ of addr 120..146: tick, PWM, current, velocity, position, vel/pos trajectory, voltage, temperature */
int  dxl_read_state(uint8_t id, dxl_state_t *s);

#endif /* DXL_H */
