/*
 * dxl.c — Minimal DYNAMIXEL Protocol 2.0 driver for STM32 HAL
 *
 * Packet: FF FF FD 00 | ID | LEN_L LEN_H | INST | PARAMS... | CRC_L CRC_H
 * LEN = (instruction + params + CRC) byte count
 */
#include "dxl.h"
#include <string.h>

#define DXL_INST_PING   0x01
#define DXL_INST_READ   0x02
#define DXL_INST_WRITE  0x03
#define DXL_INST_STATUS 0x55
#define DXL_RX_TIMEOUT_MS 5

static UART_HandleTypeDef *dxl_huart;
uint16_t dxl_dbg_rx_got = 0;      /* bytes received in the last transaction */
uint8_t  dxl_dbg_rx[16];          /* first bytes of the last reply (debug) */
static uint8_t txbuf[64];
static uint8_t rxbuf[64];

/* CRC-16 (poly 0x8005, init 0), as specified by Protocol 2.0.
 * v4: table-driven (table built once in dxl_init) -> ~8x faster than bit-by-bit,
 * needed for 500 Hz logging at 16 MHz. */
static uint16_t crc_table[256];

static void dxl_crc_init(void)
{
  for (int i = 0; i < 256; i++) {
    uint16_t c = (uint16_t)(i << 8);
    for (int b = 0; b < 8; b++)
      c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x8005) : (uint16_t)(c << 1);
    crc_table[i] = c;
  }
}

static uint16_t dxl_crc(const uint8_t *d, uint16_t n)
{
  uint16_t crc = 0;
  for (uint16_t i = 0; i < n; i++)
    crc = (uint16_t)((crc << 8) ^ crc_table[((crc >> 8) ^ d[i]) & 0xFF]);
  return crc;
}

void dxl_init(UART_HandleTypeDef *huart)
{
  dxl_huart = huart;
  dxl_crc_init();
  HAL_HalfDuplex_EnableReceiver(dxl_huart);
}

/* Receive n bytes by polling the USART registers directly.
 * At 1 Mbps a byte arrives every 10 us (only 160 CPU cycles at 16 MHz).
 * HAL_UART_Receive() in STM32Cube FW_F4 >= 1.28.x does extra checks per byte and
 * ABORTS the whole transfer on the first overrun, so the reply was cut after ~7 bytes.
 * This loop is short enough to keep up; reading SR then DR also clears ORE/NE/FE.
 * Returns the number of bytes received (< n means timeout). */
static uint16_t dxl_rx(uint8_t *buf, uint16_t n, uint32_t timeout_ms)
{
  USART_TypeDef *u = dxl_huart->Instance;
  uint32_t t0 = HAL_GetTick();
  uint16_t got = 0;
  while (got < n) {
    if (u->SR & (USART_SR_RXNE | USART_SR_ORE)) {
      buf[got++] = (uint8_t)u->DR;
    } else if ((HAL_GetTick() - t0) > timeout_ms) {
      break;
    }
  }
  return got;
}

/* Send one instruction packet and (unless broadcast) receive the status packet.
 * nrp = number of parameter bytes expected in the reply. */
static int dxl_txrx(uint8_t id, uint8_t inst, const uint8_t *params, uint16_t np,
                    uint8_t *rparams, uint16_t nrp)
{
  uint16_t i = 0;
  txbuf[i++] = 0xFF; txbuf[i++] = 0xFF; txbuf[i++] = 0xFD; txbuf[i++] = 0x00;
  txbuf[i++] = id;
  i += 2;                                   /* LEN filled in below */
  txbuf[i++] = inst;
  for (uint16_t k = 0; k < np; k++) {
    if ((uint32_t)i + 4u > sizeof(txbuf)) return DXL_ERR_SIZE;
    txbuf[i++] = params[k];
    /* byte stuffing: FF FF FD -> FF FF FD FD */
    if (txbuf[i - 1] == 0xFD && txbuf[i - 2] == 0xFF && txbuf[i - 3] == 0xFF)
      txbuf[i++] = 0xFD;
  }
  uint16_t len = (uint16_t)(i - 7 + 2);     /* inst + params + CRC */
  txbuf[5] = len & 0xFF;
  txbuf[6] = len >> 8;
  uint16_t crc = dxl_crc(txbuf, i);
  txbuf[i++] = crc & 0xFF;
  txbuf[i++] = crc >> 8;

  uint16_t rlen = (uint16_t)(11 + nrp);
  if (rlen > sizeof(rxbuf)) return DXL_ERR_SIZE;

  /* flush any stale byte / overrun BEFORE transmitting, so the reply is never discarded */
  __HAL_UART_CLEAR_OREFLAG(dxl_huart);
  dxl_dbg_rx_got = 0;

  /* transmit */
  HAL_HalfDuplex_EnableTransmitter(dxl_huart);
  if (HAL_UART_Transmit(dxl_huart, txbuf, i, 10) != HAL_OK) {
    HAL_HalfDuplex_EnableReceiver(dxl_huart);
    return DXL_ERR_TX;
  }
  /* HAL_UART_Transmit returns after TC: line is free, switch to receive */
  HAL_HalfDuplex_EnableReceiver(dxl_huart);
  if (id == 0xFE) return DXL_OK;            /* broadcast: no status packet */

  memset(rxbuf, 0, rlen);
  dxl_dbg_rx_got = dxl_rx(rxbuf, rlen, DXL_RX_TIMEOUT_MS);
  memcpy(dxl_dbg_rx, rxbuf, sizeof(dxl_dbg_rx));
  if (dxl_dbg_rx_got < rlen)
    return DXL_ERR_TIMEOUT;

  /* validate status packet (note: byte-stuffed status packets are not handled) */
  if (rxbuf[0] != 0xFF || rxbuf[1] != 0xFF || rxbuf[2] != 0xFD ||
      rxbuf[4] != id   || rxbuf[7] != DXL_INST_STATUS)
    return DXL_ERR_PACKET;
  uint16_t rcrc = (uint16_t)(rxbuf[rlen - 2] | (rxbuf[rlen - 1] << 8));
  if (rcrc != dxl_crc(rxbuf, (uint16_t)(rlen - 2)))
    return DXL_ERR_CRC;
  if (rxbuf[8] & 0x7F)                      /* bit7 = hardware alert, ignored here */
    return rxbuf[8] & 0x7F;
  if (rparams && nrp) memcpy(rparams, &rxbuf[9], nrp);
  return DXL_OK;
}

int dxl_ping(uint8_t id, uint16_t *model)
{
  uint8_t p[3];
  int r = dxl_txrx(id, DXL_INST_PING, NULL, 0, p, 3);
  if (r == DXL_OK && model) *model = (uint16_t)(p[0] | (p[1] << 8));
  return r;
}

int dxl_write(uint8_t id, uint16_t addr, const uint8_t *data, uint16_t len)
{
  uint8_t p[34];
  if (len > 32) return DXL_ERR_SIZE;
  p[0] = addr & 0xFF;
  p[1] = addr >> 8;
  memcpy(&p[2], data, len);
  return dxl_txrx(id, DXL_INST_WRITE, p, (uint16_t)(len + 2), NULL, 0);
}

int dxl_write1(uint8_t id, uint16_t addr, uint8_t v)
{
  return dxl_write(id, addr, &v, 1);
}

int dxl_write2(uint8_t id, uint16_t addr, int16_t v)
{
  uint8_t d[2] = { (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF) };
  return dxl_write(id, addr, d, 2);
}

int dxl_write4(uint8_t id, uint16_t addr, int32_t v)
{
  uint32_t u = (uint32_t)v;
  uint8_t d[4] = { (uint8_t)u, (uint8_t)(u >> 8), (uint8_t)(u >> 16), (uint8_t)(u >> 24) };
  return dxl_write(id, addr, d, 4);
}

int dxl_read(uint8_t id, uint16_t addr, uint8_t *data, uint16_t len)
{
  uint8_t p[4] = { (uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8),
                   (uint8_t)(len & 0xFF),  (uint8_t)(len >> 8) };
  return dxl_txrx(id, DXL_INST_READ, p, 4, data, len);
}

static int32_t le32(const uint8_t *b)
{
  return (int32_t)((uint32_t)b[0] | ((uint32_t)b[1] << 8) |
                   ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24));
}

int dxl_read_state(uint8_t id, dxl_state_t *s)
{
  uint8_t d[27];                            /* addr 120..146 */
  int r = dxl_read(id, DXL_ADDR_REALTIME_TICK, d, 27);
  if (r != DXL_OK) return r;
  s->tick     = (uint16_t)(d[0] | (d[1] << 8));
  s->pwm      = (int16_t)(d[4] | (d[5] << 8));
  s->current  = (int16_t)(d[6] | (d[7] << 8));
  s->velocity = le32(&d[8]);    /* 128 */
  s->position = le32(&d[12]);   /* 132 */
  s->vel_traj = le32(&d[16]);   /* 136 */
  s->pos_traj = le32(&d[20]);   /* 140 */
  s->volt     = (uint16_t)(d[24] | (d[25] << 8));   /* 144 */
  s->temp     = d[26];                              /* 146 */
  return DXL_OK;
}

/* v4: short read for fast logging, addr 120..135 (tick, PWM, current, velocity, position).
 * 16 data bytes instead of 27 -> about 0.5 ms per read at 1 Mbps. */
int dxl_read_motion(uint8_t id, dxl_state_t *s)
{
  uint8_t d[16];
  int r = dxl_read(id, DXL_ADDR_REALTIME_TICK, d, 16);
  if (r != DXL_OK) return r;
  s->tick     = (uint16_t)(d[0] | (d[1] << 8));
  s->pwm      = (int16_t)(d[4] | (d[5] << 8));
  s->current  = (int16_t)(d[6] | (d[7] << 8));
  s->velocity = le32(&d[8]);    /* 128 */
  s->position = le32(&d[12]);   /* 132 */
  return DXL_OK;
}
