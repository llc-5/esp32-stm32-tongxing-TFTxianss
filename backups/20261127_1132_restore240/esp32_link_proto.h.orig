#ifndef __LINK_PROTO_H
#define __LINK_PROTO_H

#include <stdint.h>

/* ============================================================================
 * ESP32-S31 -> STM32F103CBT6 摄像头画面传输协议（UART 版，两端必须一致）
 *
 * 一包 = 一整行像素，共 245 字节：
 *   [0]   0xA5        magic1
 *   [1]   0x5A        magic2（双 magic，便于错位后重新同步）
 *   [2]   frame_id    帧号 0~255 循环，仅调试用
 *   [3]   row         行号 0~119
 *   [4]   xor         pkt[2] ^ pkt[3] ^ 240 字节像素
 *   [5..244]          120 个 RGB565 像素，高字节在前
 *
 * 接线：
 *   ESP32 GPIO43 (UART1 TX) -> STM32 PA10 (USART1 RX)
 *   ESP32 GPIO42 (READY 输入) <- STM32 PB10 (READY 输出)
 *   GND 共地
 *
 * 握手：STM32 写完一行后拉高 READY；ESP32 等 READY=1 后发一包；
 *       STM32 收到 magic1 立即拉低 READY，防止下一包覆盖。
 * ========================================================================== */

/* 画幅缩减优化：240x240 -> 120x120，每帧数据量降到 1/4。
 * 摄像头仍按 240x240 采集，ESP32 在 PSRAM 里缩放到 120x120 再发。
 * 120x120 在 240x320 屏上水平居中(x=60)、垂直居中(y=100)。 */
#define LINK_PIX_W        120u
#define LINK_PIX_H        120u
#define LINK_ROW_BYTES    (LINK_PIX_W * 2u)              /* 240 */
#define LINK_HDR_BYTES    5u
#define LINK_PKT_BYTES    (LINK_HDR_BYTES + LINK_ROW_BYTES)  /* 245 */
#define LINK_SCREEN_X     60u
#define LINK_SCREEN_Y     100u

#define LINK_MAGIC1       0xA5u
#define LINK_MAGIC2       0x5Au

/* 波特率翻倍：2M -> 4M，UART 理论上限从 1.7fps 提到 3.4fps。
 * 与画幅缩减叠加后预期 ~8fps。若杜邦线丢字节(C/G 计数涨)，
 * 两端同时改回 2000000UL 重编重烧。 */
#define LINK_UART_BAUD    4000000UL

#endif
