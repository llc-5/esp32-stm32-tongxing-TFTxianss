#ifndef __LINK_PROTO_H
#define __LINK_PROTO_H

#include <stdint.h>

/* ============================================================================
 * ESP32-S31 -> STM32F103CBT6 摄像头画面传输协议（UART 版，两端必须一致）
 *
 * 一包 = 一整行像素，共 485 字节：
 *   [0]   0xA5        magic1
 *   [1]   0x5A        magic2（双 magic，便于错位后重新同步）
 *   [2]   frame_id    帧号 0~255 循环，仅调试用
 *   [3]   row         行号 0~239
 *   [4]   xor         pkt[2] ^ pkt[3] ^ 480 字节像素
 *   [5..484]          240 个 RGB565 像素，高字节在前
 *
 * 接线：
 *   ESP32 GPIO43 (UART1 TX) -> STM32 PA10 (USART1 RX)
 *   ESP32 GPIO42 (READY 输入) <- STM32 PB10 (READY 输出)
 *   GND 共地
 *
 * 握手：STM32 写完一行后拉高 READY；ESP32 等 READY=1 后发一包；
 *       STM32 收到 magic1 立即拉低 READY，防止下一包覆盖。
 * ========================================================================== */

/* 已复原为原始 240x240 画幅、2Mbps 波特率（2026-11-27）。
 * 这是实测能稳定出图的配置；120x120 + 4M 的小画幅方案已撤销。 */
#define LINK_PIX_W        240u
#define LINK_PIX_H        240u
#define LINK_ROW_BYTES    (LINK_PIX_W * 2u)              /* 480 */
#define LINK_HDR_BYTES    5u
#define LINK_PKT_BYTES    (LINK_HDR_BYTES + LINK_ROW_BYTES)  /* 485 */
#define LINK_SCREEN_Y     40u

#define LINK_MAGIC1       0xA5u
#define LINK_MAGIC2       0x5Au

#define LINK_UART_BAUD    2000000UL

#endif
