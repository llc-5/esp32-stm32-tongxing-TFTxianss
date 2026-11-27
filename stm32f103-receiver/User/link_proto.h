#ifndef __LINK_PROTO_H
#define __LINK_PROTO_H

#include <stdint.h>

/* ============================================================================
 * ESP32-S31 -> STM32F103CBT6 摄像头画面传输协议（SPI 版，两端必须逐字一致）
 *
 * 一包 = 一整行像素，共 485 字节（ESP32 一次 SPI 事务发完，CS 包住整包）：
 *   [0]   0xA5        magic1
 *   [1]   0x5A        magic2（双 magic，便于错位后重新同步）
 *   [2]   frame_id    帧号 0~255 循环，仅调试用
 *   [3]   row         行号 0~239
 *   [4]   xor         pkt[2] ^ pkt[3] ^ 480 字节像素
 *   [5..484]          240 个 RGB565 像素，高字节在前
 *
 * 物理层：SPI mode 0（CPOL=0 / CPHA=0），MSB first，8 bit，ESP32 为主机。
 *
 * 接线（TFT 已从 ESP32 拔掉，原屏幕脚位复用为链路）：
 *   ESP32 GPIO45 (SCK   输出) -> STM32 PB13 (SPI2_SCK)
 *   ESP32 GPIO44 (MOSI  输出) -> STM32 PB15 (SPI2_MOSI)
 *   ESP32 GPIO43 (CS    输出) -> STM32 PB12 (SPI2_NSS)
 *   ESP32 GPIO42 (READY 输入) <- STM32 PB10 (READY 输出)
 *   GND 共地（必须接，否则高速时钟全是误码）
 *
 * 握手（沿用 UART 版已验证过的节奏，只是搬到了 SPI 上）：
 *   1. STM32 把 SPI2-RX 的 DMA 武装成 485 字节，然后拉高 READY
 *   2. ESP32 等 READY=1 -> CS 拉低 -> 送出 485 字节 -> CS 拉高
 *   3. STM32 的 DMA 收满立刻拉低 READY，校验 + 写屏，再回到第 1 步
 *
 * 为什么 STM32 必须用 DMA 收：
 *   SPI 从机没有流控，字节来了就读，读不及就 OVR 丢数据。8MHz 下
 *   一个字节只有 1us，而写一行屏要 200us 以上，靠中断/轮询逐字节收
 *   必然溢出——这正是之前 SPI 版一包都收不到的原因。
 *   DMA 整包搬完才打扰 CPU，从机侧就没有吞吐压力了。
 * ========================================================================== */

/* 画幅 240x240，屏上纵向居中（上下各 40 像素） */
#define LINK_PIX_W        240u
#define LINK_PIX_H        240u
#define LINK_ROW_BYTES    (LINK_PIX_W * 2u)                  /* 480 */
#define LINK_HDR_BYTES    5u
#define LINK_PKT_BYTES    (LINK_HDR_BYTES + LINK_ROW_BYTES)  /* 485 */
#define LINK_SCREEN_Y     40u

#define LINK_MAGIC1       0xA5u
#define LINK_MAGIC2       0x5Au

/* ESP32 主机时钟。
 * 8MHz 是杜邦线下的稳妥值（485B/行 = 485us，一帧约 116ms）。
 * 若实测 C/G 计数一直是 0，可以试 16000000UL（一帧约 58ms），
 * STM32 从机上限 18MHz；但杜邦线过长时会出误码，出问题就退回 8MHz。 */
#define LINK_SPI_HZ       8000000UL

#endif
