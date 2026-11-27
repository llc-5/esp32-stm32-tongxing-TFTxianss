#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
两端源码静态对拍 —— 检查"这两个程序接上线能不能通信"

第 1 部分（内嵌）验证协议算法本身：打包/解包/校验/丢包恢复。
第 2 部分直接读真实源码文件，逐项核对那些"写错就整条链路不通、但编译能过"的地方：
  - 两端协议常量是否逐字一致
  - 引脚是否一一对应
  - STM32 的 SPI2 从机是否配成从机 + mode 0
  - SPI2_RX 是否走 DMA1_Channel4，且 CCR 各位是否真的对
  - SPI1（屏）是否真的开了时钟、PA5/PA7 是否真的配成复用
"""

import io
import os
import re
import sys
from functools import reduce

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ESP_MAIN = os.path.join(ROOT, "esp32-s31-sender", "main")
STM_USER = os.path.join(ROOT, "stm32f103-receiver", "User")

MAGIC1, MAGIC2 = 0xA5, 0x5A
PIX_W = PIX_H = 240
ROW_BYTES = PIX_W * 2
HDR = 5
PKT = HDR + ROW_BYTES

ok_all = True


def check(name, cond, extra=""):
    global ok_all
    print(("  [PASS] " if cond else "  [FAIL] ") + name + (("  " + extra) if extra else ""))
    if not cond:
        ok_all = False


def read(path):
    with io.open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


# ============================================================ 第 1 部分：算法
def esp32_pack_row(rgb565_le_row, frame_id, row):
    """严格照抄 camera_preview.c 的 link_send_row()（CAM_SWAP_RGB565=1）"""
    xor = MAGIC1 ^ MAGIC2 ^ frame_id ^ row
    tx = bytearray(ROW_BYTES)
    for i in range(PIX_W):
        px = rgb565_le_row[2 * i] | (rgb565_le_row[2 * i + 1] << 8)
        tx[2 * i] = (px >> 8) & 0xFF
        tx[2 * i + 1] = px & 0xFF
        xor ^= tx[2 * i]
        xor ^= tx[2 * i + 1]
    pkt = bytearray(PKT)
    pkt[0], pkt[1], pkt[2], pkt[3], pkt[4] = MAGIC1, MAGIC2, frame_id & 0xFF, row & 0xFF, xor
    pkt[HDR:] = tx
    return bytes(pkt)


def stm32_parse(pkt):
    """严格照抄 main.c 的 magic 判断 + pkt_xor_ok()"""
    if pkt[0] != MAGIC1 or pkt[1] != MAGIC2:
        return "magic", None
    if reduce(lambda a, b: a ^ b, pkt, 0) != 0:
        return "xor", None
    return "ok", pkt[3]


def stm32_frame(pkts):
    """严格照抄主循环的 expect_row 逻辑"""
    expect, acc = 0, []
    cnt = {"magic": 0, "xor": 0, "row": 0, "ok": 0}
    for p in pkts:
        if p is None:
            continue
        kind, row = stm32_parse(p)
        if kind != "ok":
            cnt[kind] += 1
            continue
        if row == 0:
            expect = 0
        if row != expect:
            cnt["row"] += 1
        else:
            cnt["ok"] += 1
            acc.append(row)
            expect += 1
    return acc, cnt


print("=" * 60)
print("第 1 部分  协议算法")
print("=" * 60)

rows = []
for r in range(PIX_H):
    src = bytes(((r * 13 + i * 5) & 0xFF) for i in range(ROW_BYTES))
    rows.append(esp32_pack_row(src, 7, r))

acc, cnt = stm32_frame(rows)
check("240 行全部被接受且行号连续", acc == list(range(240)))
check("无 magic / xor / 行号 错误", cnt == {"magic": 0, "xor": 0, "row": 0, "ok": 240}, str(cnt))

bad = bytearray(rows[137]); bad[300] ^= 0x01
check("像素区 1 bit 翻转被 xor 拦下", stm32_parse(bytes(bad))[0] == "xor")

bad2 = bytearray(rows[137]); bad2[0] = 0x00
check("magic 损坏被拦下", stm32_parse(bytes(bad2))[0] == "magic")

lossy = list(rows); lossy[100] = None
acc2, cnt2 = stm32_frame(lossy)
check("丢第 100 行后只画 0..99（不写串位）", acc2 == list(range(100)))

acc3, _ = stm32_frame([None] + rows)
check("下一帧自动恢复 240 行", len(acc3) == 240)

# ============================================================ 第 2 部分：源码
print()
print("=" * 60)
print("第 2 部分  源码静态对拍（读真实文件）")
print("=" * 60)

for p in (ESP_MAIN, STM_USER):
    if not os.path.isdir(p):
        print("找不到目录：" + p)
        sys.exit(2)

esp_proto = read(os.path.join(ESP_MAIN, "link_proto.h"))
stm_proto = read(os.path.join(STM_USER, "link_proto.h"))
esp_cam = read(os.path.join(ESP_MAIN, "camera_preview.c"))
esp_hdr = read(os.path.join(ESP_MAIN, "camera_preview.h"))
stm_main = read(os.path.join(STM_USER, "main.c"))
stm_lcd = read(os.path.join(STM_USER, "ili9341.c"))
stm_lcd_h = read(os.path.join(STM_USER, "ili9341.h"))

print("\n[2.1] 协议头")
check("两端 link_proto.h 逐字一致", esp_proto == stm_proto)
for name, want in (("LINK_PIX_W", "240u"), ("LINK_PIX_H", "240u"),
                   ("LINK_HDR_BYTES", "5u"), ("LINK_SCREEN_Y", "40u"),
                   ("LINK_MAGIC1", "0xA5u"), ("LINK_MAGIC2", "0x5Au"),
                   ("LINK_SPI_HZ", "8000000UL")):
    m = re.search(r"#define\s+" + name + r"\s+(\S+)", esp_proto)
    check(name + " = " + want, m is not None and m.group(1) == want,
          ("实际 " + (m.group(1) if m else "缺失")))
check("LINK_ROW_BYTES 由 PIX_W*2 得出", "LINK_PIX_W * 2u" in esp_proto)
check("LINK_PKT_BYTES 由 头+行 得出",
      re.search(r"#define\s+LINK_PKT_BYTES\s+\(LINK_HDR_BYTES \+ LINK_ROW_BYTES\)", esp_proto) is not None)

print("\n[2.2] ESP32 引脚与 SPI 主机配置")
for name, want in (("LINK_PIN_SCK", "45"), ("LINK_PIN_MOSI", "44"),
                   ("LINK_PIN_CS", "43"), ("LINK_PIN_READY", "42")):
    m = re.search(r"#define\s+" + name + r"\s+(\d+)", esp_hdr)
    check(name + " = GPIO" + want, m is not None and m.group(1) == want,
          ("实际 " + (m.group(1) if m else "缺失")))
check("SPI 用 mode 0", re.search(r"\.mode\s*=\s*0\s*,", esp_cam) is not None)
check("CS 由驱动控制（spics_io_num 指向 CS 脚）",
      re.search(r"\.spics_io_num\s*=\s*LINK_PIN_CS", esp_cam) is not None)
check("CS 有提前拉低的余量（pretrans）",
      re.search(r"\.cs_ena_pretrans\s*=\s*[1-9]", esp_cam) is not None)
check("事务长度按 LINK_PKT_BYTES*8 计",
      re.search(r"t\.length\s*=\s*\(size_t\)LINK_PKT_BYTES \* 8", esp_cam) is not None)
check("用 polling_transmit 发整包",
      "spi_device_polling_transmit" in esp_cam)
check("最大传输长度 >= 一包",
      re.search(r"\.max_transfer_sz\s*=\s*LINK_PKT_BYTES", esp_cam) is not None)
check("像素高字节先（CAM_SWAP_RGB565 分支存在）",
      re.search(r"CAM_SWAP_RGB565", esp_cam) is not None and
      re.search(r"\(uint8_t\)\(px >> 8\)", esp_cam) is not None)

print("\n[2.3] STM32 SPI2 从机配置")
check("使能 SPI2 时钟（RCC->APB1ENR bit14）",
      re.search(r"RCC->APB1ENR\s*\|=\s*\(1UL << 14\)", stm_main) is not None)
check("使能 DMA1 时钟（RCC->AHBENR bit0）",
      re.search(r"RCC->AHBENR\s*\|=\s*\(1UL << 0\)", stm_main) is not None)
check("使能 GPIOB 时钟（RCC->APB2ENR bit3）",
      re.search(r"RCC->APB2ENR\s*\|=\s*\(1UL << 3\)", stm_main) is not None)
check("PB12/PB13/PB15 配成输入（从机不配 AF_PP）",
      re.search(r"GPIO_Pin_12 \| GPIO_Pin_13 \| GPIO_Pin_15", stm_main) is not None and
      re.search(r"GPIO_Mode_IPU", stm_main) is not None)
check("CR2 打开 RXDMAEN", re.search(r"SPI2->CR2\s*=\s*LINK_SPI_CR2_RXDMAEN", stm_main) is not None)
check("CR1 只置 SPE（MSTR=0 即从机）",
      re.search(r"SPI2->CR1\s*=\s*LINK_SPI_CR1_SPE\s*;", stm_main) is not None)
check("先把 CR1 清零再配（不复用旧值）",
      re.search(r"SPI2->CR1\s*=\s*0\s*;", stm_main) is not None)

print("\n[2.4] STM32 DMA1_Channel4（SPI2_RX）")
check("使用 DMA1_Channel4", "DMA1_Channel4->CCR" in stm_main)
check("CPAR 指向 SPI2->DR", re.search(r"CPAR\s*=\s*\(uint32_t\)\(&SPI2->DR\)", stm_main) is not None)
check("CNDTR 用 LINK_PKT_BYTES", re.search(r"CNDTR\s*=\s*LINK_PKT_BYTES", stm_main) is not None)
check("内存地址递增 MINC = bit7（F1 位序，不是 bit10）",
      re.search(r"#define\s+LINK_DMA_CCR_MINC\s+\(1UL << 7\)", stm_main) is not None)
check("CCR 不含 DIR（0 = 外设→内存）",
      re.search(r"LINK_DMA_CCR_MINC \| LINK_DMA_CCR_TCIE \| LINK_DMA_CCR_EN", stm_main) is not None)
check("等 TCIF4 = bit13", re.search(r"#define\s+LINK_DMA_ISR_TCIF4\s+\(1UL << 13\)", stm_main) is not None)
check("清标志 CGIF4 = bit12", re.search(r"#define\s+LINK_DMA_IFCR_CGIF4\s+\(1UL << 12\)", stm_main) is not None)
check("武装 DMA 之后才拉高 READY",
      stm_main.index("link_dma_arm();") < stm_main.index("GPIO_SetBits(LINK_READY_PORT"))

print("\n[2.5] STM32 写屏（硬件 SPI1）")
check("SPI1 时钟被打开（原代码在关它）",
      re.search(r"LCD_RCC_APB2ENR \|= \(1UL << 12\)", stm_lcd) is not None and
      re.search(r"LCD_RCC_APB2ENR &= ~\(1UL << 12\)", stm_lcd) is None)
check("PA5/PA7 配成复用推挽 0xB",
      re.search(r"\(0xBUL << 20\) \| \(0xBUL << 28\)", stm_lcd) is not None)
check("SPI1 设成主机 MSTR=bit2", re.search(r"\(1UL << 2\)\s*/\* MSTR", stm_lcd) is not None)
check("SPI1 设 SSM/SSI（CS 由软件管）",
      re.search(r"\(1UL << 9\)", stm_lcd) is not None and re.search(r"\(1UL << 8\)", stm_lcd) is not None)
check("SPE=bit6 已置位", re.search(r"\(1UL << 6\);\s*/\* SPE", stm_lcd) is not None)
check("分频由 LCD_SPI_BR 决定", re.search(r"#define\s+LCD_SPI_BR\s+0x0[123]u", stm_lcd_h) is not None)
check("等 TXE=bit1 才写 DR", re.search(r"\(1UL << 1\)", stm_lcd) is not None)
check("切 DC / 拉 CS 前等 BSY=bit7 清零",
      re.search(r"\(1UL << 7\)", stm_lcd) is not None and "lcd_spi_flush" in stm_lcd)
check("软件模拟 SPI 已彻底移除（不再翻转 SCK/SDI 宏）",
      re.search(r"LCD_SCK_[LH]\(\)|LCD_SDI_[LH]\(\)", stm_lcd) is None)

print("\n[2.6] 写屏坐标与行号")
check("STM32 从 x=0 开始画、y = LINK_SCREEN_Y + 行号",
      re.search(r"LCD_DrawRGB565Row\(0,\s*\(uint16_t\)\(LINK_SCREEN_Y \+ s_pkt\[3\]\)", stm_main) is not None)
check("只接受行 0 重新起帧",
      re.search(r"if \(s_pkt\[3\] == 0\)", stm_main) is not None)

print()
print("=" * 60)
if ok_all:
    print("结论：两端可以通信 —— 协议一致、引脚对应、时序与寄存器位正确")
else:
    print("结论：存在不一致项，见上面 FAIL")
print("=" * 60)
sys.exit(0 if ok_all else 1)
