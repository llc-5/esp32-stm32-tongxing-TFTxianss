/* ============================================================================
 * STM32F103CBT6 接收端 —— ILI9341(240x320) 显示 ESP32-S31 摄像头画面
 *
 * 屏幕接线沿用原工程（ili9341.h 顶部）：
 *   PA0=RES  PA1=CS  PA2=DC  PA5=SCK  PA7=SDI  PA4=LED(背光)
 *   变化：PA5/PA7 现在交给硬件 SPI1 驱动（原来是一位位翻转 GPIO 模拟，
 *         写一帧要 390ms；改硬件后约 51ms，详见 ili9341.c）
 *
 * SPI 接收（SPI2 从机，纯寄存器初始化，不需要额外库文件）：
 *   PB13 = SPI2_SCK   （输入）
 *   PB15 = SPI2_MOSI  （输入）
 *   PB12 = SPI2_NSS   （输入，接 ESP32 的 CS）
 *   PB10 = READY 输出，通知 ESP32 可以发下一包
 *
 * STM32 只有 20KB SRAM，放不下 240x240x2=115200 字节整帧，
 * 所以协议按「一行一包」流送：收到一行立刻写屏，不存整帧。
 *
 * 为什么接收必须用 DMA：
 *   SPI 从机没有流控，时钟来了字节就得读走，读不及就 OVR 丢数据。
 *   8MHz 下一个字节只有 1us，而写一行屏要 200us 以上，靠中断或轮询
 *   逐字节收必然溢出——上一版 SPI 一包都收不到正是这个原因。
 *   SPI2_RX 固定挂在 DMA1_Channel4 上，整包 485 字节搬完才打扰 CPU。
 * ========================================================================== */

#include "stm32f10x.h"
#include "ili9341.h"
#include "link_proto.h"

/* ---- READY 握手（PB10）---- */
#define LINK_READY_PORT   GPIOB
#define LINK_READY_PIN    GPIO_Pin_10

/* ---- DMA1_Channel4（SPI2_RX）寄存器位。
 * 名字加 LINK_ 前缀：SPL 的 stm32f10x_dma.h 里已有 DMA_CCR_EN 等同名宏，
 * 直接重定义会报 "macro redefined"，这里避开。
 *
 * ⚠ STM32F1 的 DMA_CCR 位序和 F0/F3/L4 不一样，别凭印象写：
 *   bit0 EN, bit1 TCIE, bit2 HTIE, bit3 TEIE, bit4 DIR, bit5 CIRC,
 *   bit6 PINC, bit7 MINC, bit8:9 PSIZE, bit10:11 MSIZE
 * （写错成 1<<10 会落到 MSIZE 上，MINC 没置位，485 字节全写进 s_pkt[0]。） ---- */
#define LINK_DMA_CCR_EN        (1UL << 0)
#define LINK_DMA_CCR_TCIE      (1UL << 1)
#define LINK_DMA_CCR_MINC      (1UL << 7)    /* 内存地址递增，收包必需 */
#define LINK_DMA_ISR_TCIF4     (1UL << 13)   /* DMA1->ISR  ch4 传输完成 */
#define LINK_DMA_IFCR_CGIF4    (1UL << 12)   /* 写 1 清 ch4 全部标志 */
#define LINK_DMA_IFCR_CTCIF4   (1UL << 13)

/* ---- SPI2 寄存器位（同样加前缀避开 SPL 宏） ---- */
#define LINK_SPI_SR_RXNE       (1UL << 0)
#define LINK_SPI_CR1_SPE       (1UL << 6)
#define LINK_SPI_CR2_RXDMAEN   (1UL << 0)

/* 等包超时：约 150~200ms。失败不报错，下一轮重试即可；
 * 太短会误判，太长则链路没接时顶栏计数刷不出来。 */
#define LINK_WAIT_GUARD   0x00200000UL

static uint8_t s_pkt[LINK_PKT_BYTES];

/* 调试计数（放文件作用域，方便 show_debug 统一取用） */
static uint16_t n_ok, n_timeout, n_magic, n_xor, n_row, n_loop;
static uint8_t  n_page;
/* 排错用：一包里最多收到过多少字节（485 = 收满）。
 * 0 说明 SPI 时钟/片选根本没进来；485 说明收满了但校验没过。 */
static uint16_t s_rx_max;

/* --------------------------------------------------------------------------- */
/* SPI2 从机初始化：纯寄存器，本工程 Library 组只有 misc/gpio/rcc，
 * 不引入 stm32f10x_spi.c / stm32f10x_dma.c。 */
static void link_spi_init(void)
{
    GPIO_InitTypeDef gpio;

    /* 时钟：GPIOB = APB2 bit3，SPI2 = APB1 bit14，DMA1 = AHB bit0 */
    RCC->APB2ENR |= (1UL << 3);
    RCC->APB1ENR |= (1UL << 14);
    RCC->AHBENR  |= (1UL << 0);

    /* 从机侧的 SCK / MOSI / NSS 全部是「输入」，配成上拉输入即可：
     * STM32F1 的外设输入直接取自引脚，输入模式下外设照样能读到，
     * 不需要（也不应该）配成 AF_PP——AF_PP 是给主机输出用的。
     * 上拉保证没接线时 NSS 保持高电平（未选中），不会收到垃圾数据。 */
    gpio.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    /* PB10 = READY 输出 */
    gpio.GPIO_Pin   = LINK_READY_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(LINK_READY_PORT, LINK_READY_PIN);

    /* SPI2 从机 mode 0：MSTR=0 从机，CPOL=0，CPHA=0，DFF=0（8bit），
     * LSBFIRST=0（MSB 先），SSM=0（用硬件 NSS=PB12），BR 从机无效。
     * CR1 最后写、只置 SPE，其余位保持 0，正好就是这个配置。 */
    SPI2->CR1     = 0;
    SPI2->CR2     = LINK_SPI_CR2_RXDMAEN;   /* 收到的字节由 DMA 搬走 */
    SPI2->I2SCFGR = 0;                 /* 选 SPI 模式（不是 I2S） */
    SPI2->CR1     = LINK_SPI_CR1_SPE;
}

/* 武装接收：把 DMA1_Channel4 配成「485 字节、外设->内存、内存递增」。 */
static void link_dma_arm(void)
{
    DMA1_Channel4->CCR   = 0;                        /* 先关通道才能改参数 */
    DMA1->IFCR           = LINK_DMA_IFCR_CGIF4;           /* 清 ch4 全部标志 */
    DMA1_Channel4->CPAR  = (uint32_t)(&SPI2->DR);
    DMA1_Channel4->CMAR  = (uint32_t)s_pkt;
    DMA1_Channel4->CNDTR = LINK_PKT_BYTES;
    /* MINC + TCIE + EN；方向位=0 即外设->内存，PSIZE/MSIZE=00 即 8 位 */
    DMA1_Channel4->CCR   = LINK_DMA_CCR_MINC | LINK_DMA_CCR_TCIE | LINK_DMA_CCR_EN;
}

/* --------------------------------------------------------------------------- */
/* 收一整包（485 字节）：武装 DMA -> 拉高 READY -> 等 TCIF4 -> 立刻拉低 READY。
 * 返回 0 成功，1 超时。 */
static uint8_t link_recv_packet(void)
{
    uint32_t guard;

    /* 清上一包残留。OVR 必须「先读 DR 再读 SR」才能清掉，
     * 否则下一包会一直卡在溢出状态。 */
    if (SPI2->SR & LINK_SPI_SR_RXNE) {
        (void)SPI2->DR;
    }
    (void)SPI2->SR;
    (void)SPI2->DR;

    link_dma_arm();
    GPIO_SetBits(LINK_READY_PORT, LINK_READY_PIN);   /* READY=1，可以发 */

    guard = LINK_WAIT_GUARD;
    while ((DMA1->ISR & LINK_DMA_ISR_TCIF4) == 0) {
        if (--guard == 0) {
            /* 超时时把 DMA 还剩多少字节记下来，这是排错的关键数字：
             *   剩 485（收到 0 字节）→ 时钟/片选根本没进来，查 SCK、CS 接线
             *   剩 1~484           → 时钟通了但没收满，查 MOSI、线长、共地
             * 顶栏 R 显示的就是「一包里最多收到过多少字节」。 */
            uint16_t got = (uint16_t)(LINK_PKT_BYTES - DMA1_Channel4->CNDTR);
            if (got > s_rx_max) {
                s_rx_max = got;
            }
            DMA1_Channel4->CCR = 0;
            GPIO_ResetBits(LINK_READY_PORT, LINK_READY_PIN);
            return 1;                                /* 等包超时 */
        }
    }

    /* 收满立刻拉低：写屏这段时间里主机不会再发，避免覆盖 s_pkt */
    GPIO_ResetBits(LINK_READY_PORT, LINK_READY_PIN);
    DMA1_Channel4->CCR = 0;
    DMA1->IFCR = LINK_DMA_IFCR_CTCIF4;
    return 0;
}

/* 整包（含 magic/帧号/行号/校验字节/像素）异或应为 0 */
static uint8_t pkt_xor_ok(void)
{
    uint32_t i;
    uint8_t x = 0;
    for (i = 0; i < LINK_PKT_BYTES; i++) {
        x ^= s_pkt[i];
    }
    return (x == 0);
}

/* 十进制转字符串（不用 stdio，省代码量） */
static void put_num(char *buf, uint16_t v)
{
    uint8_t i = 0, j;
    char tmp[6];
    do { tmp[i++] = (char)('0' + (v % 10)); v /= 10; } while (v);
    for (j = 0; j < i; j++) buf[j] = tmp[i - 1 - j];
    buf[i] = ' ';
}

/* 顶栏调试显示（V7 = SPI 版）：一行大字，三页轮换。
 * 看到 "V7" 说明烧进去的是这一版；"V6" 是旧的 UART 版，两者接线不同。
 * 第三页的 R 是本版新加的关键排错数字：一包里最多收到过多少字节。
 *   R = 0    → SPI 时钟/片选根本没进来（查 SCK=GPIO45→PB13、CS=GPIO43→PB12）
 *   R 1~484  → 时钟通了但没收满（查 MOSI=GPIO44→PB15、共地、线长）
 *   R = 485  → 收满了，看 C/G 是否在涨（涨则是内容错，不是没收到） */
static void show_debug(void)
{
    char line[32];
    uint8_t p = 0;

    /* 必须填满空格：未初始化的栈里若有 0 会把字符串提前截断 */
    for (p = 0; p < sizeof(line); p++) {
        line[p] = ' ';
    }
    p = 0;

    LCD_FillArea(0, 0, LCD_WIDTH, LINK_SCREEN_Y, BLUE);

    line[p++] = 'V'; line[p++] = '7'; line[p++] = ' ';
    if (n_page == 0) {                /* K=有效行  T=收包超时 */
        line[p++] = 'K'; put_num(line + p, n_ok);      p += 6;
        line[p++] = 'T'; put_num(line + p, n_timeout); p += 6;
    } else if (n_page == 1) {         /* C=校验错  G=magic错 */
        line[p++] = 'C'; put_num(line + p, n_xor);     p += 6;
        line[p++] = 'G'; put_num(line + p, n_magic);   p += 6;
    } else {                          /* W=行号错  R=一包里收到过的最多字节数 */
        line[p++] = 'W'; put_num(line + p, n_row);     p += 6;
        line[p++] = 'R'; put_num(line + p, s_rx_max);  p += 6;
    }
    line[p] = 0;
    LCD_ShowStringAtC(2, 2, line, WHITE, BLUE, 2);

    if (++n_page >= 3) {
        n_page = 0;
    }
}

/* --------------------------------------------------------------------------- */
int main(void)
{
    uint8_t  expect_row = 0;
    uint8_t  first_frame_done = 0;
    uint8_t  tick = 0;
    uint8_t  idle = 0;

    LCD_SetColors(WHITE, BLUE);
    LCD_SetBgTransparent(0);
    LCD_Init();                       /* 内部已把 PA5/PA7 交给硬件 SPI1 */

    link_spi_init();

    LCD_FillArea(0, 0, LCD_WIDTH, LCD_HEIGHT, BLUE);
    show_debug();                     /* 开机就显示 V7 与计数 */

    while (1) {
        n_loop++;                     /* 心跳：主循环在跑就会变 */

        if (link_recv_packet() != 0) {
            n_timeout++;
            /* 链路没数据时也要定期刷顶栏，否则要等一帧的行数才更新一次，
             * 排错时看不出 T 在涨。8 次超时约 1.3 秒刷一次。 */
            if (++idle >= 8) {
                idle = 0;
                show_debug();
            }
            continue;
        }
        idle = 0;

        if ((s_pkt[0] != LINK_MAGIC1) || (s_pkt[1] != LINK_MAGIC2)) {
            n_magic++;
        } else if (!pkt_xor_ok()) {
            n_xor++;
        } else {
            /* 只有行 0 才能重新开始一帧；中途丢包就丢弃残帧 */
            if (s_pkt[3] == 0) {
                expect_row = 0;
            }
            if (s_pkt[3] != expect_row) {
                n_row++;
            } else {
                n_ok++;
                LCD_DrawRGB565Row(0, (uint16_t)(LINK_SCREEN_Y + s_pkt[3]),
                                  LINK_PIX_W, s_pkt + LINK_HDR_BYTES);
                expect_row++;

                if (s_pkt[3] == (uint8_t)(LINK_PIX_H - 1)) {
                    if (!first_frame_done) {
                        first_frame_done = 1;
                        LCD_FillArea(0, LINK_SCREEN_Y + LINK_PIX_H,
                                     LCD_WIDTH,
                                     LCD_HEIGHT - LINK_SCREEN_Y - LINK_PIX_H,
                                     BLUE);
                    }
                }
            }
        }

        /* 顶栏每帧刷一次。原来每 2 行刷一次（一帧 120 次），
         * 光刷顶栏就要 ~8 秒，是当时帧率的主要瓶颈；现在写屏走硬件 SPI1，
         * 一次顶栏约 8ms，一帧一次只占几个百分点。 */
        if (++tick >= LINK_PIX_H) {
            tick = 0;
            show_debug();
        }
    }
}
