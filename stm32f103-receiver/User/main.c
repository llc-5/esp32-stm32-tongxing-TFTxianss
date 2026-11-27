/* ============================================================================
 * STM32F103CBT6 接收端 —— ILI9341(240x320) 显示 ESP32-S31 摄像头画面
 *
 * 屏幕接线沿用原工程（ili/ili9341.h 顶部）：
 *   PA0=RES  PA1=CS  PA2=DC  PA5=SCK  PA7=SDI  PA4=LED(背光)
 *
 * 新增 UART 接收（USART1，纯寄存器初始化，不需要额外库文件）：
 *   PA10 = USART1 RX   8N1 @ 2,000,000 baud
 *   PB10 = READY 输出，通知 ESP32 可以发下一包
 *
 * STM32 只有 20KB SRAM，放不下 240x240x2=115200 字节整帧，
 * 所以协议按「一行一包」流送：收到一行立刻写屏，不存整帧。
 * ========================================================================== */

#include "stm32f10x.h"
#include "ili9341.h"
#include "link_proto.h"

#define LINK_READY_PORT   GPIOB
#define LINK_READY_PIN    GPIO_Pin_10

static uint8_t s_pkt[LINK_PKT_BYTES];

/* --------------------------------------------------------------------------- */
/* 纯寄存器初始化：本工程 Library 组只编了 misc/gpio/rcc，没有 stm32f10x_usart.c，
 * 所以不调用 USART_Init()/USART_Cmd()，免得还要往 Keil 里加库文件。 */
static void link_usart_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB, ENABLE);

    /* PA10 = USART1 RX。上拉输入：空闲时保持高电平，符合 UART 空闲态 */
    gpio.GPIO_Pin   = GPIO_Pin_10;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PB10 = READY 输出，拉高表示「我准备好了，可以发下一行」 */
    gpio.GPIO_Pin   = GPIO_Pin_10;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(LINK_READY_PORT, LINK_READY_PIN);

    /* USART1 挂在 APB2，蓝本工程 SystemInit 把 PCLK2 配成 72MHz。
     * BRR = PCLK2 / 波特率 = 72M / 2M = 36 (0x24)，8N1、16 倍过采样、只开 RX。 */
    USART1->BRR  = (uint16_t)(72000000UL / LINK_UART_BAUD);
    USART1->CR1  = USART_CR1_RE | USART_CR1_UE;
    USART1->CR2  = 0;                 /* 1 停止位 */
    USART1->CR3  = 0;                 /* 无硬件流控 */
}

/* --------------------------------------------------------------------------- */
/* 收一整包（485 字节）。
 *
 * 流程：拉高 READY -> 等首字节 -> 收到立刻拉低 READY -> 收完剩余字节。
 * ESP32 只有看到 READY=0 才认为这一包被接住，下一包会重新等 READY=1。
 * 因此写屏期间 READY 保持低，主机不会发下一包覆盖。
 */
static uint8_t link_recv_packet(void)
{
    uint32_t guard;
    uint32_t i;
    uint8_t b;

    /* 丢掉上一包残留字节，避免行数据错位 */
    while (USART1->SR & USART_FLAG_RXNE) {
        b = (uint8_t)USART1->DR;
        (void)b;
    }

    GPIO_SetBits(LINK_READY_PORT, LINK_READY_PIN);        /* READY=1，请发下一包 */

    guard = 0x00200000UL;   /* 整包共用一个超时，不要每字节重置 */
    while (!(USART1->SR & USART_FLAG_RXNE)) {
        if (--guard == 0) {
            GPIO_ResetBits(LINK_READY_PORT, LINK_READY_PIN);
            return 1;                                     /* 等首字节超时 */
        }
    }
    b = (uint8_t)USART1->DR;
    GPIO_ResetBits(LINK_READY_PORT, LINK_READY_PIN);      /* 收到首字节，立刻拉低 */
    s_pkt[0] = b;

    for (i = 1; i < LINK_PKT_BYTES; i++) {
        while (!(USART1->SR & USART_FLAG_RXNE)) {
            if (--guard == 0) {
                return 1;                                 /* 主机中途断流 */
            }
        }
        s_pkt[i] = (uint8_t)USART1->DR;
    }
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

/* 顶栏调试显示（V6 = UART 版）：一行大字，三页轮换。
 * 看到 "V6" 说明跑的是这一版；"V5" 是旧的 SPI 版。 */
static void show_debug(uint16_t ok, uint16_t to, uint16_t cr,
                       uint16_t mg, uint16_t rw, uint16_t loop, uint8_t page)
{
    char line[32];
    uint8_t p = 0;

    /* 必须填满空格：未初始化的栈里若有 0 会把字符串提前截断 */
    for (p = 0; p < sizeof(line); p++) {
        line[p] = ' ';
    }
    p = 0;

    LCD_FillArea(0, 0, LCD_WIDTH, LINK_SCREEN_Y, BLUE);

    line[p++] = 'V'; line[p++] = '6'; line[p++] = ' ';
    if (page == 0) {                  /* K=有效行  T=收包超时 */
        line[p++] = 'K'; put_num(line + p, ok);  p += 6;
        line[p++] = 'T'; put_num(line + p, to);  p += 6;
    } else if (page == 1) {           /* C=校验错  G=magic错 */
        line[p++] = 'C'; put_num(line + p, cr);  p += 6;
        line[p++] = 'G'; put_num(line + p, mg);  p += 6;
    } else {                          /* W=行号错  L=主循环心跳 */
        line[p++] = 'W'; put_num(line + p, rw);    p += 6;
        line[p++] = 'L'; put_num(line + p, loop);  p += 6;
    }
    line[p] = 0;
    LCD_ShowStringAtC(2, 2, line, WHITE, BLUE, 2);
}

/* --------------------------------------------------------------------------- */
int main(void)
{
    uint8_t expect_row = 0;
    uint8_t first_frame_done = 0;
    uint16_t n_ok = 0, n_timeout = 0, n_magic = 0, n_xor = 0, n_row = 0;
    uint16_t n_loop = 0;
    uint16_t tick = 0;

    LCD_SetColors(WHITE, BLUE);
    LCD_SetBgTransparent(0);
    LCD_Init();                       /* 沿用原工程的 ILI9341 初始化序列 */

    link_usart_init();

    LCD_FillArea(0, 0, LCD_WIDTH, LCD_HEIGHT, BLUE);
    show_debug(0, 0, 0, 0, 0, 0, 0);  /* 开机就显示 V6 与计数 */

    while (1) {
        n_loop++;                     /* 心跳：主循环在跑就会变 */

        if (link_recv_packet() != 0) {
            n_timeout++;
        } else if ((s_pkt[0] != LINK_MAGIC1) || (s_pkt[1] != LINK_MAGIC2)) {
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

        /* 每 2 圈刷新一次顶栏，三页轮换 */
        tick++;
        if (tick >= 2) {
            tick = 0;
            show_debug(n_ok, n_timeout, n_xor, n_magic, n_row,
                       (uint16_t)(n_loop % 1000),
                       (uint8_t)((n_loop / 2) % 3));
        }
    }
}
