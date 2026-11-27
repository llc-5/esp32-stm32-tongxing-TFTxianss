#ifndef __ILI9341_H
#define __ILI9341_H

 

#include <stdint.h>

 
 
#define LCD_PIN_MUX_BY_LABEL   0

 
#define LCD_LED_CTRL      1
#define LCD_LED_PIN       4

 
#define LCD_USE_EXTENDED_INIT  1

 
#define LCD_HCLK_HZ   72000000UL

 
#define LCD_MADCTL_NORMAL   0x48
#define LCD_MADCTL_FLIPPED  0x08
#define LCD_MADCTL_ALT      0xC8
#define LCD_MADCTL_ALT2     0x88
#define LCD_MADCTL_VALUE    LCD_MADCTL_NORMAL

 
#define WHITE  0xFFFF
#define BLACK  0x0000
#define RED    0xF800
#define GREEN  0x07E0
#define BLUE   0x001F
#define YELLOW 0xFFE0

 
#define LCD_FG_COLOR   RED
#define LCD_BG_COLOR   BLUE

#define LCD_WIDTH   240
#define LCD_HEIGHT  320

 
typedef struct {
    volatile uint32_t CRL;
    volatile uint32_t CRH;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t BRR;
    volatile uint32_t LCKR;
} LCD_GPIO_Regs;

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
} LCD_SysTick_Regs;

#define LCD_GPIOA         ((LCD_GPIO_Regs *)0x40010800U)
#define LCD_RCC_APB2ENR   (*(volatile uint32_t *)0x40021018U)
#define LCD_SYSTICK       ((LCD_SysTick_Regs *)0xE000E010U)

/* ---- 硬件 SPI1（PA5=SCK, PA7=MOSI，正好是本屏原来用的两根脚）---- */
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
} LCD_SPI_Regs;

#define LCD_SPI           ((LCD_SPI_Regs *)0x40013000U)   /* SPI1 基址 */

/* 波特率分频：BR[2:0]，SPI1 挂 APB2 = 72MHz
 *   000 -> /2  = 36MHz   001 -> /4  = 18MHz
 *   010 -> /8  =  9MHz   011 -> /16 = 4.5MHz
 * ILI9341 手册写周期最快约 100ns(10MHz)，18MHz 是大量模块的常用值；
 * 若出现花屏/错位就改成 0x02（9MHz）。 */
#define LCD_SPI_BR        0x01u

#define PIN_RES   0
#define PIN_CS    1
#define PIN_SDI   7
#define PIN_SCK   5
#define PIN_DC    2

#define LCD_RES_H()   (LCD_GPIOA->BSRR = (1UL << PIN_RES))
#define LCD_RES_L()   (LCD_GPIOA->BSRR = (1UL << (PIN_RES + 16)))
#define LCD_CS_H()    (LCD_GPIOA->BSRR = (1UL << PIN_CS))
#define LCD_CS_L()    (LCD_GPIOA->BSRR = (1UL << (PIN_CS + 16)))
#define LCD_DC_H()    (LCD_GPIOA->BSRR = (1UL << PIN_DC))
#define LCD_DC_L()    (LCD_GPIOA->BSRR = (1UL << (PIN_DC + 16)))
#define LCD_SCK_H()   (LCD_GPIOA->BSRR = (1UL << PIN_SCK))
#define LCD_SCK_L()   (LCD_GPIOA->BSRR = (1UL << (PIN_SCK + 16)))
#define LCD_SDI_H()   (LCD_GPIOA->BSRR = (1UL << PIN_SDI))
#define LCD_SDI_L()   (LCD_GPIOA->BSRR = (1UL << (PIN_SDI + 16)))

#if LCD_LED_CTRL
#define LCD_LED_H()   (LCD_GPIOA->BSRR = (1UL << LCD_LED_PIN))
#define LCD_LED_L()   (LCD_GPIOA->BSRR = (1UL << (LCD_LED_PIN + 16)))
#endif

 
void LCD_Init(void);
void LCD_Fill(uint16_t color);
void LCD_ShowChar(uint16_t x, uint16_t y, char ch,
                  uint16_t fc, uint16_t bc, uint8_t size);
void LCD_ShowString(uint16_t x, uint16_t y, const char *s,
                    uint16_t fc, uint16_t bc, uint8_t size);
void LCD_ShowStringCentered(const char *s,
                            uint16_t fc, uint16_t bc, uint8_t size);

 
void LCD_ShowCharC(uint16_t x, uint16_t y, char ch,
                   uint16_t fc, uint16_t bc, uint8_t size);
void LCD_ShowStringAtC(uint16_t x, uint16_t y, const char *s,
                       uint16_t fc, uint16_t bc, uint8_t size);

 
uint16_t LCD_RGB(uint8_t r, uint8_t g, uint8_t b);
void     LCD_SetColors(uint16_t fg, uint16_t bg);
void     LCD_ShowStringCenteredC(const char *s, uint8_t size);
void     LCD_ClearScreen(void);
void     LCD_FillArea(uint16_t x, uint16_t y,
                      uint16_t w, uint16_t h, uint16_t color);

/* 画一行 RGB565 像素（摄像头链路用）。
 * data 每像素 2 字节，高字节在前；每次单独开窗，保证行号不串位。 */
void     LCD_DrawRGB565Row(uint16_t x, uint16_t y, uint16_t w, const uint8_t *data);
void     LCD_SetBgTransparent(uint8_t enable);

 
void LCD_ShowImage256(uint16_t x, uint16_t y,
                      uint16_t w, uint16_t h,
                      const uint16_t *palette, const uint8_t *img);

#endif
