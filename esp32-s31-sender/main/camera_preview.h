#ifndef __CAMERA_PREVIEW_H
#define __CAMERA_PREVIEW_H

#include <stdint.h>
#include "esp_err.h"

/* ============================================================================
 * 摄像头模块（官方丝印 2x9 排针）-> ESP32-S31 -> ILI9341
 *
 * 官方丝印引脚顺序（从模块正面看，2 排 9 针）：
 *   上排：GND  SCL  SDA  D0  D2  D4  D6  PCLK  PWDN
 *   下排：3.3V VSYNC HREF RST D1  D3  D5  D7    FLASH
 *
 * 注意：模块自带有源晶振（24MHz）给传感器做 XCLK，排针上根本没有 XCLK 脚，
 *       所以 ESP32 不需要输出 XCLK，驱动里 xclk_io = -1、xclk_freq = 0。
 *       丝印型号写的是 OV5640（500W），固件同时保留 OV2640 自动探测。
 *
 * 接到这块超薄核心板（屏幕 40/42/43/44/45 不动，SCL=3 / SDA=4 已接）：
 *
 *   模块丝印   ESP32-S31 GPIO   核心板排针丝印
 *   GND             GND            G
 *   SCL              3             3       已接
 *   SDA              4             4       已接
 *   D0              35            35
 *   D2              38            38
 *   D4               0             0
 *   D6               2             2
 *   PCLK            46            46
 *   PWDN         不接             -
 *   3.3V          3.3V            3V3
 *   VSYNC           48            48
 *   HREF            49            49
 *   RST          不接             -
 *   D1              37            37      (strapping，见下)
 *   D3              39            39
 *   D5               1             1
 *   D7              47            47
 *   FLASH        不接             -
 *
 * ⚠ GPIO37 是 strapping 脚（JTAG 信号源选择），模块自带晶振、上电就会驱动
 *   数据线。出厂 eFuse 默认忽略 GPIO37 电平，所以正常安全；GPIO36 是
 *   VDD_SPI 电压检测脚，风险更大，本方案特意避开不用。
 *
 * 注：GPIO0/1/2/3/4 属于 LP GPIO，但 ESP32-S31 的 GPIO 交换矩阵寄存器覆盖
 *   全部 GPIO0~61，可以接到 LCD_CAM 的 DVP 信号上；已查 IDF 源码确认。
 *   之前用的 GPIO8~19、20~25 在这块板的排针上都没有，已全部作废。
 * ========================================================================== */

#define CAM_PIN_SCL        3
#define CAM_PIN_SDA        4
#define CAM_PIN_D0        35
#define CAM_PIN_D1        37
#define CAM_PIN_D2        38
#define CAM_PIN_D3        39
#define CAM_PIN_D4         0
#define CAM_PIN_D5         1
#define CAM_PIN_D6         2
#define CAM_PIN_D7        47
#define CAM_PIN_PCLK      46
#define CAM_PIN_VSYNC     48
#define CAM_PIN_HSYNC     49          /* 模块丝印 HREF */

/* 模块自带 24MHz 有源晶振，ESP32 不输出 XCLK */
#define CAM_PIN_XCLK      (-1)
#define CAM_XCLK_HZ       0

/* 显示区域：摄像头画面缩放到 240x240，在 240x320 屏上垂直居中（上下各 40 像素） */
#define CAM_FRAME_W       240
#define CAM_FRAME_H       240
#define CAM_SCREEN_X      0
#define CAM_SCREEN_Y      40

/* 若实测发现红蓝互换，把下面宏改成 0 */
#define CAM_SWAP_RGB565   1

/* ============================================================================
 * 摄像头 -> STM32 的 SPI 链路（TFT 已从 ESP32 拔掉，原屏幕脚位复用）
 *
 *   ESP32-S31 GPIO45 (SCK   输出) -> STM32 PB13 (SPI2_SCK)
 *   ESP32-S31 GPIO44 (MOSI  输出) -> STM32 PB15 (SPI2_MOSI)
 *   ESP32-S31 GPIO43 (CS    输出) -> STM32 PB12 (SPI2_NSS)
 *   ESP32-S31 GPIO42 (READY 输入) <- STM32 PB10 (READY 输出)
 *   GND 共地（必须）
 *
 * 这正是原来 ESP32 直接驱动 ILI9341 用的那四根线，一根没加也没换。
 * SPI mode 0，8MHz，MSB first；摄像头脚位完全不动。
 * ========================================================================== */
#define LINK_PIN_SCK      45
#define LINK_PIN_MOSI     44
#define LINK_PIN_CS       43
#define LINK_PIN_READY    42

esp_err_t camera_preview_init(void);
esp_err_t camera_preview_run(void);   /* 无限循环采帧并刷屏，出错才返回 */
esp_err_t camera_link_run(void);      /* 无限循环采帧并送给 STM32，出错才返回 */

/* 摄像头初始化失败时用的备胎：只跑 SPI 链路，发彩条测试图案。
 * 用来单独验证两板连线，不依赖摄像头是否接触良好。 */
esp_err_t camera_link_selftest(void);

#endif
