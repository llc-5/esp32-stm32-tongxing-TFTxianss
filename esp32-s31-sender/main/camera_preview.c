/* ============================================================================
 * DVP 摄像头（OV5640 / OV2640 自动识别）-> UART -> STM32F103CBT6 -> ILI9341
 *
 * 模块自带 24MHz 有源晶振，排针没有 XCLK 脚，所以 ESP32 不输出 XCLK。
 * 底层用乐鑫官方 esp_video / esp_cam_sensor 组件（V4L2 风格接口），
 * 不自己写寄存器序列、不自己拼 DVP 时序。摄像头引脚在 camera_preview.h，
 * 与原来的工程完全一致，一根没动；原来接 TFT 的 43 改 UART1 TX、42 改 READY。
 *
 * OV2640 请求 240x240 RGB565；OV5640 只给 800x600，就在 PSRAM 里
 * 居中裁剪 + 最近邻缩放到 240x240，再逐行送给 STM32。
 * ========================================================================== */

#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "camera_preview.h"
#include "link_proto.h"

/* 原「ESP32 直接刷屏」的演示代码保留但默认不编译，方便需要时切回去 */
#define CAM_KEEP_LOCAL_PREVIEW   0
#if CAM_KEEP_LOCAL_PREVIEW
#include "ili9341.h"
#include "font.h"
#endif

static const char *TAG = "CAM_LINK";
#define BUFFER_COUNT      2

static const esp_video_init_dvp_config_t s_dvp_cfg = {
    .sccb_config = {
        .init_sccb = true,
        .i2c_config = {
            .port    = 0,
            .scl_pin = CAM_PIN_SCL,
            .sda_pin = CAM_PIN_SDA,
        },
        .freq = 50000,         /* 长杜邦线时 100kHz 容易读不到传感器，降到 50kHz */
    },
    .reset_pin = -1,       /* 模块 RST 不接，靠上电复位 */
    .pwdn_pin  = -1,       /* 模块 PWDN 不接 */
    .dvp_pin = {
        .data_width = CAM_CTLR_DATA_WIDTH_8,
        .data_io = {
            CAM_PIN_D0, CAM_PIN_D1, CAM_PIN_D2, CAM_PIN_D3,
            CAM_PIN_D4, CAM_PIN_D5, CAM_PIN_D6, CAM_PIN_D7,
        },
        .vsync_io = CAM_PIN_VSYNC,
        .de_io    = CAM_PIN_HSYNC,
        .pclk_io  = CAM_PIN_PCLK,
        .xclk_io  = CAM_PIN_XCLK,     /* -1：模块自带 24MHz 有源晶振 */
    },
    .xclk_freq = CAM_XCLK_HZ,         /* 0：ESP32 不输出 XCLK */
};

static const esp_video_init_config_t s_video_cfg = {
    .dvp = &s_dvp_cfg,
};

static int      s_fd = -1;
static uint8_t *s_buffer[BUFFER_COUNT];
static uint32_t s_buffer_len[BUFFER_COUNT];

/* 传感器实际输出尺寸（OV2640 可能是 240x240，OV5640 是 800x600） */
static uint32_t s_cap_w;
static uint32_t s_cap_h;
/* 需要缩放时用的 240x240 RGB565 中转缓冲（PSRAM） */
static uint8_t *s_scaled;

/* UART 链路 */
#define LINK_UART_NUM     UART_NUM_1
static uint8_t s_tx_row[LINK_ROW_BYTES];   /* 480 字节，单行 BE 格式 */

/* 把 s_cap_w x s_cap_h 的 RGB565 帧居中裁剪成正方形，最近邻缩放到 240x240 */
static void scale_frame_to_buf(const uint8_t *src)
{
    uint32_t side = (s_cap_w < s_cap_h) ? s_cap_w : s_cap_h;
    uint32_t ox   = (s_cap_w - side) / 2;
    uint32_t oy   = (s_cap_h - side) / 2;
    uint16_t *dst = (uint16_t *)s_scaled;

    for (uint32_t y = 0; y < LINK_PIX_H; y++) {
        uint32_t sy = oy + (y * side) / LINK_PIX_H;
        const uint16_t *row = (const uint16_t *)(src + (size_t)sy * s_cap_w * 2);
        for (uint32_t x = 0; x < LINK_PIX_W; x++) {
            uint32_t sx = ox + (x * side) / LINK_PIX_W;
            dst[y * LINK_PIX_W + x] = row[sx];
        }
    }
}

#if CAM_KEEP_LOCAL_PREVIEW
static void draw_scaled_rgb565(const uint8_t *src)
{
    scale_frame_to_buf(src);
    ili9341_draw_rgb565(CAM_SCREEN_X, CAM_SCREEN_Y,
                        CAM_FRAME_W, CAM_FRAME_H,
                        s_scaled, CAM_SWAP_RGB565);
}
#endif

/* SCCB 总线卡死时（SDA 被从机拉低）用 9 个时钟把它冲出来，
 * 然后把引脚还给 esp_video 的 I2C 驱动。 */
static void sccb_bus_recover(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << CAM_PIN_SCL) | (1ULL << CAM_PIN_SDA),
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(CAM_PIN_SDA, 1);
    gpio_set_level(CAM_PIN_SCL, 1);
    esp_rom_delay_us(20);
    for (int i = 0; i < 9; i++) {
        gpio_set_level(CAM_PIN_SCL, 0);
        esp_rom_delay_us(20);
        gpio_set_level(CAM_PIN_SCL, 1);
        esp_rom_delay_us(20);
    }
    gpio_reset_pin(CAM_PIN_SCL);
    gpio_reset_pin(CAM_PIN_SDA);
}

esp_err_t camera_preview_init(void)
{
    /* 摄像头模块上电到 SCCB 可访问需要一点时间 */
    vTaskDelay(pdMS_TO_TICKS(500));
    sccb_bus_recover();

    /* 上电/复位瞬间 SCCB 偶尔会失败，重试几次比直接放弃稳 */
    esp_err_t err = ESP_FAIL;
    for (int attempt = 1; attempt <= 3; attempt++) {
        err = esp_video_init(&s_video_cfg);
        if (err == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "esp_video_init failed (%s), retry %d/3",
                 esp_err_to_name(err), attempt);
        esp_video_deinit();
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    ESP_RETURN_ON_FALSE(err == ESP_OK, err, TAG,
                        "esp_video_init failed after 3 attempts");

    /* esp_video_init 返回 OK 后，/dev/video2 节点可能还没注册完，
     * 偶发 open 失败；原先只重试初始化不重试 open，这里补上。 */
    for (int attempt = 1; attempt <= 5; attempt++) {
        s_fd = open(ESP_VIDEO_DVP_DEVICE_NAME, O_RDONLY);
        if (s_fd >= 0) {
            break;
        }
        ESP_LOGW(TAG, "open %s failed, retry %d/5", ESP_VIDEO_DVP_DEVICE_NAME, attempt);
        /* 设备节点没起来说明 sensor 探测那一步没成功，必须整套重新初始化，
         * 只重试 open 是没用的。 */
        esp_video_deinit();
        vTaskDelay(pdMS_TO_TICKS(300));
        err = esp_video_init(&s_video_cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "re-init failed: %s", esp_err_to_name(err));
        }
    }
    ESP_RETURN_ON_FALSE(s_fd >= 0, ESP_FAIL, TAG,
                        "open %s failed after 5 attempts", ESP_VIDEO_DVP_DEVICE_NAME);

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = CAM_FRAME_W;
    fmt.fmt.pix.height      = CAM_FRAME_H;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;

    if (ioctl(s_fd, VIDIOC_S_FMT, &fmt) != 0) {
        /* OV5640 驱动只有 800x600 RGB565，退回它支持的格式再缩放 */
        ESP_LOGW(TAG, "240x240 RGB565 not accepted, retry 800x600 (OV5640)");
        memset(&fmt, 0, sizeof(fmt));
        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = 800;
        fmt.fmt.pix.height      = 600;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
        ESP_RETURN_ON_FALSE(ioctl(s_fd, VIDIOC_S_FMT, &fmt) == 0, ESP_FAIL, TAG,
                            "VIDIOC_S_FMT failed (240x240 and 800x600 RGB565 both rejected)");
    }

    s_cap_w = fmt.fmt.pix.width;
    s_cap_h = fmt.fmt.pix.height;
    ESP_LOGI(TAG, "format: %lux%lu RGB565, %lu bytes/frame",
             (unsigned long)s_cap_w, (unsigned long)s_cap_h,
             (unsigned long)fmt.fmt.pix.sizeimage);

    if (s_cap_w != LINK_PIX_W || s_cap_h != LINK_PIX_H) {
        s_scaled = heap_caps_malloc((size_t)LINK_PIX_W * LINK_PIX_H * 2,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        ESP_RETURN_ON_FALSE(s_scaled != NULL, ESP_ERR_NO_MEM, TAG,
                            "no PSRAM for %ux%lu scale buffer",
                            LINK_PIX_W, (unsigned long)LINK_PIX_H);
        ESP_LOGI(TAG, "will crop %lux%lu center square and scale to %ux%lu",
                 (unsigned long)s_cap_w, (unsigned long)s_cap_h,
                 LINK_PIX_W, (unsigned long)LINK_PIX_H);
    }

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count  = BUFFER_COUNT;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_RETURN_ON_FALSE(ioctl(s_fd, VIDIOC_REQBUFS, &req) == 0, ESP_FAIL, TAG,
                        "VIDIOC_REQBUFS failed");

    for (int i = 0; i < BUFFER_COUNT; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        ESP_RETURN_ON_FALSE(ioctl(s_fd, VIDIOC_QUERYBUF, &buf) == 0, ESP_FAIL, TAG,
                            "VIDIOC_QUERYBUF[%d] failed", i);

        s_buffer[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                      MAP_SHARED, s_fd, buf.m.offset);
        ESP_RETURN_ON_FALSE(s_buffer[i] != MAP_FAILED, ESP_ERR_NO_MEM, TAG,
                            "mmap[%d] failed, len=%u", i, buf.length);
        s_buffer_len[i] = buf.length;

        ESP_RETURN_ON_FALSE(ioctl(s_fd, VIDIOC_QBUF, &buf) == 0, ESP_FAIL, TAG,
                            "VIDIOC_QBUF[%d] failed", i);
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_RETURN_ON_FALSE(ioctl(s_fd, VIDIOC_STREAMON, &type) == 0, ESP_FAIL, TAG,
                        "VIDIOC_STREAMON failed");

    ESP_LOGI(TAG, "stream started, %d buffers", BUFFER_COUNT);
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * UART 链路初始化：复用原来接 TFT 的 43 脚做 UART1 TX，
 * 42 原 TFT 复位脚改成 READY 输入。44/45 悬空不用。
 * -------------------------------------------------------------------------- */
static esp_err_t link_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LINK_PIN_READY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,   /* STM32 没上电时不要误判为就绪 */
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "READY gpio config failed");

    uart_config_t uart_cfg = {
        .baud_rate = LINK_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(LINK_UART_NUM, 256, 0, 0, NULL, 0),
                        TAG, "uart_driver_install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(LINK_UART_NUM, &uart_cfg),
                        TAG, "uart_param_config failed");
    /* TX=GPIO43；RX/CTS/RTS 不用，设成 UART_PIN_NO_CHANGE */
    ESP_RETURN_ON_ERROR(uart_set_pin(LINK_UART_NUM, LINK_PIN_TX,
                                      UART_PIN_NO_CHANGE,
                                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin failed");

    ESP_LOGI(TAG, "link ready: TX=%d READY=%d @%lu baud 8N1",
             LINK_PIN_TX, LINK_PIN_READY, (unsigned long)LINK_UART_BAUD);
    return ESP_OK;
}

/* 等 READY 变成指定电平，超时返回 false */
static bool wait_ready(int level, int timeout_ms)
{
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(LINK_PIN_READY) != level) {
        if ((esp_timer_get_time() - start) / 1000 > timeout_ms) {
            return false;
        }
        /* 不能用 vTaskDelay(1)：本工程 tick=100Hz，一次就是 10ms，
         * 逐行握手会多出 240*10ms 的延迟。taskYIELD 只让出 CPU，不引入 tick 延迟。 */
        taskYIELD();
    }
    return true;
}

/* 发一行：485 字节，双 magic + 帧号 + 行号 + 异或校验 + 480 字节像素 */
static esp_err_t link_send_row(const uint8_t *rgb565_le,
                               uint8_t frame_id, uint8_t row)
{
    uint8_t pkt[LINK_PKT_BYTES];
    uint8_t xor = LINK_MAGIC1 ^ LINK_MAGIC2 ^ frame_id ^ row;
    int written;

    if (!wait_ready(1, 5000)) {
        ESP_LOGW(TAG, "timeout waiting READY=1 at row %u (check PB10->GPIO42)", row);
        return ESP_ERR_TIMEOUT;
    }
    if (row <= 1) {
        ESP_LOGI(TAG, "row %u: READY=1, sending %d bytes via UART1", row, LINK_PKT_BYTES);
    }

    for (uint32_t i = 0; i < LINK_PIX_W; i++) {
#if CAM_SWAP_RGB565
        /* V4L2 是低字节在前，ILI9341 要高字节在前 */
        uint16_t px = ((const uint16_t *)rgb565_le)[i];
        s_tx_row[i * 2]     = (uint8_t)(px >> 8);
        s_tx_row[i * 2 + 1] = (uint8_t)(px & 0xFF);
#else
        s_tx_row[i * 2]     = rgb565_le[i * 2];
        s_tx_row[i * 2 + 1] = rgb565_le[i * 2 + 1];
#endif
        xor ^= s_tx_row[i * 2];
        xor ^= s_tx_row[i * 2 + 1];
    }

    pkt[0] = LINK_MAGIC1;
    pkt[1] = LINK_MAGIC2;
    pkt[2] = frame_id;
    pkt[3] = row;
    pkt[4] = xor;
    memcpy(pkt + LINK_HDR_BYTES, s_tx_row, LINK_ROW_BYTES);

    written = uart_write_bytes(LINK_UART_NUM, (const char *)pkt, LINK_PKT_BYTES);
    if (written != LINK_PKT_BYTES) {
        ESP_LOGE(TAG, "uart_write_bytes=%d, expected %d", written, LINK_PKT_BYTES);
        return ESP_FAIL;
    }
    /* 必须等硬件把 485 字节全部发出去，否则下一包会覆盖 ring buffer */
    esp_err_t err = uart_wait_tx_done(LINK_UART_NUM, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_wait_tx_done failed: %s", esp_err_to_name(err));
        return err;
    }
    if (row == 0) {
        ESP_LOGI(TAG, "row 0: UART transmit done");
    }

    /* STM32 收到首字节后立刻拉低 READY；等它拉低才算这一包被接住。
     * 超时只告警不报错：下一包会再等 READY=1，不会丢同步。 */
    if (!wait_ready(0, 100)) {
        ESP_LOGW(TAG, "READY not pulled low at row %u (check PB10->GPIO42)", row);
    }
    return ESP_OK;
}

esp_err_t camera_link_run(void)
{
    uint32_t frames = 0;
    uint8_t frame_id = 0;
    int64_t t0 = esp_timer_get_time();
    static bool logged_first_buf = false;

    ESP_RETURN_ON_ERROR(link_init(), TAG, "link init failed");

    while (1) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        ESP_RETURN_ON_FALSE(ioctl(s_fd, VIDIOC_DQBUF, &buf) == 0, ESP_FAIL, TAG,
                            "VIDIOC_DQBUF failed");

        if (!logged_first_buf) {
            logged_first_buf = true;
            ESP_LOGI(TAG, "first DQBUF: index=%u flags=0x%x", buf.index, buf.flags);
        }

        if ((buf.flags & V4L2_BUF_FLAG_DONE) && buf.index < BUFFER_COUNT) {
            const uint8_t *frame = (const uint8_t *)s_buffer[buf.index];
            if (s_scaled != NULL) {
                scale_frame_to_buf(frame);
                frame = s_scaled;
            }

            frame_id++;
            esp_err_t err = ESP_OK;
            uint16_t failed_row = 0;
            for (uint16_t row = 0; row < LINK_PIX_H; row++) {
                err = link_send_row(frame + (uint32_t)row * LINK_ROW_BYTES,
                                    frame_id, (uint8_t)row);
                if (err != ESP_OK) {
                    failed_row = row;
                    ESP_LOGE(TAG, "row %u send failed: %s", row, esp_err_to_name(err));
                    break;
                }
            }

            if (err == ESP_OK) {
                frames++;
                if (frames == 1) {
                    ESP_LOGI(TAG, "first frame sent OK (240 rows x 485B)");
                }
                if (frames % 5 == 0) {
                    int64_t ms = (esp_timer_get_time() - t0) / 1000;
                    ESP_LOGI(TAG, "%lu frames in %lld ms (~%.1f fps)",
                             (unsigned long)frames, (long long)ms,
                             ms > 0 ? (double)frames * 1000.0 / (double)ms : 0.0);
                }
            } else {
                /* 链路暂时不通（例如 STM32 比 ESP32 晚上电）时继续重试，
                 * 不能一次失败就退出，否则 STM32 起来后 ESP32 也不会再发。 */
                ESP_LOGW(TAG, "frame aborted at row %u, retry next frame", failed_row);
            }
        }

        ESP_RETURN_ON_FALSE(ioctl(s_fd, VIDIOC_QBUF, &buf) == 0, ESP_FAIL, TAG,
                            "VIDIOC_QBUF failed");
    }
}

#if CAM_KEEP_LOCAL_PREVIEW
esp_err_t camera_preview_run(void)
{
    uint32_t frames = 0;
    int64_t  t0 = esp_timer_get_time();

    ili9341_fill_screen(RGB(255, 255, 255));

    while (1) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        ESP_RETURN_ON_FALSE(ioctl(s_fd, VIDIOC_DQBUF, &buf) == 0, ESP_FAIL, TAG,
                            "VIDIOC_DQBUF failed");

        if ((buf.flags & V4L2_BUF_FLAG_DONE) && buf.index < BUFFER_COUNT) {
            if (s_scaled) {
                draw_scaled_rgb565((const uint8_t *)s_buffer[buf.index]);
            } else {
                ili9341_draw_rgb565(CAM_SCREEN_X, CAM_SCREEN_Y,
                                    CAM_FRAME_W, CAM_FRAME_H,
                                    (const uint8_t *)s_buffer[buf.index],
                                    CAM_SWAP_RGB565);
            }

            frames++;
            if (frames % 30 == 0) {
                int64_t ms = (esp_timer_get_time() - t0) / 1000;
                ESP_LOGI(TAG, "%lu frames in %lld ms (~%.1f fps)",
                         (unsigned long)frames, (long long)ms,
                         ms > 0 ? (double)frames * 1000.0 / (double)ms : 0.0);
            }
        }

        ESP_RETURN_ON_FALSE(ioctl(s_fd, VIDIOC_QBUF, &buf) == 0, ESP_FAIL, TAG,
                            "VIDIOC_QBUF failed");
    }
}
#endif
