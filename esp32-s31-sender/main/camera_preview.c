/* ============================================================================
 * DVP 摄像头（OV5640 / OV2640 自动识别）-> SPI -> STM32F103CBT6 -> ILI9341
 *
 * 模块自带 24MHz 有源晶振，排针没有 XCLK 脚，所以 ESP32 不输出 XCLK。
 * 底层用乐鑫官方 esp_video / esp_cam_sensor 组件（V4L2 风格接口），
 * 不自己写寄存器序列、不自己拼 DVP 时序。摄像头引脚在 camera_preview.h，
 * 与原来的工程完全一致，一根没动。
 *
 * OV2640 请求 240x240 RGB565；OV5640 只给 800x600，就在 PSRAM 里
 * 居中裁剪 + 最近邻缩放到 240x240，再逐行通过 SPI 送给 STM32。
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
#include "driver/spi_master.h"
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

/* SPI 链路：一次事务发一整包（485 字节），CS 由驱动自动包住整包 */
#define LINK_SPI_HOST     SPI2_HOST
static spi_device_handle_t s_spi;
static uint8_t s_tx_row[LINK_ROW_BYTES];   /* 480 字节，单行 BE 格式 */
static uint8_t s_pkt[LINK_PKT_BYTES];      /* 485 字节整包：头 + 行像素 */

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
 * SPI 链路初始化：45=SCK, 44=MOSI, 43=CS（主机输出），42=READY 输入。
 * 这四根就是原来 ESP32 直接刷 ILI9341 用的线，接线不用动。
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

    spi_bus_config_t bus = {
        .mosi_io_num     = LINK_PIN_MOSI,
        .miso_io_num     = -1,                  /* 单向：STM32 不回数据 */
        .sclk_io_num     = LINK_PIN_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = LINK_PKT_BYTES,
    };
    /* 用 DMA 通道：485 字节超过 FIFO，交给 DMA 发才能把 CPU 让出来 */
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LINK_SPI_HOST, &bus, SPI_DMA_CH_AUTO),
                        TAG, "spi_bus_initialize failed");

    spi_device_interface_config_t dev = {
        .clock_speed_hz = LINK_SPI_HZ,
        .mode           = 0,                    /* CPOL=0, CPHA=0，与 STM32 从机一致 */
        .spics_io_num   = LINK_PIN_CS,
        .queue_size     = 1,
        /* CS 提前 4 个位周期拉低：STM32F1 的 SPI 从机要过约 1 个 APB 时钟
         * 才认定自己被选中，NSS 和第一个时钟沿贴太近会丢最高位。
         * 发完再多保持 1 个位周期，避免最后一位还没采样 CS 就抬起来。 */
        .cs_ena_pretrans  = 4,
        .cs_ena_posttrans = 1,
        .flags          = SPI_DEVICE_NO_DUMMY,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(LINK_SPI_HOST, &dev, &s_spi),
                        TAG, "spi_bus_add_device failed");

    ESP_LOGI(TAG, "link ready: SCK=%d MOSI=%d CS=%d READY=%d @%lu Hz mode0",
             LINK_PIN_SCK, LINK_PIN_MOSI, LINK_PIN_CS, LINK_PIN_READY,
             (unsigned long)LINK_SPI_HZ);
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

/* 发一行：485 字节，双 magic + 帧号 + 行号 + 异或校验 + 480 字节像素。
 * 一次 spi_device_polling_transmit 发完整包，CS 由驱动在事务前后自动拉低/拉高。 */
static esp_err_t link_send_row(const uint8_t *rgb565_le,
                               uint8_t frame_id, uint8_t row)
{
    uint8_t xor = LINK_MAGIC1 ^ LINK_MAGIC2 ^ frame_id ^ row;
    spi_transaction_t t;

    /* 先把整包拼好，再去等 READY：这段约 50us 的组装时间能和
     * STM32 写上一行屏的时间重叠，不必串行浪费。 */
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

    s_pkt[0] = LINK_MAGIC1;
    s_pkt[1] = LINK_MAGIC2;
    s_pkt[2] = frame_id;
    s_pkt[3] = row;
    s_pkt[4] = xor;
    memcpy(s_pkt + LINK_HDR_BYTES, s_tx_row, LINK_ROW_BYTES);

    if (!wait_ready(1, 5000)) {
        ESP_LOGW(TAG, "timeout waiting READY=1 at row %u (check PB10->GPIO42)", row);
        return ESP_ERR_TIMEOUT;
    }
    if (row <= 1) {
        ESP_LOGI(TAG, "row %u: READY=1, sending %d bytes via SPI", row, LINK_PKT_BYTES);
    }

    memset(&t, 0, sizeof(t));
    t.length    = (size_t)LINK_PKT_BYTES * 8;    /* 以 bit 计 */
    t.tx_buffer = s_pkt;

    esp_err_t err = spi_device_polling_transmit(s_spi, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi transmit failed at row %u: %s", row, esp_err_to_name(err));
        return err;
    }
    if (row == 0) {
        ESP_LOGI(TAG, "row 0: SPI transmit done");
    }

    /* STM32 的 DMA 收满立刻拉低 READY；等它拉低才算这一包真的被接住。
     * 拉不低说明 STM32 没在收（没烧 V7、SPI 线没接、或 DMA 没武装），
     * 这时中止本帧，比"继续发 240 行然后误报成功"有利于排错。 */
    if (!wait_ready(0, 100)) {
        ESP_LOGW(TAG, "READY not pulled low at row %u "
                      "(is STM32 flashed with V7? check PB10->GPIO42)", row);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t camera_link_selftest(void)
{
    /* 摄像头起不来时的备胎：把 SPI 链路单独跑起来，发彩条测试图案。
     * 目的是把"摄像头接触不良"和"两板连线不对"两个问题拆开排查——
     * 只要 STM32 的 K 开始涨、屏上出现彩条，就证明 SPI 那 4 根线是通的。 */
    static const uint16_t bar[6] = {
        0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF,  /* 红 绿 蓝 黄 紫 青 */
    };
    uint8_t  row_le[LINK_ROW_BYTES];
    uint32_t frames = 0;
    int64_t  t0;

    ESP_LOGW(TAG, "camera unavailable -> SPI LINK SELF-TEST (colour bars)");
    ESP_LOGW(TAG, "screen showing bars + K rising on STM32 means the SPI wiring is OK");

    ESP_RETURN_ON_ERROR(link_init(), TAG, "link init failed");
    t0 = esp_timer_get_time();

    while (1) {
        uint32_t bar_x = (frames * 4u) % LINK_PIX_W;    /* 白条每帧右移，能看出刷新 */
        esp_err_t err = ESP_OK;

        for (uint16_t row = 0; row < LINK_PIX_H; row++) {
            for (uint32_t i = 0; i < LINK_PIX_W; i++) {
                uint32_t seg = (i * 6u) / LINK_PIX_W;
                uint32_t d   = (i > bar_x) ? (i - bar_x) : (bar_x - i);
                uint16_t px;

                if (d < 6) {
                    px = 0xFFFF;                          /* 移动白条 */
                } else if (row < 6 || row >= (LINK_PIX_H - 6)) {
                    px = 0x0000;                          /* 上下黑边：行号错位一眼可见 */
                } else {
                    px = bar[seg];
                }
                row_le[i * 2]     = (uint8_t)(px & 0xFF); /* 小端，与摄像头输出一致 */
                row_le[i * 2 + 1] = (uint8_t)(px >> 8);
            }
            err = link_send_row(row_le, (uint8_t)frames, (uint8_t)row);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "selftest row %u failed: %s", row, esp_err_to_name(err));
                break;
            }
        }

        if (err == ESP_OK) {
            frames++;
            if (frames == 1) {
                ESP_LOGI(TAG, "SELFTEST first frame sent OK - SPI link is alive");
            }
            if (frames % 5 == 0) {
                int64_t ms = (esp_timer_get_time() - t0) / 1000;
                ESP_LOGI(TAG, "SELFTEST %lu frames in %lld ms (~%.1f fps)",
                         (unsigned long)frames, (long long)ms,
                         ms > 0 ? (double)frames * 1000.0 / (double)ms : 0.0);
            }
        } else {
            ESP_LOGW(TAG, "SELFTEST frame aborted at row - check SCK/MOSI/CS/READY + GND");
        }
    }
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
                    ESP_LOGI(TAG, "first frame sent OK (%u rows x %dB over SPI)",
                             (unsigned)LINK_PIX_H, LINK_PKT_BYTES);
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
