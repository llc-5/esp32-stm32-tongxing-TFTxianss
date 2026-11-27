# ESP32-S31 摄像头 → STM32F103 摄像头转屏系统

ESP32-S31 驱动 DVP 摄像头（OV2640 / OV5640 自动识别）采集画面，通过 **UART** 逐行传给 STM32F103CBT6，STM32 把画面写到 ILI9341（240×320 RGB565）TFT 屏上。

> 当前版本：**UART 传输版**（双机通讯已从 SPI 改为 UART）。
> 摄像头端固件已实测能识别 `OV2640 PID=0x26` 并输出 240×240 RGB565。
> 若摄像头 SCCB 读不到 ID，先重插摄像头 2×9 排线再复位 ESP32。

## 目录结构

```
esp32-stm32/
├── esp32-s31-sender/       # ESP32 发送端工程（ESP-IDF 6.2）
│   ├── main/               # 摄像头采集 + UART 发送（camera_preview.c / link_proto.h）
│   ├── CMakeLists.txt / sdkconfig / partitions.csv / dependencies.lock
│   ├── flash.ps1 / flash.bat / flash_chunked.py / read_serial.py   # 编译烧录脚本
│   └── .vscode / .devcontainer                                     # 开发环境配置
├── stm32f103-receiver/     # STM32 接收端工程（Keil MDK-ARM）
│   ├── User/main.c         # USART1 RX 接收 + ILI9341 写屏（纯寄存器初始化）
│   ├── User/link_proto.h   # 协议常量（与 ESP32 端逐字一致）
│   ├── User/ili9341.c/.h   # 屏驱动 + LCD_DrawRGB565Row()
│   ├── Library / Start     # STM32 标准库与启动文件
│   └── Project.uvprojx     # Keil 工程文件
└── docs/                   # 引脚接线图（md/html）、使用说明
```

## 接线概要

摄像头 16 根 DVP 线接 ESP32（位置不变）：SCL=GPIO3、SDA=GPIO4、D0~D7=35/37/38/39/0/1/2/47、PCLK=46、VSYNC=48、HREF=49；模块自带 24MHz 晶振，不接 XCLK。

**ESP32 → STM32（UART）：**

| 信号 | ESP32-S31 | STM32F103 |
|---|---|---|
| UART1 TX | GPIO43 | PA10（USART1 RX） |
| READY | GPIO42 | PB10 |
| GND | GND | GND |

**ILI9341 → STM32：** RES=PA0、CS=PA1、DC=PA2、SCK=PA5、SDI=PA7、LED=PA4，3.3V 供电。

详细接线与排错见 [`docs/引脚接线图.md`](docs/引脚接线图.md)、[`docs/使用说明.md`](docs/使用说明.md)。

## 传输协议

每行一包 485 字节：`A5 5A | 帧号 | 行号(0~239) | 异或校验 | 480 字节 RGB565(高字节在前)`，波特率 2,000,000 8N1。
STM32 收到一行立即写屏，不缓存整帧（STM32F103CBT6 仅 20KB SRAM）。屏幕顶栏三页调试计数：`K` 有效行、`T` 收包超时、`C` 校验错、`G` magic 错、`W` 行号错、`L` 主循环心跳；版本号 `V6` 表示 UART 版。

## 编译与烧录

**ESP32 发送端**（需 ESP-IDF 6.2 环境，COM10 为例）：

```powershell
cd esp32-s31-sender
.\flash.ps1 -Port COM10      # 编译 + 烧录 + 串口验证
```

**STM32 接收端：** Keil 打开 `stm32f103-receiver/Project.uvprojx` → Rebuild → 烧录。
Include Paths 需包含 `.\User`。USART1 为纯寄存器初始化，不需要在工程里加 `stm32f10x_usart.c`。

## 注意

- 两个工程目录未包含在 GitHub 仓库中的构建产物（`build/`、`Objects/`、`Listings/`、`managed_components/` 等），首次使用需重新编译生成。
- 两块板 3.3V 各自供电，**只共地，不要并接 3.3V**。
- 若 `C`/`G` 计数增长（串口丢字节），两端同时把 `LINK_UART_BAUD` 降到 `1000000UL` 后重新编译烧录。
