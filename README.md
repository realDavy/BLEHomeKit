# ESP32-C3-LCDkit HomeKit BLE 调光灯

在 [ESP32-C3-LCDkit](https://docs.espressif.com/projects/esp-dev-kits/zh_CN/latest/esp32c3/esp32-c3-lcdkit/index.html) 上，用 [realDavy/HAP](https://github.com/realDavy/HAP) 通过 **蓝牙 LE（HAP-BLE）** 接入 Apple HomeKit，并实现官方 [knob_panel](https://github.com/espressif/esp-dev-kits/tree/01b2ac80b0e7517502f49495df506dd34d1bc8af/examples/esp32-c3-lcdkit/examples/knob_panel) 示例中的 LED 调光逻辑。

本配件 **不走 Wi-Fi / HAP over IP**，iPhone 通过蓝牙发现并配对。

## 硬件

开发板由主板 `ESP32-C3-LCDkit_MB`（ESP32-C3-MINI-1，4 MB Flash）和 1.28 英寸 GC9A01 子板组成。本工程用到的外设：

| 功能 | GPIO | 说明 |
|------|------|------|
| RGB LED (WS2812) | IO8 | 调光输出，与 knob_panel 的 `bsp_led_rgb_set` 一致 |
| 旋转编码器 A | IO10 | 顺时针提高亮度 |
| 旋转编码器 B | IO6 | 逆时针降低亮度 |
| 编码器按键 | IO9 | 短按切换暖白/冷白；长按 3 秒清除 HomeKit 配对 |
| LCD 背光 | IO5 | 亮度 UI |
| LCD SPI | IO0/1/2/7 | SDA / SCL / D/C / CS |

### 与 knob_panel LED 调光的对应关系

官方 `ui_light_2color.c`：

- 旋钮左右：亮度按 **25%** 步进（0 / 25 / 50 / 75 / 100）
- 短按：暖白 ↔ 冷白
- 冷白：`R=G=B = 0xFF * pwm / 100`
- 暖白：`R=G = 0xFF * pwm / 100`，`B = 0x33 * pwm / 100`

本工程把同一套混色接到板载 WS2812，并映射为 HomeKit `Lightbulb`：

| HomeKit 特征 | 本地行为 |
|--------------|----------|
| On | 开关；亮度为 0 时视为关闭 |
| Brightness (0–100) | PWM 占空比；旋钮仍按 25% 步进 |
| ColorTemperature (140–500 mired) | 冷白约 154 mired，暖白约 370 mired |

Home App 中的开关、亮度条、色温会驱动 LED；旋钮操作会反向通知已配对的 iPhone。

## 软件要求

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/index.html) **v5.3 或更高**（与 knob_panel 一致）
- Python 3 + ESP-IDF 组件管理器（构建时会拉取 BSP / led_strip / knob）

## 构建与烧录

`idf.py` **必须在本仓库根目录执行**（该目录下有 `CMakeLists.txt`）。clone 之后先 `cd` 进去，不要在 `esp-idf/examples` 或其他上级目录运行。

```bash
# 工程目前在 PR 分支上，不要只 clone 默认的 main（main 里还没有 CMakeLists.txt）
git clone -b cursor/fix-flicker-ble-discovery-7c32 --recurse-submodules \
    https://github.com/realDavy/BLEHomeKit.git
cd BLEHomeKit
ls CMakeLists.txt   # 确认当前就在工程根目录

. $IDF_PATH/export.sh
# 若之前编过旧固件，必须删掉 sdkconfig，否则 BSP 双缓冲 96KB 的默认值会留下来
rm -f sdkconfig
idf.py set-target esp32c3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

如果已经 clone 了默认 `main`，在仓库里切换分支即可：

```bash
cd BLEHomeKit
git fetch origin
git checkout cursor/fix-flicker-ble-discovery-7c32
git submodule update --init --recursive
rm -f sdkconfig
ls CMakeLists.txt
idf.py set-target esp32c3
```

若出现 `CMakeLists.txt not found in project directory .../examples`，说明还在上级目录，执行 `cd BLEHomeKit` 后再跑 `idf.py`。

### 屏闪 / 家庭 App 搜不到蓝牙配件

旧固件在 `[BleTransport] Starting...` 之后会 `abort()` 并重启。原因是 ESP32-C3-LCDkit BSP 默认给 LVGL 分配 **双缓冲 240×100（约 96 KB）**，剩下的堆不够 HAP 注册 GATT，`new` 失败即 `abort()`。屏幕跟着重启循环闪，广播也起不来。

本分支改为：

- 单缓冲 20 行（约 9.6 KB）
- 先启动 HAP-BLE，再启动 LVGL
- 补上 NimBLE GAP/GATT 服务和 `sodium_init`

重新编译前请删除工程目录里的 `sdkconfig`（见上方命令），否则旧的缓冲配置会继续生效。启动成功时应能看到 `NimBLE Synced`、`Advertising started`，以及 `heap after hap start` 仍有空闲内存。

若已经能发现配件但配对在 Pair Setup M2 后断开：旧代码会一次 GATT Read 返回约 418 字节，超过 ATT MTU 256。新固件应打印 `Returning 253/418 bytes` 这类分片。配对码以屏幕和串口打印的 `XXX-XX-XXX` 为准。

USB 口打不开时，按官方下载模式：

1. 按住旋钮
2. 短按 `REST`
3. 松开旋钮后再烧录

## HomeKit 配对

配对码按设备出厂 MAC 生成（格式 `XXX-XX-XXX`），同一块板每次开机都一样，不同板不同。屏幕和串口日志都会打印该码；Apple 禁止的弱码（如 `111-11-111`、`123-45-678`）会被自动跳过。

1. 固件启动后，屏幕显示 `HomeKit BLE` 和 `PIN xxx-xx-xxx`
2. iPhone 打开「家庭」→ 添加配件 →「更多选项…」
3. 选择 **LCDkit Light**（蓝牙配件）
4. 输入屏幕上的配对码

Identify 时板载 RGB LED 会闪三次。长按旋钮 3 秒会执行 HomeKit 恢复出厂（清除配对），可重新添加。

## 工程结构

```
├── main/                      # 应用：HAP 配件 + 调光 + 旋钮 + UI
├── components/hap_wrapper/    # 封装 realDavy/HAP（git submodule）
├── components/yahap-pal/      # ESP-IDF 平台层（NimBLE / NVS / 加密）
├── partitions.csv
└── sdkconfig.defaults         # ESP32-C3 + NimBLE + 4MB Flash
```

HAP 协议栈来自 [realDavy/HAP](https://github.com/realDavy/HAP) 的 ESP32 BLE 灯泡示例，平台抽象使用 [esp32-yahap-pal](https://github.com/rednblkx/esp32-yahap-pal)。

## 操作说明

| 操作 | 效果 |
|------|------|
| 顺时针旋转 | 亮度 +25% |
| 逆时针旋转 | 亮度 -25% |
| 短按旋钮 | 暖白 / 冷白 |
| 长按 3 秒 | 清除 HomeKit 配对 |
| Home App 开关 / 亮度 / 色温 | 同步到 WS2812 和屏幕 |
