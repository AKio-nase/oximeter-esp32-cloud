# oximeter-esp32-cloud

血氧 / 心率的端到云监控系统。ESP32 采集，MQTT over TLS 上云，浏览器实时看数。

> 本仓库仅用于个人作品展示；未经许可不得用于商业用途。仓库内不含真实 WiFi / MQTT 凭据，均为占位符与公共 CA。

2026 年 3 月到 6 月的课程项目，硬件选型、固件、云端接入、网页都是一个人做的。

## 目录

```
firmware/oximeter/   ESP32 固件（Arduino）
web/index.html       监控页，单文件，原生 JS
docs/screenshots/    接线图 / OLED 实拍 / 网页截图
```

## 怎么跑

1. 装 Arduino IDE 和 ESP32 开发板支持，库需要 PubSubClient、U8g2、SparkFun MAX3010x。
2. 打开 `firmware/oximeter/oximeter.ino`，把顶部这几个常量换成自己的：

```cpp
const char* ssid         = "YOUR_WIFI_SSID";
const char* password     = "YOUR_WIFI_PASSWORD";
const char* mqttServer   = "YOUR_INSTANCE.ala.cn-hangzhou.emqxsl.cn";
const char* mqttUser     = "YOUR_MQTT_USER";
const char* mqttPassword = "YOUR_MQTT_PASSWORD";
```

3. 烧录，串口能看到连 WiFi 和连 broker 的日志。
4. 打开 `web/index.html`，填同一个 host、端口 8084、用户名密码，点连接。

## 数据链路

- 设备端：ESP32 → MQTTS 8883（TLS）→ EMQX Cloud
- 网页端：mqtt.js → WSS 8084 → 同一个实例
- 主题：`sensor/oxygen/status` 上行，`sensor/oxygen/control` 下行

上行 JSON：

```json
{"spo2": 96.7, "heartRate": 72, "isBlinking": 0}
```

## 几个实现上的细节

- MAX30102 走 I2C（SDA=21，SCL=22），200Hz 采红光和红外反射光强。
- 血氧用 25 点 IR/RED 比值法算完再平滑；心率用 PBA 脉搏算法加 8 点滑动平均。
- 光学测量容易出半频误检（真实 60 显示成 30），所以按 430–1200ms 的间隔窗口过滤，只留 50–140 BPM。
- ADC 是 18 位、满量程 262143，做了饱和保护；信号弱的时候自动调高 LED 驱动电流。
- 手指在位判断：IR 低于 25000 或 RED 低于 5000 连续 5 帧就算手指离开。
- 报警带滞回，连续 3 次低于 88% 才响，回到 92% 以上自动解除。
- 网页可以往 control 主题发指令，远程关掉设备端的报警。

## 已知问题

- 血氧绝对值只能看趋势。指尖反射式测量误差本来就大，没和指夹血氧仪做过对比标定。
- 手一动数值就飘，安静坐着的时候才稳。
- 云端用的是 EMQX Cloud 共享实例，连接参数全在代码里手填，没做配网页面。
- 网页是纯静态文件，双击就能开，也因此没有构建和自动化测试。

## 说明

仓库里没有真实凭据。`ca_cert.h` 里是 EMQX Cloud 的公共 CA 证书。
