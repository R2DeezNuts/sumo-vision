#include "Arduino.h"
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

#ifndef WIFI_SSID
#error "WIFI_SSID is not defined. Create .env from .env.example."
#endif

#ifndef WIFI_PASSWORD
#error "WIFI_PASSWORD is not defined. Create .env from .env.example."
#endif

#ifndef LAPTOP_IP
#error "LAPTOP_IP is not defined. Create .env from .env.example."
#endif

#ifndef LAPTOP_PORT
#define LAPTOP_PORT 5005
#endif

#ifndef CONTROL_PORT
#define CONTROL_PORT 5006
#endif

#ifndef STATIC_IP
#define STATIC_IP "192.168.1.145"
#endif

#ifndef GATEWAY_IP
#define GATEWAY_IP "192.168.1.1"
#endif

#ifndef WIFI_SUBNET_MASK
#define WIFI_SUBNET_MASK "255.255.255.0"
#endif

#ifndef DNS_IP
#define DNS_IP "192.168.1.1"
#endif

#ifndef CAMERA_FRAME_SIZE
#define CAMERA_FRAME_SIZE FRAMESIZE_QVGA
#endif

#ifndef CAMERA_JPEG_QUALITY
#define CAMERA_JPEG_QUALITY 12
#endif

#ifndef TARGET_FPS
#define TARGET_FPS 20
#endif

#ifndef UDP_PAYLOAD_SIZE
#define UDP_PAYLOAD_SIZE 1400
#endif

static_assert(UDP_PAYLOAD_SIZE <= 1400, "UDP_PAYLOAD_SIZE should stay <= 1400 to avoid IP fragmentation");

static constexpr uint16_t UDP_MAGIC = 0xCAFE;
static constexpr uint8_t UDP_VERSION = 1;
static constexpr size_t UDP_HEADER_SIZE = 20;
static constexpr TickType_t WIFI_RETRY_DELAY = pdMS_TO_TICKS(500);

WiFiUDP udp;
WiFiUDP controlUdp;
IPAddress laptopIp;
uint32_t frameId = 0;
uint8_t currentJpegQuality = CAMERA_JPEG_QUALITY;

static bool parseIpConfig(const char *text, IPAddress &ip) {
  return ip.fromString(text);
}

static void setupLedFlash() {
#if defined(LED_GPIO_NUM)
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);
#endif
}

static void writeU16(uint8_t *buf, size_t offset, uint16_t value) {
  buf[offset] = value & 0xFF;
  buf[offset + 1] = (value >> 8) & 0xFF;
}

static void writeU32(uint8_t *buf, size_t offset, uint32_t value) {
  buf[offset] = value & 0xFF;
  buf[offset + 1] = (value >> 8) & 0xFF;
  buf[offset + 2] = (value >> 16) & 0xFF;
  buf[offset + 3] = (value >> 24) & 0xFF;
}

static void fillUdpHeader(
  uint8_t *packet,
  uint32_t currentFrameId,
  uint16_t packetId,
  uint16_t packetCount,
  uint32_t frameSize,
  uint16_t payloadSize
) {
  writeU16(packet, 0, UDP_MAGIC);
  packet[2] = UDP_VERSION;
  packet[3] = 0;
  writeU32(packet, 4, currentFrameId);
  writeU16(packet, 8, packetId);
  writeU16(packet, 10, packetCount);
  writeU32(packet, 12, frameSize);
  writeU16(packet, 16, payloadSize);
  writeU16(packet, 18, 0);
}

static bool sendFrameUdp(const camera_fb_t *fb) {
  if (!fb || !fb->buf || fb->len == 0 || fb->format != PIXFORMAT_JPEG) {
    return false;
  }

  const uint16_t packetCount = (fb->len + UDP_PAYLOAD_SIZE - 1) / UDP_PAYLOAD_SIZE;
  if (packetCount == 0) {
    return false;
  }

  uint8_t packet[UDP_HEADER_SIZE + UDP_PAYLOAD_SIZE];
  const uint32_t currentFrameId = frameId++;

  for (uint16_t packetId = 0; packetId < packetCount; packetId++) {
    const size_t offset = static_cast<size_t>(packetId) * UDP_PAYLOAD_SIZE;
    const uint16_t payloadSize = min(static_cast<size_t>(UDP_PAYLOAD_SIZE), fb->len - offset);

    fillUdpHeader(packet, currentFrameId, packetId, packetCount, fb->len, payloadSize);
    memcpy(packet + UDP_HEADER_SIZE, fb->buf + offset, payloadSize);

    if (!udp.beginPacket(laptopIp, LAPTOP_PORT)) {
      return false;
    }
    const size_t written = udp.write(packet, UDP_HEADER_SIZE + payloadSize);
    if (!udp.endPacket() || written != UDP_HEADER_SIZE + payloadSize) {
      return false;
    }
  }

  return true;
}

static void sendControlResponse(const IPAddress &remoteIp, uint16_t remotePort, const String &message) {
  controlUdp.beginPacket(remoteIp, remotePort);
  controlUdp.write(reinterpret_cast<const uint8_t *>(message.c_str()), message.length());
  controlUdp.endPacket();
}

static bool parseQualityCommand(const String &command, int &quality) {
  String normalized = command;
  normalized.trim();
  normalized.toUpperCase();

  int separator = normalized.indexOf(' ');
  if (separator < 0) {
    separator = normalized.indexOf('=');
  }
  if (separator < 0) {
    return false;
  }

  String key = normalized.substring(0, separator);
  String valueText = normalized.substring(separator + 1);
  key.trim();
  valueText.trim();
  if (key != "QUALITY" && key != "JPEG_QUALITY" && key != "Q") {
    return false;
  }
  if (valueText.length() == 0) {
    return false;
  }

  for (size_t i = 0; i < valueText.length(); i++) {
    if (!isDigit(valueText.charAt(i))) {
      return false;
    }
  }
  quality = valueText.toInt();
  return true;
}

static void handleControlUdp() {
  const int packetSize = controlUdp.parsePacket();
  if (packetSize <= 0) {
    return;
  }

  char buffer[64];
  const int bytesRead = controlUdp.read(buffer, sizeof(buffer) - 1);
  if (bytesRead <= 0) {
    return;
  }
  buffer[bytesRead] = '\0';

  const IPAddress remoteIp = controlUdp.remoteIP();
  const uint16_t remotePort = controlUdp.remotePort();
  String command(buffer);
  int quality = 0;
  if (!parseQualityCommand(command, quality)) {
    sendControlResponse(remoteIp, remotePort, "ERR UNKNOWN_COMMAND");
    return;
  }
  if (quality < 4 || quality > 63) {
    sendControlResponse(remoteIp, remotePort, "ERR QUALITY_RANGE 4 63");
    return;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor) {
    sendControlResponse(remoteIp, remotePort, "ERR NO_SENSOR");
    return;
  }
  if (sensor->set_quality(sensor, quality) != 0) {
    sendControlResponse(remoteIp, remotePort, "ERR SET_QUALITY");
    return;
  }

  currentJpegQuality = static_cast<uint8_t>(quality);
  Serial.printf("Camera JPEG quality changed to %u by %s:%u\n", currentJpegQuality, remoteIp.toString().c_str(), remotePort);
  sendControlResponse(remoteIp, remotePort, String("OK QUALITY ") + String(currentJpegQuality));
}

static void udpStreamTask(void *parameter) {
  (void)parameter;

  const uint32_t frameIntervalMs = max<uint32_t>(1, 1000 / TARGET_FPS);
  uint32_t sentFrames = 0;
  uint32_t droppedFrames = 0;
  uint32_t lastReportMs = millis();

  while (true) {
    const uint32_t frameStartMs = millis();
    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
      droppedFrames++;
      Serial.println("Camera capture failed");
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (sendFrameUdp(fb)) {
      sentFrames++;
    } else {
      droppedFrames++;
    }

    esp_camera_fb_return(fb);

    const uint32_t nowMs = millis();
    if (nowMs - lastReportMs >= 2000) {
      const float seconds = (nowMs - lastReportMs) / 1000.0f;
      Serial.printf(
        "UDP video: %.1f fps sent, %u dropped, target=%s:%u\n",
        sentFrames / seconds,
        droppedFrames,
        LAPTOP_IP,
        LAPTOP_PORT
      );
      sentFrames = 0;
      droppedFrames = 0;
      lastReportMs = nowMs;
    }

    const uint32_t elapsedMs = millis() - frameStartMs;
    if (elapsedMs < frameIntervalMs) {
      vTaskDelay(pdMS_TO_TICKS(frameIntervalMs - elapsedMs));
    } else {
      taskYIELD();
    }
  }
}

static bool initCamera() {
  const bool hasPsram = psramFound();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = CAMERA_FRAME_SIZE;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = hasPsram ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = hasPsram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.jpeg_quality = CAMERA_JPEG_QUALITY;
  config.fb_count = hasPsram ? 2 : 1;

  Serial.printf("PSRAM: %s\n", hasPsram ? "yes" : "no");

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("Camera sensor not found");
    return false;
  }

  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }

  s->set_framesize(s, CAMERA_FRAME_SIZE);
  s->set_quality(s, CAMERA_JPEG_QUALITY);
  currentJpegQuality = CAMERA_JPEG_QUALITY;

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

  return true;
}

static void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  IPAddress localIp;
  IPAddress gatewayIp;
  IPAddress subnetMask;
  IPAddress dnsIp;
  if (!parseIpConfig(STATIC_IP, localIp) ||
      !parseIpConfig(GATEWAY_IP, gatewayIp) ||
      !parseIpConfig(WIFI_SUBNET_MASK, subnetMask) ||
      !parseIpConfig(DNS_IP, dnsIp)) {
    Serial.println("Invalid static WiFi IP config");
    return;
  }

  Serial.print("Configuring static IP: ");
  Serial.println(localIp);
  if (!WiFi.config(localIp, gatewayIp, subnetMask, dnsIp)) {
    Serial.println("WiFi.config failed");
    return;
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("WiFi connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(WIFI_RETRY_DELAY);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("WiFi connected. ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  delay(1000);
  Serial.println();
  Serial.println("ESP32-CAM UDP video boot");

  if (!laptopIp.fromString(LAPTOP_IP)) {
    Serial.printf("Invalid LAPTOP_IP: %s\n", LAPTOP_IP);
    return;
  }

  Serial.println("Initializing camera...");
  if (!initCamera()) {
    return;
  }
  Serial.println("Camera initialized");

#if defined(LED_GPIO_NUM)
  Serial.println("Keeping flash LED off...");
  setupLedFlash();
#endif

  Serial.println("Connecting WiFi...");
  connectWifi();
  udp.begin(0);
  controlUdp.begin(CONTROL_PORT);

  Serial.printf("UDP video destination: %s:%u\n", LAPTOP_IP, LAPTOP_PORT);
  Serial.printf("UDP control port: %u\n", CONTROL_PORT);
  Serial.printf(
    "Camera stream config: jpeg_quality=%u, target_fps=%u, udp_payload=%u\n",
    CAMERA_JPEG_QUALITY,
    TARGET_FPS,
    UDP_PAYLOAD_SIZE
  );

  xTaskCreatePinnedToCore(
    udpStreamTask,
    "udp_video",
    8192,
    NULL,
    1,
    NULL,
    1
  );
}

void loop() {
  handleControlUdp();
  vTaskDelay(pdMS_TO_TICKS(20));
}
