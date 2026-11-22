#include <WiFi.h>
#include <TFT_eSPI.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <errno.h>
#include <FS.h>
#include <SD.h>
#include <lwip/tcp.h>
#include <lwip/opt.h>
#include <lwip/sockets.h>
#include <driver/i2s_std.h>
#include <JPEGDEC.h>

// ============================================================
// WIFI SETTINGS
// ============================================================

const char* WIFI_SSID = "Your_Name";
const char* WIFI_PASSWORD = "Your_Pass";

WiFiServer server(3333);
WiFiClient client;

bool clientActive = false;
volatile bool wifiHasIp = false;

WiFiServer statusServer(3335);
WiFiClient statusClient;

// ============================================================
// STATUS / DIAGNOSTICS
// ============================================================

uint32_t totalFramesReceived = 0;
uint32_t lastReceiveTimeMs = 0;
uint32_t lastDecodeTimeMs = 0;
uint32_t lastDisplayTimeMs = 0;

uint32_t currentFPS = 0;
uint32_t lastFPSCheck = 0;
uint32_t framesAtLastFPSCheck = 0;

// ============================================================
// DISPLAY
// ============================================================

TFT_eSPI tft = TFT_eSPI();

// ============================================================
// TFT BACKLIGHT BRIGHTNESS
// ============================================================

#define TFT_BACKLIGHT_PIN 21

uint8_t currentBrightness = 100;

void setBacklightBrightness(uint8_t brightness) {

  brightness = constrain(brightness, 0, 100);

  currentBrightness = brightness;

  uint32_t duty = map(brightness, 0, 100, 0, 255);

  ledcWrite(TFT_BACKLIGHT_PIN, duty);

  Serial.printf(
    "TFT BRIGHTNESS: %u%%\n",
    currentBrightness
  );
}

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

#define MAX_JPEG_SIZE (96 * 1024)

JPEGDEC jpegDecoder;

uint16_t lastFrameWidth = 0;
uint16_t lastFrameHeight = 0;
int16_t lastDrawX = -32768;
int16_t lastDrawY = -32768;

// ESP32 VIDEO POSITION OFFSET
int16_t displayOffsetX = 0;
int16_t displayOffsetY = 0;

uint32_t jpegPushUs = 0;

int jpegOutput(JPEGDRAW* pDraw) {
  uint32_t start = micros();

  tft.pushImageDMA(
    pDraw->x,
    pDraw->y,
    pDraw->iWidth,
    pDraw->iHeight,
    pDraw->pPixels
  );

  jpegPushUs += micros() - start;

  return 1;
}

// ============================================================
// PLAYBACK STATE
// ============================================================

// Physical buttons are disabled.
// GPIO18 and GPIO19 are reserved for I2S audio.

volatile bool playing = true;

// ============================================================
// MAX98357A I2S AUDIO SETTINGS
// ============================================================

#define AUDIO_SAMPLE_RATE 16000

#define AUDIO_PACKET_MAX 1024
#define AUDIO_RING_SIZE 16384
#define AUDIO_PREBUFFER_SIZE 4096

#define AUDIO_SAMPLES_PER_BLOCK 256

// MAX98357A CONNECTIONS
#define I2S_BCLK 18
#define I2S_LRC  19
#define I2S_DOUT 23

#define I2S_PORT I2S_NUM_0

// Incoming audio from the PC is unsigned 8-bit mono PCM.
// Output is signed 16-bit stereo I2S.
// Both stereo channels receive the same mono sample.

static uint8_t audioBuffer[AUDIO_PACKET_MAX];
static uint8_t audioSamples[AUDIO_SAMPLES_PER_BLOCK];

static int16_t i2sSamples[AUDIO_SAMPLES_PER_BLOCK * 2];

RingbufHandle_t audioRing = nullptr;

static i2s_chan_handle_t audioTxChannel = nullptr;

volatile bool audioReady = false;

bool audioPrebuffering = true;

static volatile uint32_t audioUnderruns = 0;
static volatile uint32_t audioWriteErrors = 0;

// ============================================================
// I2S AUDIO PLAYBACK TASK
// ============================================================

void audioPlaybackTask(void* parameter) {

  uint8_t* currentItem = nullptr;

  size_t currentItemSize = 0;
  size_t currentItemOffset = 0;

  uint32_t playbackBlocks = 0;

  while (true) {

    // Unsigned 8-bit PCM silence is 128.
    memset(audioSamples, 128, sizeof(audioSamples));

    size_t copied = 0;

    if (playing && audioReady && audioRing != nullptr) {

      // Wait for enough audio before beginning playback.
      if (audioPrebuffering && currentItem == nullptr) {

        size_t freeSpace =
          xRingbufferGetCurFreeSize(audioRing);

        size_t queued = AUDIO_RING_SIZE - freeSpace;

        if (queued >= AUDIO_PREBUFFER_SIZE) {

          audioPrebuffering = false;

          Serial.println("AUDIO PREBUFFER READY");
        }
      }

      if (!audioPrebuffering) {

        while (copied < sizeof(audioSamples)) {

          if (currentItem == nullptr) {

            currentItem = (uint8_t*)xRingbufferReceive(
              audioRing,
              &currentItemSize,
              0
            );

            currentItemOffset = 0;

            if (currentItem == nullptr) {
              break;
            }
          }

          size_t available =
            currentItemSize - currentItemOffset;

          size_t needed =
            sizeof(audioSamples) - copied;

          size_t amount = min(available, needed);

          memcpy(
            audioSamples + copied,
            currentItem + currentItemOffset,
            amount
          );

          copied += amount;
          currentItemOffset += amount;

          if (currentItemOffset >= currentItemSize) {

            vRingbufferReturnItem(audioRing, currentItem);

            currentItem = nullptr;
            currentItemSize = 0;
            currentItemOffset = 0;
          }
        }

        if (copied < sizeof(audioSamples)) {
          audioUnderruns++;
        }
      }

    } else {

      // Pause: reset prebuffering and discard queued audio.
      audioPrebuffering = true;

      if (currentItem != nullptr) {

        vRingbufferReturnItem(audioRing, currentItem);

        currentItem = nullptr;
        currentItemSize = 0;
        currentItemOffset = 0;
      }

      if (audioRing != nullptr) {

        size_t discardedSize = 0;

        uint8_t* discarded;

        while ((discarded = (uint8_t*)xRingbufferReceive(
                  audioRing,
                  &discardedSize,
                  0
                )) != nullptr) {

          vRingbufferReturnItem(audioRing, discarded);
        }
      }
    }

    // Convert unsigned 8-bit mono PCM to signed 16-bit PCM.
    // Duplicate each sample into the left and right I2S slots.

    for (int i = 0; i < AUDIO_SAMPLES_PER_BLOCK; i++) {

      int16_t sample =
        ((int16_t)audioSamples[i] - 128) << 8;

      i2sSamples[i * 2] = sample;
      i2sSamples[i * 2 + 1] = sample;
    }

    // Send the block to the MAX98357A.
    size_t bytesWritten = 0;

    esp_err_t result = i2s_channel_write(
      audioTxChannel,
      i2sSamples,
      sizeof(i2sSamples),
      &bytesWritten,
      portMAX_DELAY
    );

    playbackBlocks++;

    if (result != ESP_OK ||
        bytesWritten != sizeof(i2sSamples)) {

      audioWriteErrors++;

      Serial.printf(
        "I2S WRITE ERROR: %s | WRITTEN: %u / %u\n",
        esp_err_to_name(result),
        (unsigned int)bytesWritten,
        (unsigned int)sizeof(i2sSamples)
      );
    }

    if (playbackBlocks % 100 == 0) {

      Serial.printf(
        "I2S AUDIO | COPIED: %u / %u | UNDERRUNS: %lu | WRITE ERRORS: %lu | RING FREE: %u\n",
        (unsigned int)copied,
        (unsigned int)sizeof(audioSamples),
        (unsigned long)audioUnderruns,
        (unsigned long)audioWriteErrors,
        (unsigned int)xRingbufferGetCurFreeSize(audioRing)
      );
    }
  }
}

// ============================================================
// AUDIO INITIALIZATION
// ============================================================

void setupAudio() {

  audioReady = false;

  audioRing = xRingbufferCreate(
    AUDIO_RING_SIZE,
    RINGBUF_TYPE_BYTEBUF
  );

  if (audioRing == nullptr) {

    Serial.println("AUDIO ERROR: RING BUFFER CREATION FAILED");

    return;
  }

  // Allocate an I2S TX channel.
  i2s_chan_config_t channelConfig =
    I2S_CHANNEL_DEFAULT_CONFIG(
      I2S_PORT,
      I2S_ROLE_MASTER
    );

  channelConfig.dma_desc_num = 8;
  channelConfig.dma_frame_num = 256;

  esp_err_t result = i2s_new_channel(
    &channelConfig,
    &audioTxChannel,
    nullptr
  );

  if (result != ESP_OK) {

    Serial.printf(
      "I2S ERROR: CHANNEL CREATION FAILED: %s\n",
      esp_err_to_name(result)
    );

    vRingbufferDelete(audioRing);
    audioRing = nullptr;

    audioTxChannel = nullptr;

    return;
  }

  // Configure standard Philips I2S.
  // Stereo output is used with identical left/right samples.

  i2s_std_config_t standardConfig = {

    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(
      AUDIO_SAMPLE_RATE
    ),

    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
      I2S_DATA_BIT_WIDTH_16BIT,
      I2S_SLOT_MODE_STEREO
    ),

    .gpio_cfg = {

      .mclk = I2S_GPIO_UNUSED,

      .bclk = (gpio_num_t)I2S_BCLK,

      .ws = (gpio_num_t)I2S_LRC,

      .dout = (gpio_num_t)I2S_DOUT,

      .din = I2S_GPIO_UNUSED,

      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false
      }
    }
  };

  result = i2s_channel_init_std_mode(
    audioTxChannel,
    &standardConfig
  );

  if (result != ESP_OK) {

    Serial.printf(
      "I2S ERROR: STANDARD MODE INIT FAILED: %s\n",
      esp_err_to_name(result)
    );

    i2s_del_channel(audioTxChannel);
    audioTxChannel = nullptr;

    vRingbufferDelete(audioRing);
    audioRing = nullptr;

    return;
  }

  // Enable the I2S transmitter.
  result = i2s_channel_enable(audioTxChannel);

  if (result != ESP_OK) {

    Serial.printf(
      "I2S ERROR: CHANNEL ENABLE FAILED: %s\n",
      esp_err_to_name(result)
    );

    i2s_del_channel(audioTxChannel);
    audioTxChannel = nullptr;

    vRingbufferDelete(audioRing);
    audioRing = nullptr;

    return;
  }

  // Start audio playback task.
  TaskHandle_t audioTaskHandle = nullptr;

  BaseType_t taskResult = xTaskCreatePinnedToCore(
    audioPlaybackTask,
    "AudioPlayback",
    4096,
    nullptr,
    5,
    &audioTaskHandle,
    1
  );

  if (taskResult != pdPASS) {

    Serial.println("AUDIO ERROR: PLAYBACK TASK CREATION FAILED");

    i2s_channel_disable(audioTxChannel);
    i2s_del_channel(audioTxChannel);

    audioTxChannel = nullptr;

    vRingbufferDelete(audioRing);
    audioRing = nullptr;

    return;
  }

  audioReady = true;

  Serial.println();
  Serial.println("================================");
  Serial.println("MAX98357A I2S AUDIO INITIALIZED");
  Serial.println("BCLK: GPIO18");
  Serial.println("LRC: GPIO19");
  Serial.println("DIN: GPIO23");
  Serial.println("FORMAT: PHILIPS I2S");
  Serial.println("OUTPUT: 16-BIT STEREO");
  Serial.println("SOURCE: MONO UNSIGNED 8-BIT PCM");
  Serial.println("SAMPLE RATE: 16000 HZ");
  Serial.println("PLAYBACK BLOCK: 256 MONO SAMPLES");
  Serial.println("DMA DESCRIPTORS: 8");
  Serial.println("================================");
}

// ============================================================
// WIFI EVENT HANDLER
// ============================================================

void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {

  switch (event) {

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:

      Serial.println("WIFI DRIVER: CONNECTED TO ROUTER");

      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:

      wifiHasIp = true;

      Serial.printf(
        "WIFI DRIVER: GOT IP %s\n",
        WiFi.localIP().toString().c_str()
      );

      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:

      wifiHasIp = false;

      Serial.printf(
        "WIFI DRIVER DISCONNECTED | Reason: %d\n",
        info.wifi_sta_disconnected.reason
      );

      break;

    case ARDUINO_EVENT_WIFI_STA_LOST_IP:

      wifiHasIp = false;

      Serial.println("WIFI DRIVER: LOST IP");

      break;

    default:
      break;
  }
}

// ============================================================
// READ NETWORK DATA
// ============================================================

bool readExact(uint8_t* buffer, size_t length) {

  size_t received = 0;

  unsigned long start = millis();

  int socketFd = client.fd();

  if (socketFd < 0) {
    return false;
  }

  while (received < length) {

    if (!clientActive || !client.connected()) {
      return false;
    }

    int n = recv(
      socketFd,
      buffer + received,
      length - received,
      MSG_DONTWAIT
    );

    if (n > 0) {

      received += n;

      start = millis();
    }

    else if (n == 0) {

      return false;
    }

    else {

      int err = errno;

      if (
        err != EWOULDBLOCK &&
        err != EAGAIN &&
        err != EINTR
      ) {

        Serial.printf("RECV ERROR: %d\n", err);

        return false;
      }

      if (millis() - start > 15000) {

        Serial.println("CLIENT READ TIMEOUT");

        return false;
      }

      delay(0);
    }
  }

  return true;
}

// ============================================================
// DISCONNECT CLIENT
// ============================================================

void disconnectClient() {

  if (!clientActive) {
    return;
  }

  clientActive = false;

  Serial.println("PC DISCONNECTED");

  client.stop();
}

// ============================================================
// JPEG RECEIVER
// ============================================================

bool receiveJpeg() {

  uint8_t header[8];

  if (!readExact(header, sizeof(header))) {
    return false;
  }

  uint16_t frameWidth =
    ((uint16_t)header[0] << 8) | header[1];

  uint16_t frameHeight =
    ((uint16_t)header[2] << 8) | header[3];

  uint32_t frameSize =
    ((uint32_t)header[4] << 24) |
    ((uint32_t)header[5] << 16) |
    ((uint32_t)header[6] << 8) |
    header[7];

  if (
    frameWidth == 0 ||
    frameHeight == 0 ||
    frameWidth > SCREEN_WIDTH ||
    frameHeight > SCREEN_HEIGHT ||
    frameSize == 0 ||
    frameSize > MAX_JPEG_SIZE
  ) {

    Serial.println("INVALID JPEG");

    return false;
  }

  uint8_t* jpegBuffer = (uint8_t*)heap_caps_malloc(
    frameSize,
    MALLOC_CAP_8BIT
  );

  if (!jpegBuffer) {

    Serial.println("JPEG MEMORY ALLOCATION FAILED");

    return false;
  }

  uint32_t receiveStart = micros();

  if (!readExact(jpegBuffer, frameSize)) {

    heap_caps_free(jpegBuffer);

    return false;
  }

  uint32_t receiveUs = micros() - receiveStart;

  uint32_t openStart = micros();

  int opened = jpegDecoder.openRAM(
    jpegBuffer,
    (int)frameSize,
    jpegOutput
  );

  uint32_t openUs = micros() - openStart;

  if (!opened) {

    Serial.println("JPEGDEC OPEN FAILED");

    heap_caps_free(jpegBuffer);

    return false;
  }

  uint16_t decodedWidth = jpegDecoder.getWidth();

  uint16_t decodedHeight = jpegDecoder.getHeight();

  if (
    decodedWidth != frameWidth ||
    decodedHeight != frameHeight
  ) {

    Serial.printf(
      "JPEG DIMENSION ERROR: %u x %u\n",
      decodedWidth,
      decodedHeight
    );

    jpegDecoder.close();

    heap_caps_free(jpegBuffer);

    return false;
  }


int16_t drawX =
  (SCREEN_WIDTH - frameWidth) / 2 + displayOffsetX;

int16_t drawY =
  (SCREEN_HEIGHT - frameHeight) / 2 + displayOffsetY;

  // CLEAR SCREEN ONLY WHEN SIZE OR POSITION CHANGES
  if (
    frameWidth != lastFrameWidth ||
    frameHeight != lastFrameHeight ||
    drawX != lastDrawX ||
    drawY != lastDrawY
  ) {

    tft.fillScreen(TFT_BLACK);

    lastFrameWidth = frameWidth;
    lastFrameHeight = frameHeight;

    lastDrawX = drawX;
    lastDrawY = drawY;
  }

  jpegDecoder.setPixelType(RGB565_BIG_ENDIAN);

  jpegPushUs = 0;

  tft.startWrite();

  tft.setSwapBytes(false);

  uint32_t decodeStart = micros();

  int decoded = jpegDecoder.decode(
    drawX,
    drawY,
    JPEG_USES_DMA
  );

  tft.dmaWait();

  uint32_t totalDecodeUs = micros() - decodeStart;

  tft.endWrite();

  uint32_t pushUs = jpegPushUs;

  uint32_t decodeUs =
    totalDecodeUs > pushUs ?
    totalDecodeUs - pushUs : 0;

  int jpegError = jpegDecoder.getLastError();

  jpegDecoder.close();

  heap_caps_free(jpegBuffer);

  if (!decoded) {

    Serial.printf(
      "JPEGDEC DECODE FAILED: %d\n",
      jpegError
    );

    return false;
  }

  totalFramesReceived++;

  lastReceiveTimeMs = receiveUs / 1000;

  lastDecodeTimeMs = decodeUs / 1000;

  lastDisplayTimeMs = pushUs / 1000;

  static uint32_t jpegFrames = 0;

  jpegFrames++;

  if (jpegFrames % 30 == 0) {

    Serial.printf(
      "JPEGDEC | RECEIVE: %.1f ms | OPEN: %.1f ms | DECODE: %.1f ms | TFT: %.1f ms | TOTAL: %.1f ms\n",
      receiveUs / 1000.0,
      openUs / 1000.0,
      decodeUs / 1000.0,
      pushUs / 1000.0,
      totalDecodeUs / 1000.0
    );
  }

  return true;
}

// ============================================================
// ESP32 DISPLAY POSITION RECEIVER
// ============================================================

bool receivePosition() {

  uint8_t positionBytes[4];

  if (!readExact(positionBytes, sizeof(positionBytes))) {
    return false;
  }

  displayOffsetX =
    (int16_t)(((uint16_t)positionBytes[0] << 8) |
               positionBytes[1]);

  displayOffsetY =
    (int16_t)(((uint16_t)positionBytes[2] << 8) |
               positionBytes[3]);

  Serial.printf(
    "DISPLAY POSITION UPDATED | X: %d | Y: %d\n",
    displayOffsetX,
    displayOffsetY
  );

  return true;

}

  // ============================================================
// TFT BRIGHTNESS RECEIVER
// ============================================================

bool receiveBrightness() {

  uint8_t brightness;

  if (!readExact(&brightness, 1)) {
    return false;
  }

  setBacklightBrightness(brightness);

  return true;
}

// ============================================================
// AUDIO RECEIVER
// ============================================================

bool receiveAudio() {

  uint8_t lengthBytes[2];

  if (!readExact(lengthBytes, sizeof(lengthBytes))) {
    return false;
  }

  uint16_t audioLength =
    ((uint16_t)lengthBytes[0] << 8) |
    lengthBytes[1];

  if (
    audioLength == 0 ||
    audioLength > AUDIO_PACKET_MAX
  ) {

    Serial.printf(
      "INVALID AUDIO PACKET: %u\n",
      audioLength
    );

    return false;
  }

  if (!readExact(audioBuffer, audioLength)) {
    return false;
  }

  static uint32_t packetsReceived = 0;

  static uint32_t bytesReceived = 0;

  static uint32_t packetsDiscarded = 0;

  packetsReceived++;

  bytesReceived += audioLength;

  if (
    !playing ||
    !audioReady ||
    audioRing == nullptr
  ) {

    return true;
  }

  // Make room for the newest audio packet.
  while (
    xRingbufferGetCurFreeSize(audioRing) < audioLength
  ) {

    size_t oldLength = 0;

    uint8_t* oldPacket = (uint8_t*)xRingbufferReceive(
      audioRing,
      &oldLength,
      0
    );

    if (oldPacket == nullptr) {
      break;
    }

    vRingbufferReturnItem(audioRing, oldPacket);

    packetsDiscarded++;
  }

  if (
    xRingbufferSend(
      audioRing,
      audioBuffer,
      audioLength,
      0
    ) != pdTRUE
  ) {

    packetsDiscarded++;
  }

  if (packetsReceived % 100 == 0) {

    Serial.printf(
      "AUDIO INPUT | PACKETS: %lu | BYTES: %lu | DISCARDED: %lu | RING FREE: %u\n",
      (unsigned long)packetsReceived,
      (unsigned long)bytesReceived,
      (unsigned long)packetsDiscarded,
      (unsigned int)xRingbufferGetCurFreeSize(audioRing)
    );
  }

  return true;
}

// ============================================================
// PACKET DISPATCHER - JPEG VIDEO + AUDIO ONLY
// ============================================================

void receivePacket() {

  if (!clientActive || !client.connected()) {

    disconnectClient();

    return;
  }

  uint8_t firstByte;

  if (!readExact(&firstByte, 1)) {

    disconnectClient();

    return;
  }

  // AUDIO
  if (firstByte == 'A') {

    if (!receiveAudio()) {
      disconnectClient();
    }

    return;
  }

    // DISPLAY POSITION
  if (firstByte == 'O') {

    if (!receivePosition()) {
      disconnectClient();
    }

    return;
  }

    // TFT BRIGHTNESS
  if (firstByte == 'B') {

    if (!receiveBrightness()) {
      disconnectClient();
    }

    return;
  }

  // JPEG VIDEO
  if (firstByte == 'J') {

    if (!receiveJpeg()) {
      disconnectClient();
    }

    return;
  }

  Serial.printf(
    "UNKNOWN PACKET TYPE: 0x%02X\n",
    firstByte
  );

  disconnectClient();
}

// ============================================================
// WIFI RECONNECTION
// ============================================================

void maintainWiFi() {

  static bool previousState = false;

  bool connected = wifiHasIp;

  if (connected && !previousState) {

    Serial.println("WIFI CONNECTED");
  }

  if (!connected && previousState) {

    Serial.println("WIFI CONNECTION LOST");

    disconnectClient();
  }

  previousState = connected;
}

// ============================================================
// WIFI STATUS SENDER
// ============================================================

void updateWiFiStatus() {

  static uint32_t lastStatusUpdate = 0;

  // Accept Python status connection.
  if (!statusClient || !statusClient.connected()) {

    if (statusClient) {
      statusClient.stop();
    }

    WiFiClient incoming = statusServer.accept();

    if (incoming) {

      statusClient = incoming;

      statusClient.setNoDelay(true);

      Serial.println("PYTHON STATUS CLIENT CONNECTED");
    }
  }

  uint32_t now = millis();

  // Update FPS once per second.
  if (now - lastFPSCheck >= 1000) {

    currentFPS =
      totalFramesReceived - framesAtLastFPSCheck;

    framesAtLastFPSCheck = totalFramesReceived;

    lastFPSCheck = now;
  }

  // Send status twice per second.
  if (now - lastStatusUpdate < 500) {
    return;
  }

  lastStatusUpdate = now;

  if (!statusClient || !statusClient.connected()) {
    return;
  }

  statusClient.printf(
    "WIFI=%s\n"
    "RSSI=%d\n"
    "PC_CONNECTED=%d\n"
    "PLAYING=%d\n"
    "FPS=%lu\n"
    "FRAMES_RECEIVED=%lu\n"
    "RECEIVE_MS=%lu\n"
    "DECODE_MS=%lu\n"
    "DISPLAY_MS=%lu\n"
    "FREE_HEAP=%u\n",

    wifiHasIp ? "CONNECTED" : "DISCONNECTED",

    WiFi.RSSI(),

    clientActive ? 1 : 0,

    playing ? 1 : 0,

    (unsigned long)currentFPS,

    (unsigned long)totalFramesReceived,

    (unsigned long)lastReceiveTimeMs,

    (unsigned long)lastDecodeTimeMs,

    (unsigned long)lastDisplayTimeMs,

    ESP.getFreeHeap()
  );
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  Serial.printf(
    "CHIP MODEL: %s\n",
    ESP.getChipModel()
  );

  Serial.printf(
    "CHIP REVISION: %d\n",
    ESP.getChipRevision()
  );

  Serial.printf(
    "ESP-IDF VERSION: %s\n",
    esp_get_idf_version()
  );

  Serial.printf(
    "TCP RECEIVE WINDOW: %d\n",
    TCP_WND
  );

  Serial.printf(
    "TCP MSS: %d\n",
    TCP_MSS
  );

  Serial.printf(
    "TCP SEND BUFFER: %d\n",
    TCP_SND_BUF
  );

  WiFi.onEvent(WiFiEvent);

  Serial.printf(
    "\nRESET REASON: %d\n",
    (int)esp_reset_reason()
  );

  Serial.printf(
    "FREE HEAP: %u bytes\n",
    ESP.getFreeHeap()
  );

  Serial.printf(
    "MINIMUM FREE HEAP: %u bytes\n",
    ESP.getMinFreeHeap()
  );

  // Initialize display.
  tft.init();

    // INITIALIZE TFT BACKLIGHT PWM
  if (ledcAttach(TFT_BACKLIGHT_PIN, 5000, 8)) {

    setBacklightBrightness(100);

    Serial.println("TFT BACKLIGHT PWM READY");

  } else {

    Serial.println("TFT BACKLIGHT PWM FAILED");
  }

  if (tft.initDMA()) {

    Serial.println("TFT DMA READY");

  } else {

    Serial.println("TFT DMA INIT FAILED");
  }

  tft.setRotation(1);

  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.setTextDatum(MC_DATUM);

  tft.drawString(
    "CYD VIDEO PLAYER",
    160,
    80,
    2
  );

  tft.drawString(
    "Connecting WiFi...",
    160,
    120,
    2
  );

  // Initialize MAX98357A I2S audio.
  setupAudio();

  Serial.printf(
    "LARGEST INTERNAL BLOCK: %u\n",
    heap_caps_get_largest_free_block(
      MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
    )
  );

  // Wi-Fi configuration.
  WiFi.mode(WIFI_STA);

  WiFi.setSleep(false);

  WiFi.setAutoReconnect(true);

  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );

  Serial.print("Connecting to WiFi");

  while (!wifiHasIp) {

    delay(500);

    Serial.print(".");
  }

  Serial.println();

  Serial.println("WIFI CONNECTED");

  Serial.print("ESP32 IP ADDRESS: ");

  Serial.println(WiFi.localIP());

  // Start video/audio server.
  server.begin();

  server.setNoDelay(true);

  // Start status server.
  statusServer.begin();

  statusServer.setNoDelay(true);

  WiFi.setSleep(false);

  Serial.println(
    "VIDEO/AUDIO SERVER STARTED ON PORT 3333"
  );

  Serial.println(
    "STATUS SERVER STARTED ON PORT 3335"
  );

  tft.fillScreen(TFT_BLACK);

  tft.drawString(
    "WAITING FOR PC",
    160,
    110,
    2
  );

  Serial.println("VIDEO/AUDIO SERVER STARTED");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  maintainWiFi();

  updateWiFiStatus();

  if (!wifiHasIp) {

    disconnectClient();

    delay(10);

    return;
  }

  if (clientActive && !client.connected()) {

    disconnectClient();
  }

  if (!clientActive) {

    WiFiClient incoming = server.accept();

    if (incoming.connected()) {

      client = incoming;

      clientActive = true;

      client.setNoDelay(true);

      client.setTimeout(15000);

      int receiveBuffer = 16384;

      int socketResult = client.setSocketOption(
        SOL_SOCKET,
        SO_RCVBUF,
        &receiveBuffer,
        sizeof(receiveBuffer)
      );

      Serial.printf(
        "SO_RCVBUF SET: %d | RESULT: %d\n",
        receiveBuffer,
        socketResult
      );

      int actualReceiveBuffer = 0;

      int readResult = client.getSocketOption(
        SOL_SOCKET,
        SO_RCVBUF,
        &actualReceiveBuffer,
        sizeof(actualReceiveBuffer)
      );

      Serial.printf(
        "SO_RCVBUF ACTUAL: %d | READ RESULT: %d\n",
        actualReceiveBuffer,
        readResult
      );

#ifdef TCP_WINDOW

      int tcpWindow = 32768;

      int tcpWindowResult = lwip_setsockopt(
        client.fd(),
        IPPROTO_TCP,
        TCP_WINDOW,
        &tcpWindow,
        sizeof(tcpWindow)
      );

      Serial.printf(
        "TCP_WINDOW SET: %d | RESULT: %d\n",
        tcpWindow,
        tcpWindowResult
      );

      int actualWindow = 0;

      socklen_t optLen = sizeof(actualWindow);

      int getResult = lwip_getsockopt(
        client.fd(),
        IPPROTO_TCP,
        TCP_WINDOW,
        &actualWindow,
        &optLen
      );

      Serial.printf(
        "TCP_WINDOW ACTUAL: %d | RESULT: %d\n",
        actualWindow,
        getResult
      );

#else

      Serial.println("TCP_WINDOW OPTION NOT AVAILABLE");

#endif

      Serial.println("PC CONNECTED");

      Serial.println("WAITING FOR PACKETS");
    }

    delay(2);

    return;
  }

  if (client.available() > 0) {

    receivePacket();
  }

  delay(1);
}
