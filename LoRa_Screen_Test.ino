/*
  Heltec Wireless Stick V3 + SX1262 433 MHz + ILI9341 screen test.

  Libraries:
  - RadioLib
  - Adafruit GFX Library
  - Adafruit ILI9341

  Upload this sketch to two boards.
  Change NODE_ID to 1 on the first board and 2 on the second board.

  Screen wiring:
  GND -> GND
  VCC -> 3V3
  SCL -> GPIO36
  SDA -> GPIO35
  RES -> GPIO37
  DC  -> GPIO38
  CS  -> GPIO39
  BLK -> 3V3
*/

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// ===== Change this before uploading to each board =====
#define NODE_ID 1

// ===== LoRa settings for 433 MHz board =====
#define LORA_FREQ_MHZ 433.0
#define LORA_BW_KHZ 125.0
#define LORA_SF 9
#define LORA_CR 5
#define LORA_SYNC_WORD 0x12
#define LORA_POWER_DBM 17
#define LORA_PREAMBLE_LEN 8

// ===== Heltec Wireless Stick V3 common SX1262 pins =====
#define LORA_SCK 9
#define LORA_MISO 11
#define LORA_MOSI 10
#define LORA_NSS 8
#define LORA_RST 12
#define LORA_BUSY 13
#define LORA_DIO1 14

// ===== ILI9341 pins =====
#define TFT_SCLK 36
#define TFT_MOSI 35
#define TFT_RST  37
#define TFT_DC   38
#define TFT_CS   39
#define TFT_MISO -1

SPIClass loraSpi(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSpi);
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);

String serialLine;
String lastRx = "none";
String lastTx = "none";
float lastRssi = 0;
float lastSnr = 0;
uint32_t lastAutoSend = 0;

void drawUi(const char *status) {
  tft.fillScreen(ILI9341_BLACK);
  tft.drawRect(4, 4, 312, 232, ILI9341_BLUE);

  tft.setTextWrap(false);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(12, 14);
  tft.print("LoRa Mesh NODE-");
  tft.print(NODE_ID);

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(12, 48);
  tft.print("Status: ");
  tft.print(status);

  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(12, 76);
  tft.print("TX: ");
  tft.print(lastTx.substring(0, 34));

  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(12, 104);
  tft.print("RX: ");
  tft.print(lastRx.substring(0, 34));

  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(12, 140);
  tft.print("RSSI: ");
  tft.print(lastRssi, 1);
  tft.print(" dBm");

  tft.setCursor(12, 162);
  tft.print("SNR : ");
  tft.print(lastSnr, 1);
  tft.print(" dB");

  tft.setTextColor(ILI9341_ORANGE);
  tft.setCursor(12, 200);
  tft.print("Serial input + Enter to send");
}

void sendMessage(const String &msg) {
  String packet = "NODE" + String(NODE_ID) + ":" + msg;
  radio.standby();
  int state = radio.transmit(packet);
  if (state == RADIOLIB_ERR_NONE) {
    lastTx = packet;
    Serial.print("TX OK: ");
    Serial.println(packet);
    drawUi("TX OK");
  } else {
    Serial.print("TX failed: ");
    Serial.println(state);
    drawUi("TX FAILED");
  }
  radio.startReceive();
}

void setup() {
  Serial.begin(115200);
  delay(1200);

  tft.begin();
  tft.setRotation(1);
  drawUi("BOOT");

  Serial.println();
  Serial.println("LoRa + Screen test");
  Serial.print("NODE_ID=");
  Serial.println(NODE_ID);

  loraSpi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  int state = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR, LORA_SYNC_WORD,
                          LORA_POWER_DBM, LORA_PREAMBLE_LEN);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("Radio begin failed: ");
    Serial.println(state);
    drawUi("RADIO FAIL");
    while (true) {
      delay(1000);
    }
  }

  radio.startReceive();
  drawUi("READY");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      serialLine.trim();
      if (serialLine.length() > 0) {
        sendMessage(serialLine);
      }
      serialLine = "";
    } else if (serialLine.length() < 80) {
      serialLine += c;
    }
  }

  String str;
  int state = radio.readData(str);
  if (state == RADIOLIB_ERR_NONE) {
    if (!str.startsWith("NODE" + String(NODE_ID) + ":")) {
      lastRx = str;
      lastRssi = radio.getRSSI();
      lastSnr = radio.getSNR();
      Serial.print("RX: ");
      Serial.print(str);
      Serial.print(" RSSI=");
      Serial.print(lastRssi);
      Serial.print(" SNR=");
      Serial.println(lastSnr);
      drawUi("RX OK");
    }
    radio.startReceive();
  } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
    radio.startReceive();
  }

  if (millis() - lastAutoSend > 15000) {
    lastAutoSend = millis();
    sendMessage("ping " + String(millis() / 1000));
  }
}
