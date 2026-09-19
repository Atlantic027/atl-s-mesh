/*
  Heltec Wireless Stick V3 + 2.4 inch ILI9341 SPI screen test.

  Arduino IDE libraries:
  - Adafruit GFX Library
  - Adafruit ILI9341

  Wiring:
  Screen GND -> Heltec GND
  Screen VCC -> Heltec 3V3
  Screen SCL -> Heltec GPIO36
  Screen SDA -> Heltec GPIO35
  Screen RES -> Heltec GPIO37
  Screen DC  -> Heltec GPIO38
  Screen CS  -> Heltec GPIO39
  Screen BLK -> Heltec 3V3
*/

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#define TFT_SCLK 36
#define TFT_MOSI 35
#define TFT_RST  37
#define TFT_DC   38
#define TFT_CS   39
#define TFT_MISO -1

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ILI9341 screen test start");

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);

  tft.setTextWrap(false);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(20, 20);
  tft.println("LoRa Mesh Terminal");

  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(20, 60);
  tft.println("Screen OK");

  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(20, 100);
  tft.println("NODE-01");

  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(20, 140);
  tft.println("RSSI: -- dBm");
  tft.setCursor(20, 170);
  tft.println("SNR : -- dB");

  tft.drawRect(10, 10, 300, 210, ILI9341_BLUE);
  tft.fillRect(20, 200, 280, 18, ILI9341_DARKGREEN);

  Serial.println("Draw complete");
}

void loop() {
  static uint32_t last = 0;
  static int counter = 0;

  if (millis() - last > 1000) {
    last = millis();
    counter++;

    tft.fillRect(115, 200, 80, 18, ILI9341_DARKGREEN);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.setCursor(28, 204);
    tft.print("running seconds: ");
    tft.print(counter);

    Serial.print("running seconds: ");
    Serial.println(counter);
  }
}
