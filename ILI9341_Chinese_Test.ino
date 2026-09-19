#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "MiniChineseFont16.h"

#define TFT_SCLK 36
#define TFT_MOSI 35
#define TFT_RST  37
#define TFT_DC   38
#define TFT_CS   39
#define TFT_MISO -1

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);

bool readUtf8Codepoint(const String &input, size_t &index, uint32_t &codepoint) {
  uint8_t c0 = static_cast<uint8_t>(input[index]);
  size_t remaining = input.length() - index;

  if ((c0 & 0xE0) == 0xC0 && remaining >= 2) {
    uint8_t c1 = static_cast<uint8_t>(input[index + 1]);
    if ((c1 & 0xC0) != 0x80) {
      return false;
    }
    codepoint = ((c0 & 0x1F) << 6) | (c1 & 0x3F);
    index += 2;
    return true;
  }

  if ((c0 & 0xF0) == 0xE0 && remaining >= 3) {
    uint8_t c1 = static_cast<uint8_t>(input[index + 1]);
    uint8_t c2 = static_cast<uint8_t>(input[index + 2]);
    if ((c1 & 0xC0) != 0x80 || (c2 & 0xC0) != 0x80) {
      return false;
    }
    codepoint = ((c0 & 0x0F) << 12) | ((c1 & 0x3F) << 6) | (c2 & 0x3F);
    index += 3;
    return true;
  }

  if ((c0 & 0xF8) == 0xF0 && remaining >= 4) {
    uint8_t c1 = static_cast<uint8_t>(input[index + 1]);
    uint8_t c2 = static_cast<uint8_t>(input[index + 2]);
    uint8_t c3 = static_cast<uint8_t>(input[index + 3]);
    if ((c1 & 0xC0) != 0x80 || (c2 & 0xC0) != 0x80 || (c3 & 0xC0) != 0x80) {
      return false;
    }
    codepoint = ((c0 & 0x07) << 18) | ((c1 & 0x3F) << 12) | ((c2 & 0x3F) << 6) | (c3 & 0x3F);
    index += 4;
    return true;
  }

  return false;
}

void drawMiniChineseGlyph(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t color) {
  for (uint8_t row = 0; row < 16; row++) {
    uint8_t left = pgm_read_byte(bitmap + row * 2);
    uint8_t right = pgm_read_byte(bitmap + row * 2 + 1);
    for (uint8_t col = 0; col < 8; col++) {
      if (left & (0x80 >> col)) {
        tft.drawPixel(x + col, y + row, color);
      }
      if (right & (0x80 >> col)) {
        tft.drawPixel(x + 8 + col, y + row, color);
      }
    }
  }
}

void drawMixedText(int16_t x, int16_t y, const String &text, uint16_t color, uint8_t asciiSize) {
  tft.setTextSize(asciiSize);
  tft.setTextColor(color);
  int16_t cursorX = x;

  for (size_t i = 0; i < text.length() && cursorX < 306;) {
    uint8_t c = static_cast<uint8_t>(text[i]);
    if (c >= 32 && c <= 126) {
      tft.setCursor(cursorX, y);
      tft.write(static_cast<char>(c));
      cursorX += 6 * asciiSize;
      i++;
      continue;
    }

    uint32_t codepoint = 0;
    size_t before = i;
    if (readUtf8Codepoint(text, i, codepoint)) {
      const uint8_t *glyph = findMiniChineseGlyph(codepoint);
      if (glyph != nullptr) {
        drawMiniChineseGlyph(cursorX, y, glyph, color);
        cursorX += 17;
      } else {
        tft.setCursor(cursorX, y);
        tft.print("?");
        cursorX += 6 * asciiSize;
      }
    } else {
      tft.setCursor(cursorX, y);
      tft.print(".");
      cursorX += 6 * asciiSize;
      i = before + 1;
    }
  }
}

void setup() {
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.drawRect(4, 4, 312, 232, ILI9341_BLUE);

  tft.setTextSize(2);
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(12, 14);
  tft.print("Chinese Font Test");

  drawMixedText(12, 52, "你好", ILI9341_GREEN, 2);
  drawMixedText(12, 84, "发送测试", ILI9341_YELLOW, 2);
  drawMixedText(12, 116, "收到成功", ILI9341_WHITE, 2);
  drawMixedText(12, 148, "节点信号正常", ILI9341_ORANGE, 2);
  drawMixedText(12, 190, "OK: no U8g2 needed", ILI9341_CYAN, 1);
}

void loop() {
}
