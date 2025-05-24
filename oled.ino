#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>

// Define screen dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define MAX_CHAR 12
#define SCROLL_SPEED 1200

// unsigned long lastScroll = 0;

// Initialize U8g2 for a monochrome display (SSD1306)
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

int utf8_length(const char * str);
bool isUTF8ContinuationByte(char c);
int calculateLines(const char * text, int maxCharsPerLine);
void printWrappedTextUTF8(const char * text, int maxCharsPerLine, int startLine);
void independentScroll(const char * text, int maxCharsPerLine, unsigned long currentMillis);
void clearDisplay();
void displaySetup();
void drawBackground();

int utf8_length(const char * str) {
  int length = 0;
  unsigned char * s = (unsigned char * ) str;

  while ( * s) {
    if (( * s & 0x80) == 0) {
      s++;
    } else if (( * s & 0xE0) == 0xC0) {
      s += 2;
    } else if (( * s & 0xF0) == 0xE0) {
      s += 3;
    } else if (( * s & 0xF8) == 0xF0) {
      s += 4;
    }
    length++;
  }
  return length;
}

bool isUTF8ContinuationByte(char c) {
  return (c & 0xC0) == 0x80;
}

int calculateLines(const char * text, int maxCharsPerLine) {
  int lines = 1;
  int charCount = 0;
  const char * start = text;
  const char * lastSpace = text;

  while ( * text) {
    if ( * text == ' ' || * text == '\t' || * text == '\n' || * (text + 1) == '\0') {
      lastSpace = text;
    }

    if (!isUTF8ContinuationByte( * text)) {
      charCount++;
    }

    if (charCount >= maxCharsPerLine && lastSpace != text) {
      lines++;
      charCount = 0;
      start = lastSpace + 1;
    }
    text++;
  }
  return lines;
}

void printWrappedTextUTF8(const char * text, int maxCharsPerLine, int startLine) {
  int currentLine = 0;
  int totalLines = calculateLines(text, maxCharsPerLine);
  int charCount = 0;
  const char * start = text;
  const char * lastSpace = text;

  int textHeight = totalLines * 20;  // Tổng chiều cao của tất cả các dòng văn bản (10 px mỗi dòng)
  int verticalOffset = (SCREEN_HEIGHT - textHeight) / 2;  // Căn giữa theo chiều dọc

  int y = verticalOffset + 20; // Vị trí Y ban đầu, cộng với khoảng cách căn giữa

  while ( * text) {
    if ( * text == ' ' || * text == '\t' || * text == '\n' || * (text + 1) == '\0') {
      lastSpace = text;
    }

    if (!isUTF8ContinuationByte( * text)) {
      charCount++;
    }

    if (charCount >= maxCharsPerLine && lastSpace != text) {
      if (currentLine >= startLine) {
        int lineWidth = u8g2.getStrWidth(String(start).substring(0, lastSpace - start).c_str());
        int horizontalOffset = (SCREEN_WIDTH - lineWidth) / 2;  // Căn giữa theo chiều ngang
        u8g2.drawUTF8(horizontalOffset, y, String(start).substring(0, lastSpace - start).c_str());
        y += 20;
      }
      start = lastSpace + 1;
      charCount = 0;
      currentLine++;
    }

    text++;
  }

  if (currentLine >= startLine) {
    int lineWidth = u8g2.getStrWidth(String(start).substring(0, text - start).c_str());
    int horizontalOffset = (SCREEN_WIDTH - lineWidth) / 2;  // Căn giữa theo chiều ngang
    u8g2.drawUTF8(horizontalOffset, y, String(start).substring(0, text - start).c_str());
  }
}

void printAndScroll(const char * text, int maxCharsPerLine, bool isParallel) {
  int totalLines = calculateLines(text, maxCharsPerLine);
  int scrollIndex = 0;
  do {
    unsigned long currentMillis = millis();
    if (currentMillis - lastScroll >= SCROLL_SPEED) {
      lastScroll = currentMillis;
      u8g2.clearBuffer();

      drawBackground(); // Draw background
      printWrappedTextUTF8(text, maxCharsPerLine, scrollIndex); // Căn giữa văn bản
      u8g2.sendBuffer();
      scrollIndex++;
      delay(100);
    }
  } while (scrollIndex < totalLines - 3);
}


void independentScroll(const char * text, int maxCharsPerLine, unsigned long currentMillis) {
  int totalLines = calculateLines(text, maxCharsPerLine);
  int scrollIndex = 0;

  if (currentMillis - lastScroll >= SCROLL_SPEED) {
    lastScroll = currentMillis;
    u8g2.clearBuffer();
    drawBackground(); // Draw background
    printWrappedTextUTF8(text, maxCharsPerLine, scrollIndex);
    u8g2.sendBuffer();
    scrollIndex++;
  }

  if (scrollIndex >= totalLines - 3) {
    return;
  }
}

void clearDisplay() {
  u8g2.clearBuffer();
  lastScroll = 0;
}

void displaySetup() {
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_t_vietnamese1); // Set a smaller font
}

void drawBackground() {
  u8g2.drawFrame(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT); // Draw a border around the screen
  u8g2.drawLine(0, 10, SCREEN_WIDTH, 10); // Example of a header line
  u8g2.setFont(u8g2_font_6x10_tf); // Set a smaller font for header or footer
  u8g2.drawStr(5, 8, "       STATUS"); // Example header text
  u8g2.setFont(u8g2_font_unifont_t_vietnamese1);
}


void drawBattery(const uint8_t* batteryIcon) {
  u8g2.clearBuffer();
  u8g2.drawBitmap(0,0 , 128/8, 64, batteryIcon);
  u8g2.sendBuffer();
}

