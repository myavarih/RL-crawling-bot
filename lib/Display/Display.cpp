#include "Display.h"

Display::Display() : cursorX(0), cursorY(0)
{
    oled = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
}

Display::~Display()
{
    delete oled;
}

void Display::begin()
{
    if (!oled->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }
    oled->clearDisplay();
    oled->setTextSize(1);
    oled->setTextColor(SSD1306_WHITE);
    oled->setCursor(0, 0);
    oled->display();
}

void Display::clear()
{
    oled->clearDisplay();
    cursorX = 0;
    cursorY = 0;
    oled->setCursor(0, 0);
}

void Display::print(const char *text, uint8_t x, uint8_t y)
{
    if (x != 0 || y != 0)
    {
        oled->setCursor(x, y);
        cursorX = x;
        cursorY = y;
    }
    else
    {
        oled->setCursor(cursorX, cursorY);
    }
    oled->print(text);
    // Update cursor position after printing
    cursorX = oled->getCursorX();
    cursorY = oled->getCursorY();
}

void Display::print(const String &text, uint8_t x, uint8_t y)
{
    print(text.c_str(), x, y);
}

void Display::print(int value, uint8_t x, uint8_t y)
{
    if (x != 0 || y != 0)
    {
        oled->setCursor(x, y);
        cursorX = x;
        cursorY = y;
    }
    else
    {
        oled->setCursor(cursorX, cursorY);
    }
    oled->print(value);
    // Update cursor position after printing
    cursorX = oled->getCursorX();
    cursorY = oled->getCursorY();
}

void Display::println(const char *text, uint8_t x, uint8_t y)
{
    if (x != 0 || y != 0)
    {
        oled->setCursor(x, y);
    }
    else
    {
        oled->setCursor(cursorX, cursorY);
    }
    oled->println(text);
    cursorY += 8; // Move cursor down by one line
    oled->display();
}

void Display::setCursor(uint8_t x, uint8_t y)
{
    cursorX = x;
    cursorY = y;
    oled->setCursor(x, y);
}

void Display::setTextSize(uint8_t size)
{
    oled->setTextSize(size);
}

void Display::display()
{
    oled->display();
}

void Display::refresh()
{
    oled->display();
}

void Display::drawProgressBar(uint8_t percentage)
{
    clear();
    oled->setCursor(0, 0);
    oled->print("OTA Progress:");

    // Draw progress bar
    uint8_t barWidth = 100;
    uint8_t barHeight = 10;
    uint8_t barX = 14;
    uint8_t barY = 20;

    oled->drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);
    uint8_t fillWidth = (barWidth - 2) * percentage / 100;
    oled->fillRect(barX + 1, barY + 1, fillWidth, barHeight - 2, SSD1306_WHITE);

    // Display percentage
    oled->setCursor(50, 35);
    oled->print(percentage);
    oled->print("%");
    oled->display();
}
