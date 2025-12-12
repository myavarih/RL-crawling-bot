#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class Display
{
public:
    Display();
    ~Display();
    void begin();
    void clear();
    void print(const char *text, uint8_t x = 0, uint8_t y = 0);
    void print(const String &text, uint8_t x = 0, uint8_t y = 0);
    void print(int value, uint8_t x = 0, uint8_t y = 0);
    void println(const char *text, uint8_t x = 0, uint8_t y = 0);
    void setCursor(uint8_t x, uint8_t y);
    void setTextSize(uint8_t size);
    void display();
    void refresh(); // Call oled->display() to update screen
    void drawProgressBar(uint8_t percentage);

private:
    Adafruit_SSD1306 *oled;
    uint8_t cursorX;
    uint8_t cursorY;

    static const uint8_t SCREEN_WIDTH = 128;
    static const uint8_t SCREEN_HEIGHT = 64;
    static const int8_t OLED_RESET = -1;
    static const uint8_t SCREEN_ADDRESS = 0x3C;
};

#endif // DISPLAY_H
