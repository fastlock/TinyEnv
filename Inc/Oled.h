#ifndef OLED_H
#define OLED_H

#include <cstdint>
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"  

class OLED {
public:
    OLED(I2C_HandleTypeDef* i2cHandle) : i2cDevicePtr(i2cHandle) {};
   
    void init();
    void fill(uint8_t pattern);
    void setCursor(uint8_t x, uint8_t y);
    void drawChar(uint8_t x, uint8_t y, char c);
    void print(uint8_t x, uint8_t y, const char* str);
    void clearArea(uint8_t x, uint8_t y, uint8_t width, uint8_t height);

private:
    void sendCommand(uint8_t command);
    void sendData(uint8_t data);

    I2C_HandleTypeDef* i2cDevicePtr; // Puntatore all'handle I2C

    static const int OLED_WIDTH = 128;
    static const int OLED_HEIGHT = 64;
    static const int OLED_ADDR = 0x3C << 1 ;

    // font 5x7: ogni carattere = 5 colonne (più 1 colonna vuota come spazio)
    // main.h oppure in un file font.h incluso in main.c
    static const uint8_t font5x7[96][5];
};

#endif // OLED_H
