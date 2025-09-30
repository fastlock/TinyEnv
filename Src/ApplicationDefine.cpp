#include "ApplicationDefine.h"
#include "DisplayTask.h"
#include "Oled.h"
#include "BlinkTask.h"
//#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

extern "C" {
   extern GPIO_InitTypeDef GPIO_InitStruct;
}

extern "C" void ApplicationDefine(void) {
    // Inizializza l'OLED e avvia il task di gestione del display
    static OLED oled(&hi2c1);
    static DisplayTask displayTask(&oled);
    static BlinkTask blink(GPIOA, GPIO_PIN_5);

    blink.start();
    displayTask.start();

    // Avvia qui altri task se servono

}

