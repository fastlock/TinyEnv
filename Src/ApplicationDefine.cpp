#include "ApplicationDefine.h"
#include "DisplayTask.h"
#include "Oled.h"
#include "BlinkTask.h"
#include "SensorTask.h"
#include "queue.h"
#include "AppConst.h"
#include "semphr.h"

//#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

extern "C" {
   extern GPIO_InitTypeDef GPIO_InitStruct;
}

static QueueHandle_t sensorDataQueue = nullptr;
static SemaphoreHandle_t i2cMutex = nullptr;

extern "C" void ApplicationDefine(void) {
    i2cMutex = xSemaphoreCreateMutex();
    
    sensorDataQueue = xQueueCreate(2, sizeof(SensorData_t));

    if (sensorDataQueue == NULL) {
        // Errore creazione queue
    }

    // Inizializza l'OLED e avvia il task di gestione del display
    static OLED oled(&hi2c1);
    
    static BlinkTask blink(GPIOA, GPIO_PIN_5);
    static SensorTask sensorTask(&hi2c1,sensorDataQueue,i2cMutex);
    static DisplayTask displayTask(&oled,sensorDataQueue,i2cMutex);

    
    blink.start();
    sensorTask.start();
    displayTask.start();


}

