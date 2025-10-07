#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

#include "Oled.h"
#include "cmsis_os.h"
#include "AppConst.h"
#include "queue.h"
#include "semphr.h"

class DisplayTask {
public:
    explicit DisplayTask(OLED* oled,QueueHandle_t queue, SemaphoreHandle_t mutex);
    void start();

private:

    OLED* oledPtr;
    QueueHandle_t _displayDataQueue;
    SemaphoreHandle_t _mutex;
    static void run(void* params);
    osThreadId_t taskHandle;
};

#endif
