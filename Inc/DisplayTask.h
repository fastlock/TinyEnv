#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

#include "Oled.h"
#include "cmsis_os.h"

class DisplayTask {
public:
    explicit DisplayTask(OLED* oled);
    void start();

private:
    static void run(void* params);
    OLED* oledPtr;
    osThreadId_t taskHandle;
};

#endif
