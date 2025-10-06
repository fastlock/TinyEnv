#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

#include "Oled.h"
#include "cmsis_os.h"

class DisplayTask {
public:
    explicit DisplayTask(OLED* oled);
    void start();

private:

    OLED* oledPtr;
    
    static void run(void* params);
    osThreadId_t taskHandle;
};

#endif
