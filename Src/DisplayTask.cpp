#include "DisplayTask.h"
#include <stdio.h>  // Per snprintf
#include <cstring> // Per strlen
DisplayTask::DisplayTask(OLED* oled,QueueHandle_t queue) : oledPtr(oled),_displayDataQueue(queue), taskHandle(nullptr) {}

void DisplayTask::start() {
    osThreadAttr_t attrs = {
        .name = "DisplayTask",
        .stack_size = 4096,
        .priority = (osPriority_t) osPriorityAboveNormal,
    };
    taskHandle = osThreadNew(DisplayTask::run, this, &attrs);
}

void DisplayTask::run(void* params) {
    DisplayTask* self = static_cast<DisplayTask*>(params);
    self->oledPtr->init();
    self->_displayDataQueue = self->_displayDataQueue; // Assicurati che la coda sia inizializzata
    SensorData_t receivedData;

    while (true) {
        // Controlla se ci sono nuovi dati nella coda
        if (self->_displayDataQueue != nullptr) {
            if (xQueueReceive(self->_displayDataQueue, &receivedData, 0) == pdTRUE) 
            {
                // Aggiorna il display con i nuovi dati
                char buffer[32];
                snprintf(buffer, sizeof(buffer), "Temp: %.2f C", receivedData.temperature);
                self->oledPtr->clearArea(0, 0, 128, 16); // Pulisce la parte superiore del display
                self->oledPtr->print(0, 0, buffer);
                
                snprintf(buffer, sizeof(buffer), "Hum: %.2f %%", receivedData.humidity);
                self->oledPtr->clearArea(0, 16, 128, 16); // Pulisce la parte centrale del display
                self->oledPtr->print(0, 16, buffer);
            }
            else {
            // Gestione errore: coda non inizializzata
            self->oledPtr->fill(0x00);
            self->oledPtr->print(0, 0, " NADA QUEUE");
            self->oledPtr->print(0, 2, " Display Task OK!");
            }
        }

        else {
            // Gestione errore: coda non inizializzata
            self->oledPtr->fill(0x00);
            self->oledPtr->print(0, 0, " FREERTOS Test");
            self->oledPtr->print(0, 2, " Display Task OK!");
        }
        
        osDelay(1000);
    }
}
