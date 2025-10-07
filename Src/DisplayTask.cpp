#include "DisplayTask.h"
#include <stdio.h>  // Per snprintf
#include <cstring> // Per strlen
DisplayTask::DisplayTask(OLED* oled,QueueHandle_t queue,SemaphoreHandle_t mutex) : oledPtr(oled),_displayDataQueue(queue), _mutex(mutex),taskHandle(nullptr) {}

void DisplayTask::start() {
    osThreadAttr_t attrs = {
        .name = "DisplayTask",
        .stack_size = 4096,
        .priority = (osPriority_t) osPriorityNormal,
    };
    taskHandle = osThreadNew(DisplayTask::run, this, &attrs);
}

void DisplayTask::run(void* params) {
    DisplayTask* self = static_cast<DisplayTask*>(params);

    if (xSemaphoreTake(self->_mutex, pdMS_TO_TICKS(10000)) == pdTRUE) 
    {
        self->oledPtr->init();
        self->oledPtr->fill(0x00); // Pulisce il display
        osDelay(1000);
        self->oledPtr->drawBitmap(self->oledPtr->myBitmapallArray[0],0,0,128,64); // Disegna l'immagine della goccia
        osDelay(1000);
        self->oledPtr->fill(0x00); // Pulisce il display
        osDelay(1000);
        xSemaphoreGive(self->_mutex);
    }

    else 
    {
        // Gestione errore: non è stato possibile prendere il mutex
        // Qui potresti voler loggare un errore o prendere altre azioni
    }
    SensorData_t receivedData;

    while (true) {
        // Controlla se ci sono nuovi dati nella coda
        if (self->_displayDataQueue != nullptr) {
            if (xQueueReceive(self->_displayDataQueue, &receivedData, 0) == pdTRUE) 
            {
                if (xSemaphoreTake(self->_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) 
                {
                    // Aggiorna il display con i nuovi dati
                    char buffer[32];
                    snprintf(buffer, sizeof(buffer), "Temp: %.2f C", receivedData.temperature);
                    //self->oledPtr->clearArea(0, 0, 128, 16); // Pulisce la parte superiore del display
                    self->oledPtr->print(0, 0, buffer);
                    memset(buffer, 0, sizeof(buffer));
                    snprintf(buffer, sizeof(buffer), "Hum: %.2f RH", receivedData.humidity);
                    //self->oledPtr->clearArea(0, 16, 128, 16); // Pulisce la parte centrale del display
                    self->oledPtr->print(0, 2, buffer);
                    
                    xSemaphoreGive(self->_mutex);
                } 
                else 
                {
                    // Gestione errore: non è stato possibile prendere il mutex
                    // Qui potresti voler loggare un errore o prendere altre azioni
                }
            }
            else 
            {
            // Gestione errore: coda non inizializzata
            
            }
        }

        else {
            // Gestione errore: coda non inizializzata
            
        }
        
        osDelay(1000);
    }
}
