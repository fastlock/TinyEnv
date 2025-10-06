#ifndef    ENCODER_H
#define    ENCODER_H

#include <cstdint>
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"  


class Encoder
{
private:
    TIM_HandleTypeDef* encTimPtr; // Puntatore all'handle del timer usato come encoder 
    uint16_t prev_position;
    uint16_t position;
    int16_t delta;
public:
    Encoder(TIM_HandleTypeDef* _encTim) : encTimPtr(_encTim) {};

    uint16_t getPosition() {
        position = __HAL_TIM_GET_COUNTER(encTimPtr) / 2;
        delta = (int16_t)(position - prev_position);
        prev_position = position;
        return position;
    }

    ~Encoder();
};

Encoder::Encoder(/* args */)
{
}

Encoder::~Encoder()
{
}

#endif  // ENCODER_H