#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

void Encoder_Init(void);
void Encoder_Update(void);
void Encoder_GpioExtiCallback(uint16_t gpio_pin);

#endif /* ENCODER_H_ */
