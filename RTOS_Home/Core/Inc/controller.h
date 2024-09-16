/*
 * controller.h
 *
 *  Created on: Jul 19, 2024
 *      Author: admin
 */


#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_
#include <stdint.h>
#include "stm32f4xx_hal.h"


void LED_Control(uint8_t state);
void Buzz_Control(uint16_t value);
#endif /* INC_CONTROLLER_H_ */

