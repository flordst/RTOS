/*
 * receive_data.h
 *
 *  Created on: Jul 19, 2024
 *      Author: admin
 */

#ifndef INC_RECEIVE_DATA_H_
#define INC_RECEIVE_DATA_H_
#include "stm32f4xx_hal.h"
//function of DS18B20
void DS18B20_Start(void);
void DS18B20_Write(uint8_t byte);
uint8_t DS18B20_Read(void);
float Temp_Read(void);

uint8_t Light_Read(void);

uint16_t Gas_Read(void);


#endif /* INC_RECEIVE_DATA_H_ */
