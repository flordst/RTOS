/*
 * controller.c
 *
 *  Created on: Jul 19, 2024
 *      Author: Flordst
 */

#include "main.h"
#include "STM32F407_OLED_SSD1306_Driver.h"
void LED_Control(uint8_t state){
	if (state ==1)
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

}

void Buzz_Control(uint16_t value){
	uint16_t threshold=100;
	if (value>threshold)
		HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);

}







