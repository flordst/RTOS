/*
 * receive_data.c
 *
 *  Created on: Jul 19, 2024
 *      Author: Flordst
 */

#include "receive_data.h"
#include "main.h"
//Read_temparature
ADC_HandleTypeDef hadc1;

void DS18B20_Start(void) {

    HAL_GPIO_WritePin(Temp_GPIO_Port, Temp_Pin, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(Temp_GPIO_Port, Temp_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    DS18B20_Write(0xCC);
    DS18B20_Write(0x44);
}

float Temp_Read(void) {
    uint16_t rawTemperature;
    float temperature;


    DS18B20_Write(0xCC);
    DS18B20_Write(0xBE);


    rawTemperature = DS18B20_Read();
    rawTemperature |= ((uint16_t)DS18B20_Read()) << 8;

    temperature = (float)((int16_t)rawTemperature) / 16.0f;

    return temperature;
}


void DS18B20_Write(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(Temp_GPIO_Port, Temp_Pin, GPIO_PIN_RESET);
        if (byte & 0x01) {
            HAL_Delay(5);
            HAL_GPIO_WritePin(Temp_GPIO_Port, Temp_Pin, GPIO_PIN_SET);
            HAL_Delay(60);
        } else {
            HAL_Delay(60);
            HAL_GPIO_WritePin(Temp_GPIO_Port, Temp_Pin, GPIO_PIN_SET);
            HAL_Delay(5);
        }
        byte >>= 1;
    }
}


uint8_t DS18B20_Read(void) {
    uint8_t byte = 0;

    for (uint8_t i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(Temp_GPIO_Port, Temp_Pin, GPIO_PIN_RESET) ;
        HAL_Delay(2);
        HAL_GPIO_WritePin(Temp_GPIO_Port, Temp_Pin, GPIO_PIN_SET);
        HAL_Delay(10);
        byte >>= 1;
        if (HAL_GPIO_ReadPin(Temp_GPIO_Port, Temp_Pin) == GPIO_PIN_SET) {
            byte |= 0x80;
        }
        HAL_Delay(50);
    }

    return byte;
}

//read light sensor
uint8_t Light_Read(void) {

    uint8_t sensor_value = HAL_GPIO_ReadPin(Light_GPIO_Port, Light_Pin);

    return sensor_value;
}

//read Gas sensor

uint16_t Gas_Read(void) {
    uint16_t adc_value = 0;

    if (HAL_ADC_Start(&hadc1) == HAL_OK) {
        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
            adc_value = HAL_ADC_GetValue(&hadc1);
        }
    }

    return adc_value;
}

