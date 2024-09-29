#ifndef BUTTON_H
#define BUTTON_H

#include "main.h"

typedef struct 
{
	GPIO_TypeDef *GPIOx; 
	uint16_t GPIO_Pin;
	
	uint8_t btn_current;
	uint8_t btn_last;
	uint8_t btn_filter;
	uint8_t is_debouncing;
	uint32_t time_debouncing;
	uint32_t time_start_pressing;
	uint8_t is_pressing;
	
}Button_handle;

void button_handle(Button_handle *button);
void Button_Init(Button_handle *button, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
#endif

