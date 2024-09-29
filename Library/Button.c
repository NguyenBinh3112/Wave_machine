#include "Button.h"

__weak void button_pressing_callback()
{
	
}

__weak void button_press_short_callback()
{
}
	
__weak void button_release_callback()
{
	
}
	
__weak void button_press_long_callback()
{
	
}


// ham xu li button
void button_handle(Button_handle *button)
{
	//-------------------------loc nhieu -------------------------
	uint8_t sta = HAL_GPIO_ReadPin(button->GPIOx, button->GPIO_Pin);
	if(sta != button->btn_filter)
	{
		button->btn_filter = sta;
		button->is_debouncing = 1;
		button->time_debouncing = HAL_GetTick();
	}
	// ------------- tin hieu xac lap ------------------------//
	if(button->is_debouncing && (HAL_GetTick() - button->time_debouncing) >= 15)
	{
		button->btn_current = button->btn_filter;
		button->is_debouncing = 0;
	}
	//----------------------- xu ly -------------------------//
	if(button->btn_current != button->btn_last)
	{
		if(button->btn_current == 0) // nhan xuong
		{
			button->is_pressing = 1;
			button_pressing_callback();
			button->time_start_pressing = HAL_GetTick();
		}
		else
		{
			if((HAL_GetTick() - button->time_start_pressing) <= 1000)
			{
				button_press_short_callback();
			}
			button_release_callback();
			button->is_pressing = 0;
		}
		button->btn_last = button->btn_current;
	}
	// nhan phim lau
	if(button->is_pressing && (HAL_GetTick() - button->time_start_pressing) >=1000)
	{
		button_press_long_callback();
		button->is_pressing = 0;
	}
}

// ham khoi tao button
void Button_Init(Button_handle *button, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	button->GPIOx = GPIOx;
	button->GPIO_Pin = GPIO_Pin;
	button->btn_current = 1;
	button->btn_last = 1;
	button->btn_filter = 1;
}	

