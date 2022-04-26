/*
 * adaptor.c
 *
 *  Created on: Nov 8, 2021
 *      Author: mg
 */

#include "stm32f7xx_hal.h"
#include "stm32746g_discovery_lcd.h"
#include "stdlib.h"

//images
#include "image_ok.h"
#include "image_error.h"
#include "image_warning1.h"
#include "image_techsafe_logo_color.h"
#include "bmp_parser.h"

extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc3;

volatile uint8_t SDCard_FatFsCnt = 0;
volatile uint8_t SDCard_Timer1, SDCard_Timer2;

typedef struct S_Adaptation{
	TIM_HandleTypeDef* comm_htim;
	UART_HandleTypeDef* comm_huart;

} runtime_adaptor;

typedef struct S_Digital_Channel{
	uint32_t port;
	uint32_t pin;
} Digital_Channel;

typedef struct S_ADC_Info{
	ADC_HandleTypeDef* adc;
	uint8_t ready_flag;
	uint16_t data[16];
	uint8_t ch_count;
} ADC_Info;

typedef struct S_Analog_Input_Channel{
	uint16_t data;
	ADC_Info* ai; //hold the information of the adc peripheral associated to this channel
} Analog_Input_Channel;


static ADC_Info adc_info = {.adc = &hadc3, .ready_flag = 1, .ch_count = 2};

#define DIGITAL_INPUT_CH_COUNT 8
#define DIGITAL_OUTPUT_CH_COUNT 4

static Digital_Channel inputChannel[8];
static Digital_Channel outputChannel[4];

#define ANALOG_INPUT_CH_COUNT 2
static Analog_Input_Channel analog_input_channel[ANALOG_INPUT_CH_COUNT];

void initiate_input_channels(){
	inputChannel[0].port = GPIOF;
	inputChannel[0].pin = GPIO_PIN_9;

	inputChannel[1].port = GPIOF;
	inputChannel[1].pin = GPIO_PIN_8;

	inputChannel[2].port = GPIOF;
	inputChannel[2].pin = GPIO_PIN_7;

	inputChannel[3].port = GPIOF;
	inputChannel[3].pin = GPIO_PIN_6;

	inputChannel[4].port = GPIOB;
	inputChannel[4].pin = GPIO_PIN_8;

	inputChannel[5].port = GPIOB;
	inputChannel[5].pin = GPIO_PIN_9;

	inputChannel[6].port = GPIOH;
	inputChannel[6].pin = GPIO_PIN_6;

	inputChannel[7].port = GPIOI;
	inputChannel[7].pin = GPIO_PIN_0;
}

void initiate_output_channels(){
	outputChannel[0].port = GPIOA;
	outputChannel[0].pin = GPIO_PIN_8;

	outputChannel[1].port = GPIOA;
	outputChannel[1].pin = GPIO_PIN_15;

	outputChannel[2].port = GPIOI;
	outputChannel[2].pin = GPIO_PIN_2;

	outputChannel[3].port = GPIOG;
	outputChannel[3].pin = GPIO_PIN_7;

	for(uint8_t i = 0; i<4;i++){
		hal_gpio_write_pin(i,0);
	}
}

void initate_analog_channels(){
	analog_input_channel[0].ai = &adc_info;
	analog_input_channel[0].data = 0;

	analog_input_channel[1].ai = &adc_info;
	analog_input_channel[1].data = 0;

}

runtime_adaptor ra = {.comm_htim = &htim7,
						.comm_huart = &huart1};

uint8_t uart_rx_data;

//Leds are connected to: PF0-PF2-PF13
//Please write down GPIO output function in your hardware
void hal_gpio_write_pin(uint16_t chNum, uint8_t value){
	if(chNum < DIGITAL_OUTPUT_CH_COUNT)
		HAL_GPIO_WritePin(outputChannel[chNum].port, outputChannel[chNum].pin, value);
}

uint8_t  hal_gpio_read_pin(uint32_t chNum){
	//return values >= 2, depicts error
	if(chNum < DIGITAL_INPUT_CH_COUNT)
		return HAL_GPIO_ReadPin(inputChannel[chNum].port, inputChannel[chNum].pin);
	return 0;
}

void reset_all_output_ch(){
	uint32_t i;
	for(i = 0; i<DIGITAL_OUTPUT_CH_COUNT; i++){
		hal_gpio_write_pin(i, 0);
	}
}

//------------------ADC readings with DMA---------------------------------------
uint32_t counter_hal_dma = 0, counter_dma_callback = 0;

uint32_t hal_read_analog_ch(uint32_t chNum){
	Analog_Input_Channel* ch = &analog_input_channel[chNum];
	ADC_Info* ai = ch->ai;
	if(ai->ready_flag == 1){ //check for the conv finish
		//Start DMA controlled ADC single convertion
		//this adc is working on two channels IN3 and IN10
		//so there will be two channel value after one conversion
		ai->ready_flag = 0; //disable the flag to notify there is an ongoing process
		HAL_ADC_Start_DMA(ai->adc, (uint32_t*) &ai->data, ai->ch_count);
		counter_hal_dma++;
	}

	return analog_input_channel[chNum].data;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	Analog_Input_Channel* ch;
	ADC_Info* ai;
	for(uint8_t i=0;i<ANALOG_INPUT_CH_COUNT;i++){
		ch = &analog_input_channel[i];
		ai = ch->ai;
		if(hadc == ai->adc){
			ch->data = ai->data[i];
			ai->ready_flag = 1;
			counter_dma_callback++;
		}
	}
}
//-------------------------------------------------------------------------------

//Please write down "get system tick" function in your hardware
uint32_t hal_get_tick(){
	return HAL_GetTick();
}

void /*__attribute__((weak))*/ hal_init_tick(){
	HAL_InitTick(0);
}

//Communication Channel Adaptation

//Please write down functions for your communication channel
//And put this function right after your initialisations
void init_comm_data_service(){
	HAL_UART_Receive_IT(ra.comm_huart, &uart_rx_data, 1);
}

//Please write down functions for communication timing services
//And put this function right after your initialisations
void init_comm_timing_service(){
	HAL_TIM_Base_Start_IT(ra.comm_htim);
	stop_comm_timer(ra.comm_htim);
}

void start_comm_timer(TIM_HandleTypeDef* htim){
	htim->Instance->CR1 &= ~0x01; //Stop Timer
	htim->Instance->CNT = 0; //Reset Counter
	htim->Instance->CR1 |= 0x01; //Start Timer
}

void stop_comm_timer(TIM_HandleTypeDef* htim){
	htim->Instance->CR1 &= ~0x01; //Stop Timer
	htim->Instance->CNT = 0; //Reset Counter
}


//sdcard functions
void SDTimer_Handler(void)
{
  if(SDCard_Timer1 > 0)
	  SDCard_Timer1--;

  if(SDCard_Timer2 > 0)
	  SDCard_Timer2--;
}

/*
 * filename: filename
 * buf: pointer to the data to be written
 * btw: number of bytes to write
 * bw: pointer to the variable to return number of bytes
 */

void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	/* USER CODE END Error_Handler_Debug */
}



//Callbacks
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(huart, &uart_rx_data, 1);
	start_comm_timer(ra.comm_htim);

	//When new data received copy this data to the runtime buffers
	Runtime_CommDataService_NewData_Received(0, &uart_rx_data, 1);
/*
	//Call user callback
	User_Callback_Function* ucf = get_ucf();
	if(ucf[MODBUS_UART_CALLBACK_FUNCTION_SLOT].f != 0){
		ucf[MODBUS_UART_CALLBACK_FUNCTION_SLOT].f(&uart_rx_data);
	}
	*/
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	  if (htim->Instance == TIM6) {
		  SDCard_FatFsCnt++;
		  if(SDCard_FatFsCnt >= 10)
		  {
			  SDCard_FatFsCnt = 0;
			  SDTimer_Handler();
		  }

	    HAL_IncTick();
	  }

	  else if(htim == ra.comm_htim){
		stop_comm_timer(ra.comm_htim);

		//External trigger makes runtime to process data
		//This trigging is needed for Modbus (3.5 Char)
		Runtime_CommDataService_Process_DataBuffer(0);
	  }
}

//Modbus UART Transmit Functions
void /*__attribute__((weak))*/ hal_modbus_uart_tx(uint8_t* pData, uint16_t Size){
	HAL_UART_Transmit_IT(ra.comm_huart, pData, Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
}

//-----------------------UNIQUE ID-----------------------------------------
void get_uniqueid(uint8_t* id, uint16_t len){
	uint32_t* buf = id;
	buf[0] 	= (uint32_t) READ_REG(*((uint32_t *)UID_BASE));
	buf[1] = (uint32_t) READ_REG(*((uint32_t *)(UID_BASE + 0x04U)));
	buf[2] = (uint32_t) READ_REG(*((uint32_t *)(UID_BASE + 0x14U)));
}



//---------------------Encoder Function---------------------------------------------------
int8_t hal_get_encoder_value(uint8_t ch)
{
	if(ch==0){
		return (htim1.Instance->CNT>>2);
	}
	else{
		return 0;
	}
}


//---------------------Display FUnctions---------------------------------------------------

//Display Functions
void  Display_String(int32_t startX, int32_t startY,
												int32_t width, int32_t height,
													int32_t attr, char* str, uint16_t len){

	BSP_LCD_SetFont(&Font16);

	BSP_LCD_SetTextColor(LCD_COLOR_RED); //can gather from attr
	BSP_LCD_DisplayStringAt(startX, startY, str, LEFT_MODE);
}

void  Display_Number(int32_t startX, int32_t startY,
											int32_t width, int32_t height,
												int32_t attr, int32_t val){


	char str[16];
	itoa(val, str, 10);

	BSP_LCD_SetFont(&Font16);

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(startX, startY, width, height);

	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_DisplayStringAt(startX, startY, str, LEFT_MODE);
}

typedef struct{
	uint16_t* data;
	int32_t width, height;
}s_image_bmp;

void create_image(s_image_bmp* images, uint8_t index, uint32_t* data, uint16_t width, uint16_t height){
	images[index].data = data;
	images[index].width = width;
	images[index].height = height;
}

void Display_Image(int32_t startX, int32_t startY, int32_t width,
		int32_t height, int32_t attr, int32_t val) {
	s_image_bmp images[5];
	create_image(&images, 0, image_warning, 64, 64);
	create_image(&images, 1, image_ok, 64, 64);
	create_image(&images, 2, image_error, 64, 64);
	create_image(&images, 3, image_techsafe_logo, 181, 64);

	if (val >= 5)
		return;

	//First clear the screen part
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(startX, startY, width, height);

	//Then draw the bmp to the given coordinates
	GUI_Disbitmap(startX, startY, images[val].width, images[val].height, images[val].data);
}

void Display_Animated_Image(int32_t startX, int32_t startY,
											int32_t width, int32_t height,
												int32_t attr, int32_t image_index, int32_t image_set_index){
	//define the image table enties
	/*
	uint16_t *image_table[] = { image_techsafe_logo, image_ok, image_warning,
			image_error, image_hatox_logo };*/

	s_image_bmp* image_set;
	s_image_bmp image_set0[3];

	create_image(&image_set0, 0, image_warning, 64, 64);
	create_image(&image_set0, 1, image_ok, 64, 64);
	create_image(&image_set0, 2, image_error, 64, 64);

	switch(image_set_index){
	case 0:
		if(image_index >= 0 & image_index < 3){ //control the boundary of the set
			image_set = image_set0;
		}
		else {
			image_set = 0;
		}
		break;
	default:
		image_set = 0;
		break;
	}

	//error conditions
	if(image_set == 0) return;

	//First clear the screen part
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(startX, startY, width, height);

	//Then draw the bmp to the given coordinates
	GUI_Disbitmap(startX, startY, image_set[image_index].width, image_set[image_index].height, image_set[image_index].data);
}


//-------------------------------Analog Bar-------------------
uint8_t analogbar_draw_first_call = 0;

void Display_Clear_Area(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(x, y, w, h);
}

void Display_AnalogBar_PercText(uint16_t x, uint16_t y, uint8_t val_perc) {
	BSP_LCD_SetFont(&Font16); //width:11, height:16

	//Display_Clear_Area(x, y, 44, 16); //%100
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(x, y, 44, 16);

	Display_String(x, y, 11, 16, 0, "%", 1);
	Display_Number(x + 13, y, 33, 16, 0, val_perc);
}

void Display_AnalogBar_BoundingRect(uint16_t x, uint16_t y, uint16_t w,
		uint16_t h) {
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK); //can gather from attr
	BSP_LCD_DrawRect(x, y, w, h);
}

typedef struct
{
	uint16_t x_out, y_out, w_out, h_out;
	uint16_t x_inner, y_inner, w_inner, h_inner;
	uint16_t bar_x_margin;              // a
	uint16_t bar_width, bar_draw_width; // c
	uint16_t bar_mid;                   // b
	uint8_t attr;
	uint8_t max_levels;
	uint8_t val;
} S_Analog_Bar_Graph;

void Draw_AnalogBar(S_Analog_Bar_Graph *abg)
{
	uint16_t res;

	// calculate inner margins
	abg->h_inner = abg->h_out - 10;
	abg->y_inner = abg->y_out + 5;
	abg->w_inner = abg->w_out - 10;
	abg->x_inner = abg->x_out + 5;

	if(abg->attr == 0){ //horizontal
		abg->bar_width = abg->w_inner / abg->max_levels;
		//artık kalanı hesapla,
		res = abg->w_inner - (abg->bar_width * abg->max_levels);
		//  x marjini buna göre değiştir.
		abg->x_inner += res / 2;
	}

	else{ //vertical
		abg->bar_width = abg->h_inner / abg->max_levels;
		//artık kalanı hesapla,
		res = abg->h_inner - (abg->bar_width * abg->max_levels);
		//  y marjini buna göre değiştir.
		abg->y_inner += res / 2;
	}

	abg->bar_draw_width = abg->bar_width - abg->bar_width / 4;

	Display_Clear_Area(abg->x_out, abg->y_out, abg->w_out, abg->h_out);
	Display_AnalogBar_BoundingRect(abg->x_out, abg->y_out, abg->w_out, abg->h_out);


	uint8_t bar_count = 0;
	for (uint8_t i = 0; i < abg->val; i++)
	{
		if (bar_count < 3)
		{
			BSP_LCD_SetTextColor(LCD_COLOR_RED); // can gather from attr
		}

		else if (bar_count < 6)
		{
			BSP_LCD_SetTextColor(LCD_COLOR_YELLOW); // can gather from attr
		}

		else
		{
			BSP_LCD_SetTextColor(LCD_COLOR_GREEN); // can gather from attr
		}

		if(abg->attr == 0){ //horizontal
			BSP_LCD_FillRect(abg->x_inner + abg->bar_width * bar_count++, abg->y_inner, abg->bar_draw_width, abg->h_inner);
		}

		else{//vertical
			BSP_LCD_FillRect(abg->x_inner , abg->y_inner+ abg->bar_width * ((abg->max_levels-1)-bar_count++), abg->w_inner, abg->bar_draw_width);
		}

	}
}

void Display_AnalogBar(int32_t startX, int32_t startY, int32_t width,
		int32_t height, int32_t attr, int32_t val, int32_t max_levels)
{
	S_Analog_Bar_Graph abg = {.x_out = startX, .y_out = startY, .w_out = width, .h_out = height, .attr = attr, .val = val, .max_levels = max_levels};

	// val_max, val_min should come from the upper layers, 0->val_min, 4096->val_max
	int16_t val_max = 100, val_min = 0;

	// boundary control
	if (val > val_max)
		abg.val = val_max;
	if (val < val_min)
		abg.val = val_min;
	if (width < 25)
		abg.w_out = 25;
	if (height < 25)
		abg.h_out = 25;

	switch (analogbar_draw_first_call)
	{
	case 0: // first-draw init
		Display_Clear_Area(abg.x_out, abg.y_out, abg.w_out, abg.h_out);
		Display_AnalogBar_BoundingRect(abg.x_out, abg.y_out, abg.w_out, abg.h_out);
		Draw_AnalogBar(&abg);
		analogbar_draw_first_call = 1;
		break;

	case 1:
		Draw_AnalogBar(&abg);
		break;
	}
}

#define 	LCD_FOREGROUND_LAYER   0x0001
#define 	LCD_BACKGROUND_LAYER   0x0000
#define 	LCD_FRAME_BUFFER   ((uint32_t)0xC0000000)

void init_lcd_display(){
	HAL_Delay(1000);
	BSP_LCD_LayerRgb565Init( LTDC_ACTIVE_LAYER, LCD_FRAME_BUFFER);
	//BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, LCD_FRAME_BUFFER);
	BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_DisplayOn();
}

void Display_Clear()
{
	analogbar_draw_first_call = 1;
	BSP_LCD_Clear(LCD_COLOR_WHITE);
}

//Watch function
void hal_xfer_watch_data(uint8_t len, uint8_t* watch_data){
	hal_modbus_uart_tx(watch_data, len);
}

//interrupt management
void hal_disable_interrupts(){
	__disable_irq();
}

void hal_enable_interrupts(){
	__enable_irq();
}

//initialization and resets
void ecu_deployment_reset(){
	reset_all_output_ch();
	Display_Clear();
}

void initiate_runtime()
{
	  init_comm_data_service();
	  init_comm_timing_service();
	  initiate_input_channels();
	  initiate_output_channels();
	  initate_analog_channels();
	  init_lcd_display();
	  init_fatfs();
	  ecu_deployment_reset();
}


