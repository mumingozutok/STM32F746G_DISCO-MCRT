/*
 * adaptor.c
 *
 *  Created on: Nov 8, 2021
 *      Author: mg
 */

#include "stm32f7xx_hal.h"
#include "stm32746g_discovery_lcd.h"

extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc3;

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

static Digital_Channel inputChannel[3];
static Digital_Channel outputChannel[4];

#define ANALOG_INPUT_CH_COUNT 2
static Analog_Input_Channel analog_input_channel[ANALOG_INPUT_CH_COUNT];

void initiate_input_channels(){
	inputChannel[0].port = GPIOB;
	inputChannel[0].pin = GPIO_PIN_9; //EXT3

	inputChannel[1].port = GPIOB;
	inputChannel[1].pin = GPIO_PIN_8; //EXT5

	inputChannel[2].port = GPIOF; //jOYSTİCK button
	inputChannel[2].pin = GPIO_PIN_9;
}

void initiate_output_channels(){
	outputChannel[0].port = GPIOI;
	outputChannel[0].pin = GPIO_PIN_2;

	outputChannel[1].port = GPIOA;
	outputChannel[1].pin = GPIO_PIN_15;

	outputChannel[2].port = GPIOA;
	outputChannel[2].pin = GPIO_PIN_8;

	outputChannel[3].port = GPIOB;
	outputChannel[3].pin = GPIO_PIN_15;

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
	HAL_GPIO_WritePin(outputChannel[chNum].port, outputChannel[chNum].pin, value);
}

uint8_t  hal_gpio_read_pin(uint32_t chNum){
	//return values >= 2, depicts error
	return HAL_GPIO_ReadPin(inputChannel[chNum].port, inputChannel[chNum].pin);

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


//---------------------Flash functions---------------------------------------
#define STM32F467_FLASH_OK

#ifdef STM32F467_FLASH_OK

#define ADDR_FLASH_SECTOR_23     ((uint32_t)0x081E0000)
#define ADDR_FLASH_SECTOR_22     ((uint32_t)0x081C0000)
#define ADDR_FLASH_SECTOR_21     ((uint32_t)0x081A0000)
#define ADDR_FLASH_SECTOR_20     ((uint32_t)0x08180000)

#define FLASH_MEMORY_SIZE (128*1024)

#endif

//mem_id: 0 -> Function Blocks
//mem_id: 1 -> Static Parameters
//mem_id: 2 -> Dynamic Parameters
//mem_id: 3 -> Circular FIFO (Data Storage)
void get_flash_memory_info(uint32_t* start_addr, uint32_t* size, uint8_t mem_id){
	switch(mem_id){
	/*case 0:
		*start_addr = ADDR_FLASH_SECTOR_23;
		*size = FLASH_MEMORY_SIZE;
		break;
	case 1:
		*start_addr = ADDR_FLASH_SECTOR_22;
		*size = FLASH_MEMORY_SIZE;
		break;
	case 2:
		*start_addr = ADDR_FLASH_SECTOR_21;
		*size = FLASH_MEMORY_SIZE;
		break;
	case 3:
		*start_addr = ADDR_FLASH_SECTOR_20;
		*size = FLASH_MEMORY_SIZE;
		break;*/
	default:
		*start_addr = 0;
		*size = 0;
	}
}

uint8_t write_to_flash(uint8_t* p, uint32_t start_addr, uint16_t size)
{
	uint8_t ret = 0;
#ifdef STM32F467_FLASH_OK
	uint16_t i;
	uint32_t data;

	HAL_FLASH_Unlock();

	for (i = 0; i < size; i+=4) {
                data = *(uint32_t*)(p+i);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_addr + i, data) == HAL_OK) ret = 1;
		else {
			ret = 0;
			break;
		}
	}

	HAL_FLASH_Lock();
#endif
	return 0;
}

uint8_t erase_flash(uint32_t start_addr, uint8_t mem_id)
{
	uint8_t ret = 0;
#ifdef STM32F467_FLASH_OK
	uint32_t SectorError = 0;
    FLASH_EraseInitTypeDef EraseInitStruct;


	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS         ;
	EraseInitStruct.NbSectors = 1;
	switch (mem_id){
		case 0:
			EraseInitStruct.Sector = FLASH_SECTOR_8;
			break;
		case 1:
			EraseInitStruct.Sector = FLASH_SECTOR_22;
			break;
		case 2:
			EraseInitStruct.Sector = FLASH_SECTOR_21;
			break;
		case 3:
			EraseInitStruct.Sector = FLASH_SECTOR_20;
			break;
	}


	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{
		ret = 1;
	}

	else ret = 0;

#endif

	return ret;
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

void Display_Clear()
{
	  BSP_LCD_Clear(LCD_COLOR_WHITE);
}



//Watch function
void hal_xfer_watch_data(uint8_t len, uint8_t* watch_data){
	hal_modbus_uart_tx(watch_data, len);
}

void initiate_runtime()
{
	  init_comm_data_service();
	  init_comm_timing_service();
	  initiate_input_channels();
	  initiate_output_channels();
	  initate_analog_channels();
}
