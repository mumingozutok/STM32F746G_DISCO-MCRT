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

//sdcard
#include "fatfs.h"

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

//These should be global for dealing with memory corruption
FIL fp;
FATFS fs;
uint8_t excel_data_counter = 1;

uint8_t SDCard_WriteFile(uint8_t file_id, uint32_t* data, uint32_t datalen, uint32_t* free_space){
	//FIL fp;
	FRESULT ret;
	//FATFS fs;
	uint32_t totalSpace, freeSpace;
	DWORD fre_clust;
	FATFS *pfs;
	uint32_t bw;
	uint8_t error;

	char str[10];
	char col_sep = ',';
	char row_sep = 0x0A;

	ret = f_mount(&fs, "", 0);
	if (ret != FR_OK) {
		_Error_Handler(__FILE__, __LINE__);
		return 1;
	}
	//csv recording???
	ret = f_open(&fp, "file.csv", FA_OPEN_APPEND | FA_WRITE | FA_READ);
	if (ret != FR_OK) {
		_Error_Handler(__FILE__, __LINE__);
		return 1;
	}

	/* Check freeSpace space */
	ret = f_getfree("", &fre_clust, &pfs);
	if (ret != FR_OK) {
		_Error_Handler(__FILE__, __LINE__);
		return 1;
	}

	totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);

	*free_space = freeSpace; //return the free space of the recording medium

	/* free space is less than 1kb */
	if(freeSpace < 1) return 1;

	for(uint32_t i=0;i<datalen;i++){
		itoa(data[i], str, 10); //convert data to string
		if(f_puts(str, &fp) < 0) {
			error = 1; //there is a write error then break the loop, return error
			break;
		}

		if(f_putc(row_sep, &fp) < 0){
						error = 1; //there is a write error then break the loop, return error
						break;
		}

		/*
		if(excel_data_counter == 10){
			excel_data_counter = 1;
			f_putc(row_sep, &fp);//for new row
		}
		else{
			excel_data_counter++;
			//for new coloumb
			if(f_putc(col_sep, &fp) < 0){
				error = 1; //there is a write error then break the loop, return error
				break;
			}
		}*/
	}

	if(error > 0){
		return 1;
	}

	/* Close file */
	ret = f_close(&fp);

	if (ret != FR_OK) {
		_Error_Handler(__FILE__, __LINE__);
		return 1;
	}

	return 0;
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


//---------------------Flash functions---------------------------------------
#define STM32F467_FLASH_OK

#ifdef STM32F467_FLASH_OK

#define ADDR_FLASH_SECTOR_0 ((uint32_t)0x08000000) // 32 Kbytes
#define ADDR_FLASH_SECTOR_1 ((uint32_t)0x08008000) // 32 Kbytes
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08010000) // 32 Kbytes
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x08018000) // 32 Kbytes
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08020000) // 128 Kbytes
#define ADDR_FLASH_SECTOR_5 ((uint32_t)0x08040000) // 256 Kbytes
#define ADDR_FLASH_SECTOR_6 ((uint32_t)0x08080000) // 256 Kbytes
#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x080C0000) // 256 Kbytes

#define FLASH_MEMORY_SIZE_32K (32*1024)
#define FLASH_MEMORY_SIZE_128K (128*1024)
#define FLASH_MEMORY_SIZE_256K (256*1024)


#endif

//mem_id: 0 -> Function Blocks
//mem_id: 1 -> Static Parameters
//mem_id: 2 -> Dynamic Parameters
//mem_id: 3 -> Circular FIFO (Data Storage)
void get_flash_memory_info(uint32_t* start_addr, uint32_t* size, uint8_t mem_id){
	switch(mem_id){
	case 0:
		*start_addr = ADDR_FLASH_SECTOR_7;
		*size = FLASH_MEMORY_SIZE_128K;
		break;
	case 1:
		*start_addr = ADDR_FLASH_SECTOR_6;
		*size = FLASH_MEMORY_SIZE_256K;
		break;
	case 2:
		*start_addr = ADDR_FLASH_SECTOR_5;
		*size = FLASH_MEMORY_SIZE_256K;
		break;
	case 3:
		*start_addr = ADDR_FLASH_SECTOR_4;
		*size = FLASH_MEMORY_SIZE_256K;
		break;
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
			EraseInitStruct.Sector = FLASH_SECTOR_4;
			break;
		case 1:
			EraseInitStruct.Sector = FLASH_SECTOR_5;
			break;
		case 2:
			EraseInitStruct.Sector = FLASH_SECTOR_6;
			break;
		case 3:
			EraseInitStruct.Sector = FLASH_SECTOR_7;
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

void Display_Image(int32_t startX, int32_t startY,
											int32_t width, int32_t height,
												int32_t attr, int32_t val){
	//define the image table enties
	uint16_t* image_table[] = {
			image_ok,
			image_warning,
			image_error,
			image_techsafe_logo
	};													

	if(val >= sizeof(image_table)) return;

	//First clear the screen part
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(startX, startY, width, height);

	//Then draw the bmp to the given coordinates
	GUI_Disbitmap(startX, startY, width, height, image_table[val]);
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

void initiate_runtime()
{
	  init_comm_data_service();
	  init_comm_timing_service();
	  initiate_input_channels();
	  initiate_output_channels();
	  initate_analog_channels();
	  init_lcd_display();
}


