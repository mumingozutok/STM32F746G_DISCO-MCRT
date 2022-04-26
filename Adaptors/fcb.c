/*
 * fcb.c
 *
 *  Created on: Nov 25, 2021
 *      Author: mg
 */

#include "stdint.h"
#include "fcb.h"

#include "stm32f7xx_hal.h"

static uint8_t check_for_available_memory(uint32_t start_addr, uint32_t datalen, uint8_t mem_id){
	uint32_t mem_start, mem_end, mem_size;
	get_flash_memory_info(&mem_start, &mem_size, mem_id);
	mem_end = mem_start+ mem_size;

	if(mem_end < start_addr+FLASHBLOCK_HEADER_SIZE+datalen){
		return 0; //no available memory
	}

	return 1;
}

static uint32_t write_flash_block(uint32_t start_addr, uint8_t* data, uint32_t datalen, uint8_t mem_id)
{
	//first control for the available space if there is no space then return error
	if(check_for_available_memory(start_addr, datalen, mem_id) == 1){
		Flash_Block fb = {.block_start_marker1= BLOCK_START_MAGIC1, .length = datalen};

		//write header
		write_to_flash((uint8_t*) (&fb), start_addr, FLASHBLOCK_HEADER_SIZE);

		//write data
		write_to_flash(data, start_addr+FLASHBLOCK_HEADER_SIZE, datalen);

		return start_addr+FLASHBLOCK_HEADER_SIZE+datalen; //return the finish address of the block
	}

	return 0; //no available memory
}

//search all flash to find the end of record
//last record: when doing search if you need the address of the last record function returns this value from the last_record
static Get_Current_Addr_Data get_current_address(uint32_t start_address, uint32_t memory_size){
	uint32_t* pflash_data;
	uint32_t flashptr, inc = 1;
	Flash_Block* temp_fb = (Flash_Block*) start_address;
	Get_Current_Addr_Data ret;

	ret.last_record_fb = temp_fb; //If there is no record at flash to initiate last record

	for(flashptr = start_address;flashptr<start_address+memory_size;flashptr+=inc)
	{
		temp_fb = (Flash_Block*) flashptr;

		if(temp_fb->block_start_marker1 ==BLOCK_START_MAGIC1){
			inc = temp_fb->length + FLASHBLOCK_HEADER_SIZE;
			ret.last_record_fb = temp_fb;
		}
		else{
			break;
		}
	}
	ret.flash_pointer = flashptr;

	return ret;
}

//mem_id: 0 -> Function Blocks
//mem_id: 1 -> Static Parameters
//mem_id: 2 -> Dynamic Parameters
//mem_id: 3 -> Circular FIFO (Data Storage)
uint32_t write_flash_data(uint8_t* data, uint32_t length, uint8_t mem_id){
	uint32_t mem_start_address, mem_size;

	//Its very important length to be multiply of 4
	if(length%4 != 0){
		length = length + 4 - (length %4);
	}

	//get memory properties
	get_flash_memory_info(&mem_start_address, &mem_size, mem_id);

	//get current flash pointer
	Get_Current_Addr_Data lastrecord_info = get_current_address(mem_start_address, mem_size);

	if(check_for_available_memory(lastrecord_info.flash_pointer, length, mem_id) == 0){
		erase_flash(mem_start_address, mem_id);
		lastrecord_info.flash_pointer = mem_start_address;
	}

	//write data to flash and return current flash pointer address
	return write_flash_block(lastrecord_info.flash_pointer, data, length, mem_id);
}

Flash_Block* get_flash_data(uint8_t mem_id){
	uint32_t mem_start_address, mem_size;

	//get memory properties
	get_flash_memory_info(&mem_start_address, &mem_size, mem_id);

	//get current flash pointer
	Get_Current_Addr_Data lastrecord_info = get_current_address(mem_start_address, mem_size);

	return lastrecord_info.last_record_fb;
}

//---------------------------STM32 FLASH Driver Functions-----------------------------------------
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
	uint8_t ret;
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
	return ret;
}

uint8_t erase_flash(uint32_t start_addr, uint8_t mem_id)
{
	uint8_t ret = 0;
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
		ret = 0;
	}

	else ret = 1;

	return ret;
}

uint8_t check_flash_memory(){
	uint32_t mem_start_address = 0, mem_size = 0;

	//get memory properties
	get_flash_memory_info(&mem_start_address, &mem_size, APPLICATION_MEMORY_ID);

	if(mem_start_address == 0)
		return RET_APP_ERROR_FLASHHAL;

	return 0;
}

#ifdef FCB_TEST
//--------------------TEST FUNCTIONS----------------------
void test_write_block(){

#define FLASH_TEST_DATA_LEN 12  //be careful to make this data multiply of 4

	uint8_t data[FLASH_TEST_DATA_LEN];
	uint32_t ret_addr = 0, mem_start_addr = 0, mem_size = 0;

	for(uint16_t i=0;i<FLASH_TEST_DATA_LEN;i++) data[i] = i;

	ret_addr = write_flash_data(data, FLASH_TEST_DATA_LEN);

	ret_addr = write_flash_data(data, FLASH_TEST_DATA_LEN);

	for(uint16_t i=0;i<FLASH_TEST_DATA_LEN;i++) data[i] = 80+i;

	ret_addr = write_flash_data(data, FLASH_TEST_DATA_LEN);

	Flash_Block* fb = get_flash_data();
}
#endif
