/*
 * fcb.h
 *
 *  Created on: Nov 25, 2021
 *      Author: mg
 */

#ifndef CORE_INC_FCB_H_
#define CORE_INC_FCB_H_

#define APPLICATION_MEMORY_ID 0

/*
 * 0xBABABABA
 * 0xDEDEDEDE
 * 0xXXXXXXXX - Length
 * data
 * data
 * data
 */
typedef struct S_Flash_Block{
	uint32_t block_start_marker1;

	uint32_t length;
	//data ....
}	Flash_Block;

typedef struct S_Get_Current_Addr_Data{
	Flash_Block* last_record_fb; //header of the last recorded block
	uint32_t flash_pointer; //current write pointer
} Get_Current_Addr_Data;

#define BLOCK_START_MAGIC1 0x01234567

#define FLASHBLOCK_HEADER_SIZE sizeof(Flash_Block)
#define RET_APP_ERROR_OVERFLOW 1
#define RET_APP_ERROR_ABNORMAL_DATA 2
#define RET_APP_ERROR_NORECORDING 3
#define RET_APP_ERROR_FLASHHAL 4

void test_write_block();
Flash_Block* get_flash_data(uint8_t mem_id);
uint32_t write_flash_data(uint8_t* data, uint32_t length, uint8_t mem_id);
uint8_t check_flash_memory();
uint8_t write_to_flash(uint8_t* p, uint32_t start_addr, uint16_t size);
uint8_t erase_flash(uint32_t start_addr, uint8_t mem_id);

#endif /* CORE_INC_FCB_H_ */
