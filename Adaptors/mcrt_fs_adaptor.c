/*
 * mcrt_fs_adaptor.c
 *
 *  Created on: Apr 21, 2022
 *      Author: mg
 */

#include "stdint.h"
#include "mcrt_fs_adaptor.h"
#include "fatfs.h"
#include "fcb.h"

#define APPLICATION_MEMORY_ID 0

//These should be global for dealing with memory corruption
FIL fp;
FATFS fs;
uint8_t excel_data_counter = 1;

uint8_t init_fatfs(){
	FRESULT ret;
	/* Close file */
	f_close(&fp);
	ret = f_mount(0, "", 0);                     /* Unmount the default drive */
	return ret;
}

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
			//error = 1; //there is a write error then break the loop, return error
			//break;
		}

		if(f_putc(row_sep, &fp) < 0){
						//error = 1; //there is a write error then break the loop, return error
						//break;
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
	ret = f_mount(0, "", 0);                     /* Unmount the default drive */

	if (ret != FR_OK) {
		_Error_Handler(__FILE__, __LINE__);
		return 1;
	}

	return 0;
}

//Application Record and Load Functions
uint8_t mcrt_load_application(uint8_t* application_buffer, uint32_t* application_len, uint32_t max_length)
{
	uint32_t i;
	uint8_t ret;
	uint8_t* p_flash;
	uint8_t application_flash_error;

	//Application load is fired at power-up.
	//So it is the first called functions from application domain
	//We are checking flash here and if an error detected
	//disable all the activities related to flash
	application_flash_error = check_flash_memory();

	//Check Flash if everything OK then continue
	if(application_flash_error != 0) return RET_APP_ERROR_FLASHHAL;

	Flash_Block* app_flash = get_flash_data(APPLICATION_MEMORY_ID);
	//init_application();

	if(app_flash->block_start_marker1 == BLOCK_START_MAGIC1){
		p_flash = (uint8_t*)app_flash + FLASHBLOCK_HEADER_SIZE;
		*application_len = app_flash->length;
		//ret = application_load_fromflash(p_flash, app_flash->length);
		//return ret;
	}

	else
		return RET_APP_ERROR_NORECORDING;

	if(*application_len >= max_length) {
		*application_len = 0;
		return 0;
	}

	for(i=0;i<*application_len;i++){
		application_buffer[i] = *(uint8_t*)(p_flash + i);
	}

	return 1;
}

uint8_t mcrt_record_application(uint8_t* application_buffer, uint32_t application_len)
{
	uint8_t ret;
	ret = write_flash_data(application_buffer, application_len, APPLICATION_MEMORY_ID);
	return ret;
}


