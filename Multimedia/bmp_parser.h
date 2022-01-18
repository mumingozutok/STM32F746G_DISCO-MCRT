/*
 * bmp_parser.h
 *
 *  Created on: Jan 8, 2022
 *      Author: mg
 */

#ifndef BMP_PARSER_H_
#define BMP_PARSER_H_

typedef struct S_BMP_Header_t{
	uint16_t Signature;
	uint32_t FileSize;
	uint32_t reserved;
	uint32_t DataOffset;
	uint32_t Size;
	uint32_t Width;
	uint32_t Height;
	uint16_t Planes;
	uint16_t Bits_Per_Pixel;
	uint32_t Compression;
	uint32_t ImageSize;
	uint32_t XpixelsPerM;
	uint32_t YpixelsPerM;
	uint32_t Colors_Used;
	uint32_t Important_Colors;
} BMP_Header_t;

typedef struct S_ColorTable{
	uint8_t Red; //Red intensity
	uint8_t Green; //Green intensity
	uint8_t Blue; //Blue intensity
	uint8_t reserved;
	//repeated numcolors times
} ColorTable_t;

typedef struct S_Pixel_Data{
	uint8_t* image;
}Pixel_Data_t;

typedef union U_BMP_File{
	BMP_Header_t header;
	uint8_t* p;
} BMP_File_t;

uint8_t* bmp_parser(uint8_t* p, uint16_t* Width, uint16_t* Height);
void GUI_Disbitmap(uint16_t Xpoint, uint16_t Ypoint, uint16_t width, uint16_t height, uint16_t *bmp);

#endif /* BMP_PARSER_H_ */
