/*
 * bmp_parser.c
 *
 *  Created on: Jan 8, 2022
 *      Author: mg
 */

#include "stdio.h"
#include "bmp_parser.h"

uint8_t get_uint8_le(uint8_t*p, uint16_t* index)
{
	uint8_t ret;
	ret = (uint16_t) p[*index];
	*index += 1;
	return ret;
}

uint16_t get_uint16_le(uint8_t*p, uint16_t* index)
{
	uint16_t ret;
	ret = (uint16_t) p[*index+1] <<8 | p[*index];
	*index += 2;
	return ret;
}

uint32_t get_uint32_le(uint8_t*p, uint16_t* index)
{
	uint32_t ret;
	ret = (uint32_t) p[*index+3] << 24 | p[*index+2]<< 16 | p[*index+1] <<8 | p[*index];
	*index += 4;
	return ret;
}

uint8_t* bmp_parser(uint8_t* p, uint16_t* Width, uint16_t* Height){ //input an array holding whole file.bmp
	BMP_Header_t header;
	uint16_t i = 0;

	header.Signature = get_uint16_le(p, &i);
	header.FileSize = get_uint32_le(p, &i);
	header.reserved = get_uint32_le(p, &i);
	header.DataOffset = get_uint32_le(p, &i);
	header.Size = get_uint32_le(p, &i);
	header.Width = get_uint32_le(p, &i);
	header.Height = get_uint32_le(p, &i);
	header.Planes = get_uint16_le(p, &i);
	header.Bits_Per_Pixel = get_uint16_le(p, &i);
	header.Compression = get_uint32_le(p, &i);
	header.ImageSize = get_uint32_le(p, &i);
	header.XpixelsPerM = get_uint32_le(p, &i);
	header.YpixelsPerM = get_uint32_le(p, &i);
	header.Colors_Used = get_uint32_le(p, &i);
	header.Important_Colors = get_uint32_le(p, &i);

	*Width = header.Width;
	*Height = header.Height;

	return &p[header.DataOffset];
}

//Todo: improve function with the DMA usage
void GUI_Disbitmap(uint16_t Xpoint, uint16_t Ypoint, uint16_t width, uint16_t height, uint16_t *bmp)
{
	uint16_t i, j;
	uint16_t color;
	uint32_t red, green, blue;
	/*uint16_t r5,g6,b5;
	uint16_t Width, Height;
	uint16_t index = 0;*/

	//uint8_t* image = bmp_parser(pMap, &Width, &Height);

    for(j = 0; j < height; j++) {
        for(i = 0; i <width; i ++) {
        	color = bmp[i+j*width];

        	/*red = (color << 8) &  0xF80000;
        	green = (color <<5) & 0x00FC00;
        	blue = (color << 3) & 0x00001F;*/

            BSP_LCD_DrawPixel(Xpoint+i, Ypoint+j, color);
        }
    }
}
