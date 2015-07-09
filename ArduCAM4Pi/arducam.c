/*
 * arducam.c
 *
 *  Created on: 2015.01.16
 *      Author: Lee
 */


/*
  ArduCAM.cpp - Arduino library support for CMOS Image Sensor
  Copyright (C)2011-2013 ArduCAM.com. All right reserved

  Basic functionality of this library are based on the demo-code provided by
  ArduCAM.com. You can find the latest version of the library at
  http://www.ArduCAM.com

  Now supported controllers:
		-	OV7670
		-	MT9D111
		-	OV7675
		-	OV2640
		-	OV3640
		-	OV5642
		-	OV7660
		-	OV7725

	We will add support for many other sensors in next release.

  Supported MCU platform
 		-	Theoretically support all Arduino families
  		-	Arduino UNO R3			(Tested)
  		-	Arduino MEGA2560 R3		(Tested)
  		-	Arduino Leonardo R3		(Tested)
  		-	Arduino Nano			(Tested)
  		-	Arduino DUE				(Tested)
  		-	Arduino Yun				(Tested)
  		-	Raspberry Pi			(Tested)

  If you make any modifications or improvements to the code, I would appreciate
  that you share the code with me so that I might include it in the next release.
  I can be contacted through http://www.ArduCAM.com

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*------------------------------------
	Revision History:
	2015/01/16 	V1.0	by Lee	first release for Raspberry Pi
--------------------------------------*/

#include "arducam.h"
#include "arducam_arch.h"
#include "utft_spi.h"
#include "memorysaver.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <wiringPiSPI.h>

void delayms(int i)
{
	while(i--);
}

int arducam(uint8_t model)
{

	myCAM.sensor_model = model;
	switch(myCAM.sensor_model)
	{
		case OV7660:
		case OV7670:
		case OV7675:
		case OV7725:
			myCAM.sensor_addr = 0x21;
			break;
		case MT9D111:
			myCAM.sensor_addr = 0x5d;
			break;
		case OV3640:
		case OV5642:
			myCAM.sensor_addr = 0x3c;
			break;
		case OV2640:
		case OV9655:
			myCAM.sensor_addr = 0x30;
			break;
		case MT9M112:
			myCAM.sensor_addr = 0x5d;
			break;
		default:
			myCAM.sensor_addr = 0x21;
			break;
	}
	if (!arducam_spi_init()) {
		printf("ERROR: SPI init failed\n");
		return 0;
	}

	// initialize i2c:
	if (!arducam_i2c_init(myCAM.sensor_addr)) {
		printf("ERROR: I2C init failed\n");
		return 0;
	}
	return 1;
}

void arducam_init()
{
	uint8_t reg_val;
	switch(myCAM.sensor_model)
	{
		case OV7660:
		{
			#if defined OV7660_CAM
			arducam_i2c_write(0x12, 0x80);
			delay(100);
			(void) arducam_i2c_write_regs(OV7660_QVGA);
			#endif
			break;
		}
		case OV7725:
		{
			#if defined OV7725_CAM
			arducam_i2c_write(0x12, 0x80);
			delay(100);
			(void) arducam_i2c_write_regs(OV7725_QVGA);
			arducam_i2c_read(0x15,&reg_val);
			arducam_i2c_write(0x15, (reg_val | 0x02));
			#endif
			break;
		}
		case OV7670:
		{
			#if defined OV7670_CAM
			arducam_i2c_write(0x12, 0x80);
			delay(100);
			(void) arducam_i2c_write_regs(OV7670_QVGA);
			#endif
			break;
		}
		case OV7675:
		{
			#if defined OV7675_CAM
			arducam_i2c_write(0x12, 0x80);
			delay(100);
			(void) arducam_i2c_write_regs(OV7675_QVGA);

			#endif
			break;
		}
		case MT9D111:
		{
			#if defined MT9D111_CAM
			//arducam_i2c_write_regs16(MT9D111_QVGA_3fps);
			arducam_i2c_write_regs16(MT9D111_QVGA_15fps);
			//arducam_i2c_write_regs16(MT9D111_QVGA_30fps);
			delay(1000);
			arducam_i2c_write16(0x97, 0x0020);
			arducam_i2c_write16(0xf0, 0x00);
			arducam_i2c_write16(0x21, 0x8403); //Mirror Column
			arducam_i2c_write16(0xC6, 0xA103);//SEQ_CMD
        	arducam_i2c_write16(0xC8, 0x0005); //SEQ_CMD
        	#endif
			break;

		}
		case OV5642:
		{
			#if defined OV5642_CAM
			arducam_i2c_word_write(0x3008, 0x80);

			delay(100);
			if(myCAM.m_fmt == JPEG)
			{
				arducam_i2c_write_word_regs(OV5642_1080P_Video_setting);
				arducam_i2c_word_read(0x3818,&reg_val);
				arducam_i2c_word_write(0x3818, (reg_val | 0x20) & 0xBf);
				arducam_i2c_word_read(0x3621,&reg_val);
				arducam_i2c_word_write(0x3621, reg_val | 0x20);
			}
			else
			{
				arducam_i2c_write_word_regs(OV5642_RGB_QVGA);
				arducam_i2c_word_read(0x3818,&reg_val);
				arducam_i2c_word_write(0x3818, (reg_val | 0x60) & 0xff);
				arducam_i2c_word_read(0x3621,&reg_val);
				arducam_i2c_word_write(0x3621, reg_val & 0xdf);
			}

			#endif
			break;
		}
		case OV3640:
		{
			#if defined OV3640_CAM
			(void) arducam_i2c_write_word_regs(OV3640_QVGA);
			#endif
			break;
		}
		case OV2640:
		{
			#if defined OV2640_CAM
			arducam_i2c_write(0xff, 0x01);
			arducam_i2c_write(0x12, 0x80);
			delay(100);
			if(myCAM.m_fmt == JPEG)
			{
				arducam_i2c_write_regs(OV2640_JPEG_INIT);
				arducam_i2c_write_regs(OV2640_YUV422);
				arducam_i2c_write_regs(OV2640_JPEG);
				arducam_i2c_write(0xff, 0x01);
				arducam_i2c_write(0x15, 0x00);
				arducam_i2c_write_regs(OV2640_320x240_JPEG);
				//arducam_i2c_write(0xff, 0x00);
				//arducam_i2c_write(0x44, 0x32);
			}
			else
			{
				arducam_i2c_write_regs(OV2640_QVGA);
			}
			#endif
			break;
		}
		case OV9655:
		{

			break;
		}
		case MT9M112:
		{

			break;
		}

		default:

			break;
	}
}

void arducam_flush_fifo(void)
{
	arducam_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void arducam_start_capture(void)
{
	arducam_write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

void arducam_clear_fifo_flag(void)
{
	arducam_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

uint8_t arducam_read_fifo(void)
{
	uint8_t data;
	data = arducam_spi_read(0x3D);
	return data;
}

uint8_t arducam_read_reg(uint8_t addr)
{
	uint8_t data;
	data = arducam_spi_read(addr & 0x7F);
	return data;
}

void arducam_write_reg(uint8_t addr, uint8_t data)
{
	arducam_spi_write(addr | 0x80, data);
}

void arducam_set_jpeg_size(uint8_t size)
{
	#if defined OV2640_CAM
	switch(size)
	{
		case OV2640_160x120:
			arducam_i2c_write_regs(OV2640_160x120_JPEG);
			break;
		case OV2640_176x144:
			arducam_i2c_write_regs(OV2640_176x144_JPEG);
			break;
		case OV2640_320x240:
			arducam_i2c_write_regs(OV2640_320x240_JPEG);
			break;
		case OV2640_352x288:
			arducam_i2c_write_regs(OV2640_352x288_JPEG);
			break;
		case OV2640_640x480:
			arducam_i2c_write_regs(OV2640_640x480_JPEG);
			break;
		case OV2640_800x600:
			arducam_i2c_write_regs(OV2640_800x600_JPEG);
			break;
		case OV2640_1024x768:
			arducam_i2c_write_regs(OV2640_1024x768_JPEG);
			break;
		case OV2640_1280x1024:
			arducam_i2c_write_regs(OV2640_1280x1024_JPEG);
			break;
		case OV2640_1600x1200:
			arducam_i2c_write_regs(OV2640_1600x1200_JPEG);
			break;
		default:
			arducam_i2c_write_regs(OV2640_320x240_JPEG);
			break;
	}
	#endif
}

void arducam_set_format(uint8_t fmt)
{
	if(fmt == BMP)
		myCAM.m_fmt = BMP;
	else
		myCAM.m_fmt = JPEG;
}

void getnowtime()
{
	char sec[2];
	char min[2];
	char hour[2];
	char day[2];
	char mon[2];
	char year[4];
	time_t timep;
	struct tm *p;
	time(&timep);
	p=localtime(&timep);
	memset(nowtime,0,20);

	sprintf(year,"%d",(1900+p->tm_year));
	strcat(nowtime,year);
	if((1+p->tm_mon) < 10)
	strcat(nowtime,"0");
	sprintf(mon,"%d",(1+p->tm_mon));
	strcat(nowtime,mon);
	if(p->tm_mday < 10)
	strcat(nowtime,"0");
	sprintf(day,"%d",p->tm_mday);
	strcat(nowtime,day);
	if(p->tm_hour < 10)
	strcat(nowtime,"0");
	sprintf(hour,"%d",p->tm_hour);
	strcat(nowtime,hour);
	if(p->tm_min < 10)
	strcat(nowtime,"0");
	sprintf(min,"%d",p->tm_min);
	strcat(nowtime,min);
	if(p->tm_sec < 10)
	strcat(nowtime,"0");
	sprintf(sec,"%d",p->tm_sec);
	strcat(nowtime,sec);
	printf("nowtime is %s\n",nowtime);

}
