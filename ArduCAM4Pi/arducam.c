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
#include "utft_spi.h"
#include "memorysaver.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <wiringPiSPI.h>

uint8_t wrSensorReg8_8(uint8_t regID, uint8_t regDat);
uint8_t rdSensorReg8_8(uint8_t regID, uint8_t* regDat);
uint8_t wrSensorReg8_16(uint8_t regID, uint16_t regDat);
uint8_t rdSensorReg8_16(uint8_t regID, uint16_t* regDat);
uint8_t wrSensorReg16_8(uint16_t regID, uint8_t regDat);
uint8_t rdSensorReg16_8(uint16_t regID, uint8_t* regDat);
int wrSensorRegs8_8(const struct sensor_reg reglist[]);
int wrSensorRegs8_16(const struct sensor_reg reglist[]);
int wrSensorRegs16_8(const struct sensor_reg reglist[]);

void delayms(int i)
{
	while(i--);
}

int PiCAM(uint8_t model)
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
	spi1 = wiringPiSPISetup (SPI_ARDUCAM, SPI_ARDUCAM_SPEED);
	if(spi1==-1)
	{
		printf("SPI初始化失败！\n");
		return 0;
	}
	// initialize i2c:
	FD = wiringPiI2CSetup (myCAM.sensor_addr);
	if(FD==-1)
	{
		return 0;
		printf("I2C初始化失败！\n");
	}
	return 1;
}

void InitCAM()
{
	uint8_t reg_val;
	switch(myCAM.sensor_model)
	{
		case OV7660:
		{
			#if defined OV7660_CAM
			wrSensorReg8_8(0x12, 0x80);
			delay(100);
			(void) wrSensorRegs8_8(OV7660_QVGA);
			#endif
			break;
		}
		case OV7725:
		{
			#if defined OV7725_CAM
			wrSensorReg8_8(0x12, 0x80);
			delay(100);
			(void) wrSensorRegs8_8(OV7725_QVGA);
			rdSensorReg8_8(0x15,&reg_val);
			wrSensorReg8_8(0x15, (reg_val | 0x02));
			#endif
			break;
		}
		case OV7670:
		{
			#if defined OV7670_CAM
			wrSensorReg8_8(0x12, 0x80);
			delay(100);
			(void) wrSensorRegs8_8(OV7670_QVGA);
			#endif
			break;
		}
		case OV7675:
		{
			#if defined OV7675_CAM
			wrSensorReg8_8(0x12, 0x80);
			delay(100);
			(void) wrSensorRegs8_8(OV7675_QVGA);

			#endif
			break;
		}
		case MT9D111:
		{
			#if defined MT9D111_CAM
			//wrSensorRegs8_16(MT9D111_QVGA_3fps);
			wrSensorRegs8_16(MT9D111_QVGA_15fps);
			//wrSensorRegs8_16(MT9D111_QVGA_30fps);
			delay(1000);
			wrSensorReg8_16(0x97, 0x0020);
			wrSensorReg8_16(0xf0, 0x00);
			wrSensorReg8_16(0x21, 0x8403); //Mirror Column
			wrSensorReg8_16(0xC6, 0xA103);//SEQ_CMD
        	wrSensorReg8_16(0xC8, 0x0005); //SEQ_CMD
        	#endif
			break;

		}
		case OV5642:
		{
			#if defined OV5642_CAM
			wrSensorReg16_8(0x3008, 0x80);

			delay(100);
			if(myCAM.m_fmt == JPEG)
			{
				wrSensorRegs16_8(OV5642_1080P_Video_setting);
				rdSensorReg16_8(0x3818,&reg_val);
				wrSensorReg16_8(0x3818, (reg_val | 0x20) & 0xBf);
				rdSensorReg16_8(0x3621,&reg_val);
				wrSensorReg16_8(0x3621, reg_val | 0x20);
			}
			else
			{
				wrSensorRegs16_8(OV5642_RGB_QVGA);
				rdSensorReg16_8(0x3818,&reg_val);
				wrSensorReg16_8(0x3818, (reg_val | 0x60) & 0xff);
				rdSensorReg16_8(0x3621,&reg_val);
				wrSensorReg16_8(0x3621, reg_val & 0xdf);
			}

			#endif
			break;
		}
		case OV3640:
		{
			#if defined OV3640_CAM
			(void) wrSensorRegs16_8(OV3640_QVGA);
			#endif
			break;
		}
		case OV2640:
		{
			#if defined OV2640_CAM
			wrSensorReg8_8(0xff, 0x01);
			wrSensorReg8_8(0x12, 0x80);
			delay(100);
			if(myCAM.m_fmt == JPEG)
			{
				wrSensorRegs8_8(OV2640_JPEG_INIT);
				wrSensorRegs8_8(OV2640_YUV422);
				wrSensorRegs8_8(OV2640_JPEG);
				wrSensorReg8_8(0xff, 0x01);
				wrSensorReg8_8(0x15, 0x00);
				wrSensorRegs8_8(OV2640_320x240_JPEG);
				//wrSensorReg8_8(0xff, 0x00);
				//wrSensorReg8_8(0x44, 0x32);
			}
			else
			{
				wrSensorRegs8_8(OV2640_QVGA);
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

void flush_fifo(void)
{
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void capture(void)
{
	write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

void clear_fifo_flag(void)
{
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

uint8_t read_fifo(void)
{
	uint8_t data;
	data = bus_read(0x3D);
	return data;
}

uint8_t read_reg(uint8_t addr)
{
	uint8_t data;
	data = bus_read(addr & 0x7F);
	return data;
}

void write_reg(uint8_t addr, uint8_t data)
{
	bus_write(addr | 0x80, data);
}

void bus_write(uint8_t address, uint8_t value)
{
	uint8_t spiData [2] ;
	spiData [0] = address ;
  	spiData [1] = value ;
	wiringPiSPIDataRW (SPI_ARDUCAM, spiData, 2) ;
}

uint8_t bus_read(uint8_t address)
{
	uint8_t spiData[2];
	spiData[0] = address ;
	spiData[1] = 0x00 ;
  	wiringPiSPIDataRW (SPI_ARDUCAM, spiData, 2) ;
  	return spiData[1];
}

uint8_t wrSensorReg8_8(uint8_t regID, uint8_t regDat)
{
	if(FD != -1)
	{
		wiringPiI2CWriteReg8(FD,regID,regDat);
		return(1);
	}
	return 0;
}

uint8_t rdSensorReg8_8(uint8_t regID, uint8_t* regDat)
{

	if(FD != -1)
	{
		*regDat = wiringPiI2CReadReg8(FD,regID);
		return(1);
	}
	return 0;
}

uint8_t wrSensorReg8_16(uint8_t regID, uint16_t regDat)
{
	if(FD != -1)
	{
		wiringPiI2CWriteReg16(FD,regID,regDat);
		return(1);
	}
	return 0;
}

uint8_t rdSensorReg8_16(uint8_t regID, uint16_t* regDat)
{
	if(FD != -1)
	{
		*regDat = wiringPiI2CReadReg16(FD,regID);
		return(1);
	}
	return 0;
}

uint8_t wrSensorReg16_8(uint16_t regID, uint8_t regDat)
{
	uint8_t reg_H,reg_L;
	uint16_t value;
	reg_H = (regID >> 8) & 0x00ff;
	reg_L = regID & 0x00ff;
	value =  regDat << 8 | reg_L;
	if(FD != -1)
	{
		i2c_smbus_write_word_data(FD, reg_H, value);
		return(1);
	}
	return 0;
}

uint8_t rdSensorReg16_8(uint16_t regID, uint8_t* regDat)
{
	uint8_t reg_H,reg_L;
	int r;
	reg_H = (regID >> 8) & 0x00ff;
	reg_L = regID & 0x00ff;
	if(FD != -1)
	{
		r = i2c_smbus_write_byte_data(FD,reg_H,reg_L);
		if(r<0)
			return 0;
		*regDat = i2c_smbus_read_byte(FD);
		return(1);
	}
	return 0;
}

int wrSensorRegs8_8(const struct sensor_reg reglist[])
{
	uint16_t reg_addr = 0;
	uint16_t reg_val = 0;
	const struct sensor_reg *next = reglist;

	while ((reg_addr != 0xff) | (reg_val != 0xff))
	{
		reg_addr = pgm_read_word(&next->reg);
		reg_val = pgm_read_word(&next->val);
		if (!wrSensorReg8_8(reg_addr, reg_val)) {
			return 0;
		}
	   	next++;
	}

	return 1;
}


int wrSensorRegs8_16(const struct sensor_reg reglist[])
{
	unsigned int reg_addr,reg_val;
	const struct sensor_reg *next = reglist;

	while ((reg_addr != 0xff) | (reg_val != 0xffff))
	{
		reg_addr = pgm_read_word(&next->reg);
		reg_val = pgm_read_word(&next->val);
		if (!wrSensorReg8_16(reg_addr, reg_val)) {
			return 0;
		}
	   	next++;
	}

	return 1;
}

int wrSensorRegs16_8(const struct sensor_reg reglist[])
{
	unsigned int reg_addr,reg_val;
	const struct sensor_reg *next = reglist;

	while ((reg_addr != 0xffff) | (reg_val != 0xff))
	{
		reg_addr = pgm_read_word(&next->reg);
		reg_val = pgm_read_word(&next->val);
		if (!wrSensorReg8_16(reg_addr, reg_val)) {
			return 0;
		}
	   	next++;
	}

	return 1;
}

void OV2640_set_JPEG_size(uint8_t size)
{
	#if defined OV2640_CAM
	switch(size)
	{
		case OV2640_160x120:
			wrSensorRegs8_8(OV2640_160x120_JPEG);
			break;
		case OV2640_176x144:
			wrSensorRegs8_8(OV2640_176x144_JPEG);
			break;
		case OV2640_320x240:
			wrSensorRegs8_8(OV2640_320x240_JPEG);
			break;
		case OV2640_352x288:
			wrSensorRegs8_8(OV2640_352x288_JPEG);
			break;
		case OV2640_640x480:
			wrSensorRegs8_8(OV2640_640x480_JPEG);
			break;
		case OV2640_800x600:
			wrSensorRegs8_8(OV2640_800x600_JPEG);
			break;
		case OV2640_1024x768:
			wrSensorRegs8_8(OV2640_1024x768_JPEG);
			break;
		case OV2640_1280x1024:
			wrSensorRegs8_8(OV2640_1280x1024_JPEG);
			break;
		case OV2640_1600x1200:
			wrSensorRegs8_8(OV2640_1600x1200_JPEG);
			break;
		default:
			wrSensorRegs8_8(OV2640_320x240_JPEG);
			break;
	}
	#endif
}

void set_format(uint8_t fmt)
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
