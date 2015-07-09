#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <wiringPiSPI.h>
#include "arducam.h"
#include "arducam_arch.h"

#define	SPI_ARDUCAM_SPEED	1000000
#define	SPI_ARDUCAM		      0


static int FD;

bool arducam_spi_init(void)
{
	int spi = wiringPiSPISetup(SPI_ARDUCAM, SPI_ARDUCAM_SPEED);
	return spi != -1;
}

bool arducam_i2c_init(uint8_t sensor_addr)
{
	FD = wiringPiI2CSetup(sensor_addr);
	return FD != -1;
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
