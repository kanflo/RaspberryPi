
#ifndef __ARDUCAM_ARCH_H__
#define __ARDUCAM_ARCH_H__

#include <stdbool.h>
#include <stdint.h>

bool arducam_spi_init(void);
bool arducam_i2c_init(uint8_t sensor_addr);

void bus_write(uint8_t address, uint8_t value);
uint8_t bus_read(uint8_t address);
uint8_t wrSensorReg8_8(uint8_t regID, uint8_t regDat);
uint8_t rdSensorReg8_8(uint8_t regID, uint8_t* regDat);
uint8_t wrSensorReg8_16(uint8_t regID, uint16_t regDat);
uint8_t rdSensorReg8_16(uint8_t regID, uint16_t* regDat);
uint8_t wrSensorReg16_8(uint16_t regID, uint8_t regDat);
uint8_t rdSensorReg16_8(uint16_t regID, uint8_t* regDat);
int wrSensorRegs8_8(const struct sensor_reg reglist[]);
int wrSensorRegs8_16(const struct sensor_reg reglist[]);
int wrSensorRegs16_8(const struct sensor_reg reglist[]);

#endif // __ARDUCAM_ARCH_H__