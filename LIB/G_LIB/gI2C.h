#ifndef __I2CLIB_H
#define __I2CLIB_H

#include "stdint.h"
#include "GPIO_by_DTU.h"

#ifdef __cplusplus
 extern "C" {
#endif



void I2CG_Init(void);
void I2C_LowLevel_Init(I2C_TypeDef* I2Cx);
void I2C_LowLevel_DeInit(I2C_TypeDef* I2Cx);
uint32_t I2C_WrBuf(uint8_t DevAddr, uint8_t *buf, uint32_t cnt, I2C_TypeDef* I2Cx);
uint32_t I2C_RdBuf(uint8_t DevAddr, uint8_t *buf, uint32_t cnt, I2C_TypeDef* I2Cx);
uint32_t I2C_RdBufEasy(uint8_t DevAddr, uint8_t *buf, uint32_t cnt, I2C_TypeDef* I2Cx);

//Write a single byte to the given register address
uint32_t I2C_WrData(uint8_t DevAddr, uint8_t RegAddr, uint8_t data, I2C_TypeDef* I2Cx);

//Reads "cnt" number of data starting from RegAddr
uint32_t I2C_RdData(uint8_t DevAddr, uint8_t RegAddr, uint8_t *buf, uint32_t cnt, I2C_TypeDef* I2Cx);


#ifdef __cplusplus
}
#endif
#endif //__I2CLIB_H
