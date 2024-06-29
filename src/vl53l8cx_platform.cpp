/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "globals.h"
#include "vl53l8cx_platform.h"
#include "stm32f4xx_hal_spi.h"

SPI_HandleTypeDef hspi1;

// ----------------------------------

SPI spi(D4, D5, D3);

uint8_t WrMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAddress,
		uint8_t *p_values,
		uint32_t size)
{
    printf("platform :: WrMulti addr:%d size:%d\r\n", RegisterAddress, size);
    
    HAL_Delay(10);

    uint8_t status = 0;
	int32_t i = 0;
	uint32_t position = 0;
	uint32_t data_size = 0;
	uint16_t    temp;
	uint8_t data_write[VL53L8CX_COMMS_CHUNK_SIZE + 2];

  for (position = 0; position < size; position += VL53L8CX_COMMS_CHUNK_SIZE) {
    printf("platform :: WrMulti \t\t position:%d\r\n", position);
    if (size > VL53L8CX_COMMS_CHUNK_SIZE) {
      if ((position + VL53L8CX_COMMS_CHUNK_SIZE) > size) {
        data_size = size - position;
      } else {
        data_size = VL53L8CX_COMMS_CHUNK_SIZE;
      }
    } else {
      data_size = size;
    }

    temp = RegisterAddress+position;
    printf("platform :: WrMulti \t\t temp:%d\r\n", temp);

    data_write[0] = SPI_WRITE_MASK(temp) >> 8;
    data_write[1] = SPI_WRITE_MASK(temp) & 0xFF;

    for (i=0; i<data_size; i++) {
        data_write[i+2] = p_values[position + i];
    }

    data_size += 2;
    printf("platform :: WrMulti \t\t data_size:%d\r\n", data_size);

    HAL_Delay(10);


    // dev_spi->beginTransaction(SPISettings(p_platform->spi_speed, MSBFIRST, SPI_MODE3));

//     // cs = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(NCS_C_GPIO_Port, NCS_Pin_C, GPIO_PIN_RESET);
    printf("platform :: WrMulti \t\t HAL_GPIO_WritePin RESET\r\n");

    printf("HAL_SPI_Transmit on %d for %d\r\n", &hspi1.State, data_size);
// SPI: 
    // Arduino is SPIClass::transfer(void *buf, size_t count) 
    // p_platform->dev_spi->transfer(&data_write, data_size);
   // status |= spi.write(data_write, data_write, data_write, 0);
    // status |= HAL_SPI_Transmit(&hspi1, data_write, data_write, 100*data_size);
//     // cs = GPIO_PIN_SET;
    HAL_GPIO_WritePin(NCS_C_GPIO_Port, NCS_Pin_C, GPIO_PIN_SET);
    printf("platform :: WrMulti \t\t HAL_GPIO_WritePin SET\r\n");

  }

//   // if (p_platform->address) {
    
//   // } else {
//   //   return p_platform->Write(p_platform->address, RegisterAddress, p_values, size);
//   // }

  return status;
  
}

uint8_t RdMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAddress,
		uint8_t *p_values,
		uint32_t size)
{
  uint8_t status = 0;


	uint32_t position = 0;
	uint32_t data_size = 0;
	uint16_t    temp;
	uint8_t data_write[VL53L8CX_COMMS_CHUNK_SIZE + 2];

	for (position = 0; position < size; position += VL53L8CX_COMMS_CHUNK_SIZE) {
		if (size > VL53L8CX_COMMS_CHUNK_SIZE) {
			if ((position + VL53L8CX_COMMS_CHUNK_SIZE) > size) {
				data_size = size - position;
			} else {
				data_size = VL53L8CX_COMMS_CHUNK_SIZE;
			}
		} else {
			data_size = size;
		}

		temp = RegisterAddress+position;

		data_write[0] = SPI_READ_MASK(temp) >> 8;
		data_write[1] = SPI_READ_MASK(temp) & 0xFF;

		HAL_GPIO_WritePin(NCS_C_GPIO_Port, NCS_Pin_C, GPIO_PIN_RESET);

		status |= HAL_SPI_Transmit(&hspi1, data_write, 2, 0x1000);
		status |= HAL_SPI_Receive(&hspi1, p_values + position, data_size, 100*data_size);

		HAL_GPIO_WritePin(NCS_C_GPIO_Port, NCS_Pin_C, GPIO_PIN_SET);
	}

	return status;
}



uint8_t RdByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAddress,
		uint8_t *p_value)
{
  uint8_t status = RdMulti(p_platform, RegisterAddress, p_value, 1);
  return status;
}

uint8_t WrByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAddress,
		uint8_t value)
{
    printf("platform :: WrByte addr:%d val:%d\r\n", RegisterAddress, value);
  uint8_t status = WrMulti(p_platform, RegisterAddress, &value, 1);
  return status;
}


// ----------------------------------

void SwapBuffer(
    uint8_t     *buffer,
    uint16_t     size)
{
  uint32_t i, tmp;

  /* Example of possible implementation using <string.h> */
  for(i = 0; i < size; i = i + 4)
  {
    tmp = (
      buffer[i]<<24)
    |(buffer[i+1]<<16)
    |(buffer[i+2]<<8)
    |(buffer[i+3]);

    memcpy(&(buffer[i]), &tmp, 4);
  }
}

uint8_t WaitMs(
		VL53L8CX_Platform *p_platform,
		uint32_t TimeMs)
{
  uint32_t tickstart;
  tickstart = p_platform->GetTick();

  while ((p_platform->GetTick() - tickstart) < TimeMs);

  return 0;
}

void MX_SPI1_Init() {
  hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;

	/* Change SPI baudrate using prescaler:
	 * Default clock is 84MHz
	 *   --> prescaler 2 gives 42MHz
	 *   --> prescaler 4 gives 21MHz
	 *   --> prescaler 8 gives 10.5 Mhz
	 *   --> prescaler 16 gives 5.25 Mhz
	 *   --> prescaler 32 gives 2.6 Mhz
	 *   --> prescaler 64 gives 1.3 Mhz
	 *   ...
	 *  This default configuration is given for 1.3 MHz
	 */
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;

	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        printf("HAL_SPI_Init error\r\n");
  }
}