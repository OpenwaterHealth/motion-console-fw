/*
 * tca9548.c
 *
 *  Created on: May 7, 2025
 *      Author: gvigelet
 */

#include "tca9548a.h"

#include <stdio.h>
#include <string.h>


// Declare handles for your two multiplexers
TCA9548A_HandleTypeDef iic_mux[2];

int8_t TCA9548A_Init(uint8_t mux_index, I2C_HandleTypeDef *hi2c, uint8_t address) {

    // Initialize handle
	iic_mux[mux_index].hi2c = hi2c;
	iic_mux[mux_index].i2c_address = address;
	iic_mux[mux_index].current_channel = 0xFF; // No channel selected initially

    // Disable all channels to start with a known state
    return TCA9548A_DisableAll(mux_index);
}

int TCA9548A_scan_channel(uint8_t mux_index, uint8_t mux_channel, uint8_t* addr_list, size_t list_size, bool display) {

   uint8_t found = 0;

   if (iic_mux[mux_index].hi2c == NULL) {
       return TCA9548A_ERR_INIT;
   }

   if (TCA9548A_SelectChannel(mux_index, mux_channel) != TCA9548A_OK) {
		printf("error selecting channel %d\r\n", mux_channel);
		return TCA9548A_ERR_INIT;
   }

   // Iterate through all possible 7-bit addresses
   for (uint8_t address = 0x00; address <= 0x7F; address++) {
       HAL_StatusTypeDef status;
       status = HAL_I2C_IsDeviceReady(iic_mux[mux_index].hi2c, address << 1, 2, 50); // Address shift left by 1 for read/write bit
       if (status == HAL_OK) {
       	if(addr_list != NULL && found < list_size) {
       		addr_list[found] = address;
       	}
       	found++;
       	if(display)
       	{
       		printf("%2x ", address);
       	}
       }else{
       	if(display)
       	{
       		printf("-- ");
       	}
       }
       if (display && (address + 1) % 16 == 0)  printf("\r\n");
   }

	if(display)
	{
		printf("\r\n");
	    fflush(stdout);
	}

   return found;
}


int8_t TCA9548A_SelectChannel(uint8_t mux_index, uint8_t channel) {
    if (iic_mux[mux_index].hi2c == NULL) {
        return TCA9548A_ERR_INIT;
    }

    if (channel > 7) {
        return TCA9548A_ERR_CHANNEL;
    }

    uint8_t control_byte = 1 << channel;
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(iic_mux[mux_index].hi2c, iic_mux[mux_index].i2c_address << 1, &control_byte, 1, HAL_MAX_DELAY);

    if (status != HAL_OK) {
        return TCA9548A_ERR_BUS;
    }

    iic_mux[mux_index].current_channel = channel;
    return TCA9548A_OK;
}

int8_t TCA9548A_GetCurrentChannel(uint8_t mux_index) {

    if (iic_mux[mux_index].current_channel == 0xFF) {
        return TCA9548A_ERR_CHANNEL; // No channel selected
    }

    return iic_mux[mux_index].current_channel;
}

int8_t TCA9548A_Write_Data(uint8_t mux_index, uint8_t channel, uint8_t i2c_addr, uint8_t mem_addr, uint8_t data_len, uint8_t* pData) {
    if (iic_mux[mux_index].hi2c == NULL) {
        return TCA9548A_ERR_INIT;
    }

    int8_t ret = TCA9548A_SelectChannel(mux_index, channel);
    if(ret != TCA9548A_OK) {
    	return ret;
    }


    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(iic_mux[mux_index].hi2c,
    											i2c_addr << 1,
												mem_addr,
                                                I2C_MEMADD_SIZE_8BIT,
												pData,
												data_len,
                                                HAL_MAX_DELAY);

    if (status != HAL_OK) {
        // handle error (HAL_I2C_ERROR_TIMEOUT, HAL_I2C_ERROR_AF, etc.)
    	printf("write i2c handle error\r\n");
        ret = TCA9548A_ERR_BUS;
    } else {
    	ret = TCA9548A_OK;
    }

    return ret;
}

int8_t TCA9548A_Read_Data(uint8_t mux_index, uint8_t channel, uint8_t i2c_addr, uint8_t mem_addr, uint8_t data_len, uint8_t* pData)
{
    if (iic_mux[mux_index].hi2c == NULL) {
        return TCA9548A_ERR_INIT;
    }

    int8_t ret = TCA9548A_SelectChannel(mux_index, channel);
    if(ret != TCA9548A_OK) {
    	return ret;
    }


    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(iic_mux[mux_index].hi2c,
    		i2c_addr << 1,
			mem_addr,
			I2C_MEMADD_SIZE_8BIT,
			pData,
			data_len,
			HAL_MAX_DELAY);

    if (status == HAL_OK) {
        ret = TCA9548A_OK;
    } else {
		// Handle error
		printf("read i2c handle error\r\n");
        ret = TCA9548A_ERR_BUS;
	}

    return ret;
}

int8_t TCA9548A_DisableAll(uint8_t mux_index) {
    if (iic_mux[mux_index].hi2c == NULL) {
        return TCA9548A_ERR_INIT;
    }

    uint8_t control_byte = 0x00; // Disable all channels
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(iic_mux[mux_index].hi2c, iic_mux[mux_index].i2c_address << 1, &control_byte, 1, HAL_MAX_DELAY);

    if (status != HAL_OK) {
        return TCA9548A_ERR_BUS;
    }

    iic_mux[mux_index].current_channel = 0xFF; // Indicate no channel selected
    return TCA9548A_OK;
}


