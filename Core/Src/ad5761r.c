#include "ad5761r.h"

#include <stdio.h>

// ---- Internal helpers ----
static inline void cs_low(const ad5761r_dev *d){ HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_RESET); }
static inline void cs_high(const ad5761r_dev *d){ HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET); }

static inline void drv_gpio_write(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState st) {
    if (port) HAL_GPIO_WritePin(port, pin, st);
}
static inline GPIO_PinState drv_gpio_read(GPIO_TypeDef* port, uint16_t pin) {
    if (port) return HAL_GPIO_ReadPin(port, pin);
    return 0;
}


// Convert volts to 16-bit code for chosen span; clamps to range
uint16_t volts_to_code(const ad5761r_dev *dev, float v)
{
  // Compute nominal span and limits
  float vmin=0.f, vmax=5.f; // default
  switch (dev->ra) {
    case AD5761R_RANGE_M_10V_TO_P_10V: vmin=-10.f; vmax=+10.f; break;
    case AD5761R_RANGE_0_V_TO_P_10V: vmin=0.f;   vmax=+10.f; break;
    case AD5761R_RANGE_M_5V_TO_P_5V:  vmin=-5.f;  vmax=+5.f;  break;
    case AD5761R_RANGE_0V_TO_P_5V:  vmin=0.f;   vmax=+5.f;  break;
    case AD5761R_RANGE_M_2V5_TO_P_7V5: vmin=-2.5f; vmax=+7.5f; break;
    case AD5761R_RANGE_M_3V_TO_P_3V:  vmin=-3.f;  vmax=+3.f;  break;
    case AD5761R_RANGE_0V_TO_P_16V: vmin=0.f;   vmax=+16.f; break;
    case AD5761R_RANGE_0V_TO_P_20V: vmin=0.f;   vmax=+20.f; break;
  }

  // Optional 5% overrange
  if (dev->ovr_en) {
    float span = (vmax - vmin);
    vmin -= 0.05f * span * 0.5f;   // extend both ends equally
    vmax += 0.05f * span * 0.5f;
  }

  if (v < vmin) v = vmin;
  if (v > vmax) v = vmax;

  // Coding:
  // Unipolar (0..Vmax): straight binary 0..65535
  // Bipolar: use two’s complement if B2C=1 (recommended)
  if (vmin >= 0.f) {
    float code = (v - vmin) * (65535.0f / (vmax - vmin));
    if (code < 0.f)
    {
    	code = 0.f;
    	if (code > 65535.f)
    	{
    		code = 65535.f;
    	}
    }
    return (uint16_t)(code + 0.5f);
  } else {
    float span = vmax - vmin; // e.g., 20 V for ±10V
    if (dev->b2c_range_en) {
      // map v in [vmin,vmax] -> int16_t range −32768..+32767
      float sc = v * (32767.0f / (span / 2.0f)); // ±full-scale -> ±32767
      int32_t s = (int32_t)(sc + (sc >= 0 ? 0.5f : -0.5f));
      if (s < -32768)
      {
    	  s = -32768;
    	  if (s > 32767)
    	  {
    		  s = 32767;
    	  }
      }
      return (uint16_t)((uint16_t)s); // reinterpret as 16-bit
    } else {
      // Bipolar straight binary: offset binary
      float code = ( (v - vmin) * (65535.0f / span) );
      if (code < 0.f)
      {
    	  code = 0.f;
    	  if (code > 65535.f)
    	  {
    		  code = 65535.f;
          }
      }
      return (uint16_t)(code + 0.5f);
    }
  }
}

/**
 * SPI write to device.
 * @param dev - The device structure.
 * @param reg_addr_cmd - The input shift register command.
 *			 Accepted values: CMD_NOP
 *					  CMD_WR_TO_INPUT_REG
 *					  CMD_UPDATE_DAC_REG_FROM_INPUT_REG
 *					  CMD_WR_UPDATE_DAC_REG
 *					  CMD_WR_CTRL_REG
 *					  CMD_SW_DATA_RESET
 *					  CMD_DIS_DAISY_CHAIN
 *					  CMD_RD_INPUT_REG
 *					  CMD_RD_DAC_REG
 *					  CMD_RD_CTRL_REG
 *					  CMD_SW_FULL_RESET
 * @param reg_data - The transmitted data.
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_write(ad5761r_dev *dev,
		      uint8_t reg_addr_cmd,
		      uint16_t reg_data)
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	uint8_t data[3];

	data[0] = reg_addr_cmd;
	data[1] = (reg_data & 0xFF00) >> 8;
	data[2] = (reg_data & 0x00FF) >> 0;
	//printf("Sending d[0] = 0x%02X, d[1] = 0x%02X,d[2] = 0x%02X\r\n", data[0], data[1], data[2]);
	cs_low(dev);
	// ret = HAL_SPI_Transmit(dev->hspi, data, 3, HAL_MAX_DELAY);
	ret = HAL_SPI_Transmit(dev->hspi, &data[0], 1, HAL_MAX_DELAY);
	ret = HAL_SPI_Transmit(dev->hspi, &data[1], 1, HAL_MAX_DELAY);
	ret = HAL_SPI_Transmit(dev->hspi, &data[2], 1, HAL_MAX_DELAY);
	cs_high(dev);

	return ret;
}

/**
 *  SPI read from device.
 * @param dev - The device structure.
 * @param reg_addr_cmd - The input shift register command.
 *			 Accepted values: CMD_NOP
 *					  CMD_WR_TO_INPUT_REG
 *					  CMD_UPDATE_DAC_REG_FROM_INPUT_REG
 *					  CMD_WR_UPDATE_DAC_REG
 *					  CMD_WR_CTRL_REG
 *					  CMD_SW_DATA_RESET
 *					  CMD_DIS_DAISY_CHAIN
 *					  CMD_RD_INPUT_REG
 *					  CMD_RD_DAC_REG
 *					  CMD_RD_CTRL_REG
 *					  CMD_SW_FULL_RESET
 * @param reg_data - The received data.
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_read(ad5761r_dev *dev,
		     uint8_t reg_addr_cmd,
		     uint16_t *reg_data)
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	uint8_t data[3];
	uint8_t rx_data[3];

	data[0] = reg_addr_cmd;
	data[1] = 0;
	data[2] = 0;
	cs_low(dev);
	ret = HAL_SPI_TransmitReceive(dev->hspi, data, rx_data, 3, HAL_MAX_DELAY);
	cs_high(dev);

	*reg_data = (rx_data[1] << 8) | rx_data[2];

	return ret;
}

/**
 * Readback the register data.
 * Note: Readback operation is not enabled if daisy-chain mode is disabled.
 * @param dev - The device structure.
 * @param reg - The register to be read.
 *		Accepted values: AD5761R_REG_INPUT
 *				 AD5761R_REG_DAC
 *				 AD5761R_REG_CTRL
 * @param reg_data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_register_readback(ad5761r_dev *dev,
				  enum ad5761r_reg reg_addr_cmd,
				  uint16_t *reg_data)
{
    HAL_StatusTypeDef ret;
    uint8_t tx[3], rx[3];

    // 1) Queue the read command (first frame) – result not valid yet
    tx[0] = reg_addr_cmd;
    tx[1] = 0;
    tx[2] = 0;

    //printf("Sending tx[2]= 0x%02X, tx[1]= 0x%02X, tx[0]= 0x%02X\r\n", tx[2],tx[1],tx[0]);
    cs_low(dev);
    ret = HAL_SPI_Transmit(dev->hspi, tx, 3, HAL_MAX_DELAY);
    cs_high(dev);
    if (ret != HAL_OK) return ret;

    // 2) Clock out the result on the next frame (send NOP)
    tx[0] = CMD_NOP; tx[1] = 0; tx[2] = 0;
    cs_low(dev);
    ret = HAL_SPI_Receive(dev->hspi, rx, 3, HAL_MAX_DELAY);
    cs_high(dev);
    if (ret != HAL_OK) return ret;

    *reg_data = (rx[1] << 8) | rx[2];
    return HAL_OK;
}

/**
 * Configure the part based on the settings stored in the device structure.
 * @param dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_config(ad5761r_dev *dev)
{
	uint16_t reg_data;

	reg_data = AD5761R_CTRL_CV(dev->cv) |
		   (dev->ovr_en ? AD5761R_CTRL_OVR : 0) |
		   (dev->b2c_range_en ? AD5761R_CTRL_B2C : 0) |
		   (dev->exc_temp_sd_en ? AD5761R_CTRL_ETS : 0) |
		   (dev->int_ref_en ? AD5761R_CTRL_IRO : 0) |
		   AD5761R_CTRL_PV(dev->pv) |
		   AD5761R_CTRL_RA(dev->ra);

	printf("Config Data 0x%02X\r\n", reg_data);
	return ad5761r_write(dev, CMD_WR_CTRL_REG, reg_data);
}

/**
 * Enable/disable daisy-chain mode.
 * @param dev - The device structure.
 * @param en_dis - Set true in order to enable the daisy-chain mode.
 *		   Accepted values: true
 *				    false
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_set_daisy_chain_en_dis(ad5761r_dev *dev,
				       bool en_dis)
{
	dev->daisy_chain_en = en_dis;

	return ad5761r_write(dev, CMD_DIS_DAISY_CHAIN,
			     AD5761R_DIS_DAISY_CHAIN_DDC(!en_dis));
}

/**
 * Get the status of the daisy-chain mode.
 * @param dev - The device structure.
 * @param en_dis - The status of the daisy-chain mode (enabled, disabled).
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_get_daisy_chain_en_dis(ad5761r_dev *dev,
				       bool *en_dis)
{
	*en_dis = dev->daisy_chain_en;

	return HAL_OK;
}

/**
 * Set the output_range.
 * @param dev - The device structure.
 * @param out_range - The output range.
 *		      Accepted values: AD5761R_RANGE_M_10V_TO_P_10V,
 *				       AD5761R_RANGE_0_V_TO_P_10V
 *				       AD5761R_RANGE_M_5V_TO_P_5V
 *				       AD5761R_RANGE_0V_TO_P_5V
 *				       AD5761R_RANGE_M_2V5_TO_P_7V5
 *				       AD5761R_RANGE_M_3V_TO_P_3V
 *				       AD5761R_RANGE_0V_TO_P_16V
 *				       AD5761R_RANGE_0V_TO_P_20V
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_set_output_range(ad5761r_dev *dev,
				 enum ad5761r_range out_range)
{
	dev->ra = out_range;

	return ad5761r_config(dev);
}

/**
 * Get the output_range.
 * @param dev - The device structure.
 * @param out_range - The output range values.
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_get_output_range(ad5761r_dev *dev,
				 enum ad5761r_range *out_range)
{
	*out_range = dev->ra;

	return HAL_OK;
}

/**
 * Set the power up voltage.
 * @param dev - The device structure.
 * @param pv - The power up voltage.
 *	       Accepted values: AD5761R_SCALE_ZERO
 *				AD5761R_SCALE_HALF
 *				AD5761R_SCALE_FULL
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_set_power_up_voltage(ad5761r_dev *dev,
				     enum ad5761r_scale pv)
{
	dev->pv = pv;

	return ad5761r_config(dev);
}

/**
 * Get the power up voltage.
 * @param dev - The device structure.
 * @param pv - The power up voltage.
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_get_power_up_voltage(ad5761r_dev *dev,
				     enum ad5761r_scale *pv)
{
	*pv = dev->pv;

	return HAL_OK;
}

/**
 * Set the clear voltage.
 * @param dev - The device structure.
 * @param cv - The clear voltage.
 *	       Accepted values: AD5761R_SCALE_ZERO
 *				AD5761R_SCALE_HALF
 *				AD5761R_SCALE_FULL
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_set_clear_voltage(ad5761r_dev *dev,
				  enum ad5761r_scale cv)
{
	dev->cv = cv;

	return ad5761r_config(dev);
}

/**
 * Get the clear voltage.
 * @param dev - The device structure.
 * @param cv - The clear voltage.
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_get_clear_voltage(ad5761r_dev *dev,
				  enum ad5761r_scale *cv)
{
	*cv = dev->cv;

	return HAL_OK;
}

/**
 * Enable/disable internal reference.
 * @param dev - The device structure.
 * @param en_dis - Set true in order to enable the internal reference.
 *		   Accepted values: true
 *				    false
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_set_internal_reference_en_dis(ad5761r_dev *dev,
		bool en_dis)
{
	dev->int_ref_en = en_dis;

	return ad5761r_config(dev);
}

/**
 * Get the status of the internal reference.
 * @param dev - The device structure.
 * @param en_dis - The status of the internal reference (enabled, disabled).
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_get_internal_reference_en_dis(ad5761r_dev *dev,
		bool *en_dis)
{
	*en_dis = dev->int_ref_en;

	return HAL_OK;
}

/**
 * Enable/disable ETS (exceed temperature shutdown) function.
 * @param dev - The device structure.
 * @param en_dis - Set true in order to enable the ETS function.
 *		   Accepted values: true
 *				    false
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_set_exceed_temp_shutdown_en_dis(ad5761r_dev *dev,
		bool en_dis)
{
	dev->exc_temp_sd_en = en_dis;

	return ad5761r_config(dev);
}

/**
 * Get the status of the ETS (exceed temperature shutdown) function.
 * @param dev - The device structure.
 * @param en_dis - The status of the ETS function (enabled, disabled).
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_get_exceed_temp_shutdown_en_dis(ad5761r_dev *dev,
		bool *en_dis)
{
	*en_dis = dev->exc_temp_sd_en;

	return HAL_OK;
}

/**
 * Enable/disable the twos complement bipolar output range.
 * @param dev - The device structure.
 * @param en_dis - Set true in order to enable the twos complement bipolar
 *		   output range.
 *		   Accepted values: true
 *				    false
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_set_2c_bipolar_range_en_dis(ad5761r_dev *dev,
		bool en_dis)
{
	dev->b2c_range_en = en_dis;

	return ad5761r_config(dev);
}

/**
 * Get the status of the twos complement bipolar output range.
 * @param dev - The device structure.
 * @param en_dis - The status of the twos complement bipolar output range
 *		   (enabled, disabled).
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_get_2c_bipolar_range_en_dis(ad5761r_dev *dev,
		bool *en_dis)
{
	*en_dis = dev->b2c_range_en;

	return HAL_OK;
}

/**
 * Enable/disable the 5% overrange.
 * @param dev - The device structure.
 * @param en_dis - Set true in order to enable the 5% overrange.
 *		   Accepted values: true
 *				    false
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_set_overrange_en_dis(ad5761r_dev *dev,
				     bool en_dis)
{
	dev->ovr_en = en_dis;

	return ad5761r_config(dev);
}

/**
 * Get the status of the 5% overrange.
 * @param dev - The device structure.
 * @param en_dis - The status of the twos 5% overrange (enabled, disabled).
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_get_overrange_en_dis(ad5761r_dev *dev,
				     bool *en_dis)
{
	*en_dis = dev->ovr_en;

	return HAL_OK;
}

/**
 * Get the short-circuit condition.
 * Note: The condition is reset at every control register write.
 * @param dev - The device structure.
 * @param sc - The status of the short-circuit condition (detected,
 *		   not detected).
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_get_short_circuit_condition(ad5761r_dev *dev,
		bool *sc)
{
	uint16_t reg_data;
	HAL_StatusTypeDef ret;

	ret = ad5761r_read(dev, CMD_RD_CTRL_REG, &reg_data);
	*sc = ((reg_data & AD5761R_CTRL_SC) >> 12);

	return ret;
}

/**
 * Get the brownout condition.
 * Note: The condition is reset at every control register write.
 * @param dev - The device structure.
 * @param bo - The status of the brownout condition (detected,
 *		   not detected).
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_get_brownout_condition(ad5761r_dev *dev,
				       bool *bo)
{
	uint16_t reg_data;
	HAL_StatusTypeDef ret;

	ret = ad5761r_read(dev, CMD_RD_CTRL_REG, &reg_data);
	*bo = ((reg_data & AD5761R_CTRL_BO) >> 11);

	return ret;
}

/**
 * Set the reset pin value.
 * @param dev - The device structure.
 * @param value - The pin value.
 *		  Accepted values: NO_OS_GPIO_LOW
 *  				   NO_OS_GPIO_HIGH
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_set_reset_pin(ad5761r_dev *dev,
		GPIO_PinState value)
{
	if (dev->rst_port) {
		drv_gpio_write(dev->rst_port, dev->rst_pin, value);
	}

	return HAL_OK;
}

/**
 * Get the reset pin value.
 * @param dev - The device structure.
 * @param value - The pin value.
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_get_reset_pin(ad5761r_dev *dev,
		GPIO_PinState *value)
{
	if (dev->rst_port) {
		*value = drv_gpio_read(dev->rst_port, dev->rst_pin);
		return HAL_OK;
	}

	return HAL_ERROR;
}

/**
 * Set the clr pin value.
 * @param dev - The device structure.
 * @param value - The pin value.
 *		  Accepted values: NO_OS_GPIO_LOW
 *  				   NO_OS_GPIO_HIGH
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_set_clr_pin(ad5761r_dev *dev,
		GPIO_PinState value)
{
	if (dev->clr_port) {
		drv_gpio_write(dev->clr_port, dev->clr_pin, value);
	}

	return HAL_OK;
}

/**
 * Get the clr pin value.
 * @param dev - The device structure.
 * @param value - The pin value.
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_get_clr_pin(ad5761r_dev *dev,
		GPIO_PinState *value)
{
	if (dev->clr_port) {
		*value = drv_gpio_read(dev->clr_port, dev->clr_pin);
		return HAL_OK;
	}

	return HAL_ERROR;
}

/**
 * Set the ldac pin value.
 * @param dev - The device structure.
 * @param value - The pin value.
 *		  Accepted values: NO_OS_GPIO_LOW
 *  				   NO_OS_GPIO_HIGH
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_set_ldac_pin(ad5761r_dev *dev,
		GPIO_PinState value)
{
	if (dev->ldac_port) {
		drv_gpio_write(dev->ldac_port, dev->ldac_pin, value);
	}

	return HAL_OK;
}

/**
 * Get the ldac pin value.
 * @param dev - The device structure.
 * @param value - The pin value.
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_get_ldac_pin(ad5761r_dev *dev,
		GPIO_PinState *value)
{
	if (dev->ldac_port) {
		*value = drv_gpio_read(dev->ldac_port, dev->ldac_pin);
		return HAL_OK;
	}

	return HAL_ERROR;
}

/**
 * Write to input register.
 * @param dev - The device structure.
 * @param dac_data - The DAC data.
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_write_input_register(ad5761r_dev *dev,
				     uint16_t dac_data)
{
	uint16_t reg_data;

	if (dev->type == AD5761R)
		reg_data = AD5761R_DATA(dac_data);
	else
		reg_data = AD5721R_DATA(dac_data);

	return ad5761r_write(dev, CMD_WR_TO_INPUT_REG, reg_data);
}

/**
 * Update DAC register.
 * @param dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_update_dac_register(ad5761r_dev *dev)
{
	return ad5761r_write(dev, CMD_UPDATE_DAC_REG, 0);
}

/**
 * Write to input register and update DAC register.
 * @param dev - The device structure.
 * @param dac_data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_write_update_dac_register(ad5761r_dev *dev,
		uint16_t dac_data)
{
	uint16_t reg_data;

	if (dev->type == AD5761R)
		reg_data = AD5761R_DATA(dac_data);
	else
		reg_data = AD5721R_DATA(dac_data);

	return ad5761r_write(dev, CMD_WR_UPDATE_DAC_REG, reg_data);
}

/**
 * Software data reset.
 * @param dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_software_data_reset(ad5761r_dev *dev)
{
	return ad5761r_write(dev, CMD_SW_DATA_RESET, 0);
}

/**
 * Software full reset.
 * @param dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_software_full_reset(ad5761r_dev *dev)
{
	return ad5761r_write(dev, CMD_SW_FULL_RESET, 0);
}

/**
 * Initialize the device.
 * @param device - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
HAL_StatusTypeDef ad5761r_init(ad5761r_dev *dev)
{
	HAL_StatusTypeDef ret = HAL_ERROR;

    if (!dev || !dev->hspi) return HAL_ERROR;

    ret = ad5761r_write(dev, CMD_SW_FULL_RESET, 0);
	HAL_Delay(1);
	ret |= ad5761r_config(dev);

	return ret;

}
