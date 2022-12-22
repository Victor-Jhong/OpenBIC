/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <logging/log.h>

#include "libutil.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"
#include "sensor.h"
#include "plat_class.h"
#include "plat_hook.h"

LOG_MODULE_REGISTER(plat_class);

#define NUMBER_OF_ADC_CHANNEL 16
#define AST1030_ADC_BASE_ADDR 0x7e6e9000

#define IO_EXPENDER_ADDRESS 0xEE
#define IO_EXPENDER_PORT1_OUTPUT_OFFSET 0x03
#define IO_EXPENDER_PORT1_CONFIG_OFFSET 0x07

/* ADC information for each channel
 * offset: register offset
 * shift: data of channel
 */
struct ADC_INFO {
	long unsigned int offset;
	int shift;
};

struct ADC_INFO adc_info[NUMBER_OF_ADC_CHANNEL] = {
	{ 0x10, 0 },  { 0x10, 16 },  { 0x14, 0 },  { 0x14, 16 },  { 0x18, 0 },	{ 0x18, 16 },
	{ 0x1C, 0 },  { 0x1C, 16 },  { 0x110, 0 }, { 0x110, 16 }, { 0x114, 0 }, { 0x114, 16 },
	{ 0x118, 0 }, { 0x118, 16 }, { 0x11C, 0 }, { 0x11C, 16 }
};

enum ADC_REF_VOL_SELECTION {
	REF_VOL_2_5V = 0x0, // 2.5V reference voltage selection
	REF_VOL_1_2V = 0x40 // 1.2V reference voltage selection
};

bool get_adc_voltage(int channel, float *voltage)
{
	CHECK_NULL_ARG_WITH_RETURN(voltage, false)

	if (channel >= NUMBER_OF_ADC_CHANNEL) {
		LOG_ERR("Invalid ADC channel-%d", channel);
		return false;
	}

	uint32_t raw_value, reg_value;
	float reference_voltage = 0.0f;

	/* Get ADC reference voltage from Aspeed chip
	 * ADC000: Engine Control
	 * [7:6] Reference Voltage Selection
	 * 00b - 2.5V / 01b - 1.2V / 10b and 11b - External Voltage
	 */
	reg_value = sys_read32(AST1030_ADC_BASE_ADDR);
	switch (reg_value & (BIT(7) | BIT(6))) {
	case REF_VOL_2_5V:
		reference_voltage = 2.5;
		break;
	case REF_VOL_1_2V:
		reference_voltage = 1.2;
		break;
	default:
		LOG_ERR("Unsupported external reference voltage");
		return false;
	}

	// Read ADC raw value
	reg_value = sys_read32(AST1030_ADC_BASE_ADDR + adc_info[channel].offset);
	raw_value =
		(reg_value >> adc_info[channel].shift) & BIT_MASK(10); // 10-bit(0x3FF) resolution

	// Real voltage = raw data * reference voltage / 2 ^ resolution(10)
	*voltage = (raw_value * reference_voltage) / 1024;

	return true;
}

GT_STAGE_REVISION_ID get_stage_by_rev_id()
{
	return (gpio_get(REV_ID0) | (gpio_get(REV_ID1) << 1) | (gpio_get(REV_ID2) << 2));
}

bool system_led_init()
{
	I2C_MSG msg;
	uint8_t i2c_max_retry = 5;

	msg.bus = I2C_BUS5;
	msg.target_addr = IO_EXPENDER_ADDRESS >> 1;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = IO_EXPENDER_PORT1_CONFIG_OFFSET;

	if (i2c_master_read(&msg, i2c_max_retry)) {
		LOG_ERR("write register fails after retry %d times\n", i2c_max_retry);
		return false;
	}

	uint8_t read_write_data = msg.data[0];
	read_write_data &= 0x3F;

	msg.tx_len = 2;
	msg.rx_len = 0;
	msg.data[0] = IO_EXPENDER_PORT1_CONFIG_OFFSET;
	msg.data[1] = read_write_data;

	if (i2c_master_write(&msg, i2c_max_retry)) {
		LOG_ERR("write register fails after retry %d times\n", i2c_max_retry);
		return false;
	}

	set_system_led(POWR_LED, SYS_LED_OFF, SYS_LED_USER_BIC);
	set_system_led(FAULT_LED, SYS_LED_OFF, SYS_LED_USER_BIC);

	check_light_fault_led();
	check_light_dc_on_led();

	return true;
}

bool set_system_led(uint8_t led_type, bool set_status, uint8_t fun_user)
{
	LOG_INF("%d set led %d status %d \n", fun_user, led_type, set_status);

	static uint8_t led_user = 0;

	if (set_status == SYS_LED_ON) {
		if (fun_user == SYS_LED_USER_BIC) {
			led_user |= 0x01;
		} else if (fun_user == SYS_LED_USER_BMC) {
			led_user |= 0x02;
		}
	} else if (set_status == SYS_LED_OFF) {
		if (fun_user == SYS_LED_USER_BIC) {
			led_user &= 0xFE;
		} else if (fun_user == SYS_LED_USER_BMC) {
			led_user &= 0xFD;
		}

		if (led_user == 0x01) {
			LOG_INF("%d was turn on by BIC \n", led_type);
			return true;

		} else if (led_user == 0x02) {
			LOG_INF("%d was turn on by BMC \n", led_type);
			return true;
		}
	}

	I2C_MSG msg;
	uint8_t i2c_max_retry = 5;

	msg.bus = I2C_BUS5;
	msg.target_addr = IO_EXPENDER_ADDRESS >> 1;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = IO_EXPENDER_PORT1_OUTPUT_OFFSET;

	if (i2c_master_read(&msg, i2c_max_retry)) {
		LOG_ERR("write register fails after retry %d times\n", i2c_max_retry);
		return false;
	}

	uint8_t read_write_data = msg.data[0];

	switch (led_type) {
	case POWR_LED:
		if (set_status == SYS_LED_OFF) {
			read_write_data &= 0x7F;
		} else if (set_status == SYS_LED_ON) {
			read_write_data |= 0x80;
		} else {
			LOG_WRN("Unsupported fault led status: %d\n", set_status);
			return false;
		}
		break;
	case FAULT_LED:
		if (set_status == SYS_LED_OFF) {
			read_write_data &= 0xBF;
		} else if (set_status == SYS_LED_ON) {
			read_write_data |= 0x40;
		} else {
			LOG_WRN("Unsupported fault led status: %d\n", set_status);
			return false;
		}
		break;
	default:
		LOG_WRN("Unsupported led type: %d\n", led_type);
		break;
	}

	msg.tx_len = 2;
	msg.rx_len = 0;
	msg.data[0] = IO_EXPENDER_PORT1_OUTPUT_OFFSET;
	msg.data[1] = read_write_data;

	if (i2c_master_write(&msg, i2c_max_retry)) {
		LOG_ERR("write register fails after retry %d times. \n", i2c_max_retry);
		return false;
	}

	return true;
}

bool is_system_fault()
{
	return !gpio_get(NIC_ADC_ALERT_N) | !gpio_get(SSD_0_7_ADC_ALERT_N) |
	       !gpio_get(SSD_8_15_ADC_ALERT_N) | !gpio_get(PEX_ADC_ALERT_N) |
	       !gpio_get(SMB_FPGA_ALERT_R_N) | !gpio_get(SMB_ALERT_PMBUS_R_N) |
	       !gpio_get(SMB_ALERT_HSC_R_N);
}

void check_light_fault_led()
{
	if (is_system_fault()) {
		set_system_led(FAULT_LED, SYS_LED_ON, SYS_LED_USER_BIC);
	} else {
		set_system_led(FAULT_LED, SYS_LED_OFF, SYS_LED_USER_BIC);
	}
}

void check_light_dc_on_led()
{
	if (is_mb_dc_on()) {
		set_system_led(POWR_LED, SYS_LED_ON, SYS_LED_USER_BIC);
	} else {
		set_system_led(POWR_LED, SYS_LED_OFF, SYS_LED_USER_BIC);
	}
}
