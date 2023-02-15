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

#include "plat_gpio.h"
#include "plat_i2c.h"
#include "sensor.h"
#include "plat_led.h"
#include "plat_hook.h"

LOG_MODULE_REGISTER(plat_led);

bool fault_led_set(uint8_t func_user, uint8_t set_status)
{
	LOG_INF("%d set fault led status %d \n", func_user, set_status);

	if ((set_status >= SYS_LED_MAX) || (func_user >= SYS_LED_USER_MAX)) {
		return 1;
	}

	static uint8_t fault_led_status = 0;
	WRITE_BIT(fault_led_status, func_user, set_status);

	if (set_system_led(FAULT_LED, ((fault_led_status != 0) ? SYS_LED_ON : SYS_LED_OFF),
			   func_user))
		return 1;

	return 0;
}

bool power_led_set(uint8_t set_status, uint8_t func_user)
{
	LOG_INF("%d set power led status %d \n", func_user, set_status);

	if ((set_status >= SYS_LED_MAX) || (func_user >= SYS_LED_USER_MAX)) {
		return 1;
	}

	static uint8_t fault_led_status = 0;
	WRITE_BIT(fault_led_status, func_user, set_status);

	if (set_system_led(POWER_LED, ((fault_led_status != 0) ? SYS_LED_ON : SYS_LED_OFF),
			   func_user))
		return 1;

	return 0;
}

void system_led_init()
{
	I2C_MSG msg = { 0 };
	uint8_t i2c_max_retry = 5;

	msg.bus = IO_EXPENDER_BUS;
	msg.target_addr = IO_EXPENDER_ADDRESS;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = IO_EXPENDER_PORT1_OUTPUT_OFFSET;

	if (i2c_master_read(&msg, i2c_max_retry)) {
		LOG_ERR("write register fails after retry %d times\n", i2c_max_retry);
	}

	msg.tx_len = 2;
	msg.rx_len = 0;
	msg.data[1] |= msg.data[0] | BIT(IO_EXPENDER_PORT1_POWER_LED_PIN) |
		       BIT(IO_EXPENDER_PORT1_FAULT_LED_PIN);
	msg.data[0] = IO_EXPENDER_PORT1_OUTPUT_OFFSET;

	if (i2c_master_write(&msg, i2c_max_retry)) {
		LOG_ERR("write register fails after retry %d times.", i2c_max_retry);
		LOG_ERR("system led init failed");
	}

	check_light_fault_led();
	check_light_dc_on_led();
}

bool set_system_led(uint8_t led_type, uint8_t set_status, uint8_t func_user)
{
	LOG_INF("%d set led %d status %d \n", func_user, led_type, set_status);

	/*set port input/output to set LED on/off*/
	I2C_MSG msg = { 0 };
	uint8_t i2c_max_retry = 5;

	msg.bus = IO_EXPENDER_BUS;
	msg.target_addr = IO_EXPENDER_ADDRESS;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = IO_EXPENDER_PORT1_CONFIG_OFFSET;

	if (i2c_master_read(&msg, i2c_max_retry)) {
		LOG_ERR("write register fails after retry %d times\n", i2c_max_retry);
		return 1;
	}

	msg.tx_len = 2;
	msg.rx_len = 0;
	msg.data[1] = msg.data[0];
	msg.data[0] = IO_EXPENDER_PORT1_CONFIG_OFFSET;
	WRITE_BIT(msg.data[1],
		  ((led_type == FAULT_LED) ? IO_EXPENDER_PORT1_FAULT_LED_PIN :
					     IO_EXPENDER_PORT1_POWER_LED_PIN),
		  ((set_status == SYS_LED_ON) ? 0 : 1));

	if (i2c_master_write(&msg, i2c_max_retry)) {
		LOG_ERR("write register fails after retry %d times\n", i2c_max_retry);
		return 1;
	}

	return 0;
}

bool is_system_fault()
{
	return !gpio_get(NIC_ADC_ALERT_N) || !gpio_get(SSD_0_7_ADC_ALERT_N) ||
	       !gpio_get(SSD_8_15_ADC_ALERT_N) || !gpio_get(PEX_ADC_ALERT_N) ||
	       !gpio_get(SMB_ALERT_PMBUS_R_N) || !gpio_get(SMB_ALERT_HSC_R_N);
}

void check_light_fault_led()
{
	if (fault_led_set((is_system_fault() ? SYS_LED_ON : SYS_LED_OFF), SYS_LED_USER_BIC) != 0)
		LOG_ERR("Check light fault led failed");
}

void check_light_dc_on_led()
{
	if (power_led_set((is_mb_dc_on() ? SYS_LED_ON : SYS_LED_OFF), SYS_LED_USER_BIC) != 0)
		LOG_ERR("Check light dc on led failed");
}
