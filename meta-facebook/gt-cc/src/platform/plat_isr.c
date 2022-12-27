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

#include <logging/log.h>

#include "libipmi.h"
#include "kcs.h"
#include "power_status.h"
#include "sensor.h"
#include "snoop.h"
#include "plat_gpio.h"
#include "plat_ipmi.h"
#include "plat_sensor_table.h"
#include "oem_1s_handler.h"
#include "hal_gpio.h"
#include "util_sys.h"
#include "pex89000.h"
#include "pldm.h"
#include "plat_mctp.h"
#include "plat_hook.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_isr);

void dc_on_init_pex();
void dc_on_send_cmd_to_dev();

K_WORK_DELAYABLE_DEFINE(dc_on_init_pex_work, dc_on_init_pex);
K_WORK_DELAYABLE_DEFINE(dc_on_send_cmd_to_dev_work, dc_on_send_cmd_to_dev);

#define DC_ON_5_SECOND 5
#define DC_ON_PEX_INIT_RETRY 5

void dc_on_init_pex()
{
	static uint8_t retry[PEX_MAX_NUMBER] = { 0 };
	uint8_t pex_sensor_num_table[PEX_MAX_NUMBER] = { SENSOR_NUM_BB_TEMP_PEX_0,
							 SENSOR_NUM_BB_TEMP_PEX_1,
							 SENSOR_NUM_BB_TEMP_PEX_2,
							 SENSOR_NUM_BB_TEMP_PEX_3 };

	for (int i = 0; i < PEX_MAX_NUMBER; i++) {
		uint8_t sensor_num = pex_sensor_num_table[i];
		sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
		pex89000_init_arg *init_arg = (pex89000_init_arg *)cfg->init_args;

		/* Only need initial when not initial yet */
		if (init_arg && !init_arg->is_init) {
			if (cfg->pre_sensor_read_hook) {
				if (!cfg->pre_sensor_read_hook(cfg->num,
							       cfg->pre_sensor_read_args)) {
					LOG_ERR("sensor 0x%x pre sensor read failed!", cfg->num);
					continue;
				}
			}
			if (pex89000_init(sensor_num) != SENSOR_INIT_SUCCESS) {
				LOG_ERR("PEX%d initial retry, (%d)", init_arg->idx, retry[i]);
				if (retry[i]++ < DC_ON_PEX_INIT_RETRY) {
					k_work_schedule(&dc_on_init_pex_work,
							K_SECONDS(DC_ON_5_SECOND));
				} else {
					LOG_ERR("PEX%d initial failed", init_arg->idx);
				}
			} else {
				LOG_INF("PEX%d initial success", init_arg->idx);
				retry[i] = 0;
			}
			if (cfg->post_sensor_read_hook) {
				if (!cfg->post_sensor_read_hook(sensor_num,
								cfg->post_sensor_read_args, NULL)) {
					LOG_ERR("sensor number 0x%x post_read failed", cfg->num);
				}
			}
		}
	}
}

void dc_on_send_cmd_to_dev()
{
	/**
   * Call function to set endpoint and get parameters for the device, the
   * function description is as defined by zephyr but argument is not used in
   * this function so put NULL here.
   */
	send_cmd_to_dev(NULL);
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
		set_system_led(POWER_LED, SYS_LED_ON, SYS_LED_USER_BIC);
	} else {
		set_system_led(POWER_LED, SYS_LED_OFF, SYS_LED_USER_BIC);
	}
}

void ISR_DC_ON()
{
	LOG_INF("System is DC %s", is_mb_dc_on() ? "on" : "off");
	/* Check whether DC on to send work to initial PEX */
	if (is_mb_dc_on()) {
		k_work_schedule(&dc_on_send_cmd_to_dev_work, K_SECONDS(DC_ON_5_SECOND));
		k_work_schedule(&dc_on_init_pex_work, K_SECONDS(DC_ON_5_SECOND));
	}

	check_light_dc_on_led();
}

void ISR_NIC_ADC_ALERT_N()
{
	if (gpio_get(NIC_ADC_ALERT_N) == 0) {
		LOG_ERR("NIC_ADC_ALERT");
	}

	check_light_fault_led();
}

void ISR_SSD_0_7_ADC_ALERT_N()
{
	if (gpio_get(SSD_0_7_ADC_ALERT_N) == 0) {
		LOG_ERR("SSD_0_7_ADC_ALERT");
	}

	check_light_fault_led();
}

void ISR_SSD_8_15_ADC_ALERT_N()
{
	if (gpio_get(SSD_8_15_ADC_ALERT_N) == 0) {
		LOG_ERR("SSD_8_15_ADC_ALERT");
	}

	check_light_fault_led();
}

void ISR_PEX_ADC_ALERT_N()
{
	if (gpio_get(PEX_ADC_ALERT_N) == 0) {
		LOG_ERR("PEX_ADC_ALERT");
	}

	check_light_fault_led();
}

void ISR_SMB_FPGA_ALERT_R_N()
{
	if (gpio_get(SMB_FPGA_ALERT_R_N) == 0) {
		LOG_ERR("SMB_FPGA_ALERT_R");
	}

	check_light_fault_led();
}

void ISR_SMB_ALERT_PMBUS_R_N()
{
	if (gpio_get(SMB_ALERT_PMBUS_R_N) == 0) {
		LOG_ERR("SMB_ALERT_PMBUS_R");
	}

	check_light_fault_led();
}

void ISR_SMB_ALERT_HSC_R_N()
{
	if (gpio_get(SMB_ALERT_HSC_R_N) == 0) {
		LOG_ERR("SMB_ALERT_HSC_R");
	}

	check_light_fault_led();
}
