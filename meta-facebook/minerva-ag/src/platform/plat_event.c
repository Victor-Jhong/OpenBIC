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
#include "plat_i2c.h"
#include "plat_class.h"
#include "plat_pldm_sensor.h"
#include "plat_gpio.h"
#include "plat_log.h"
#include "plat_event.h"
#include "plat_hook.h"

LOG_MODULE_REGISTER(plat_event);

#define CPLD_POLLING_INTERVAL_MS 1000 // 1 second polling interval

K_THREAD_STACK_DEFINE(cpld_polling_stack, POLLING_CPLD_STACK_SIZE);
struct k_thread cpld_polling_thread;
k_tid_t cpld_polling_tid;

typedef struct _vr_error_callback_info_ {
	uint8_t cpld_offset;
	uint8_t vr_status_word_access_map;
	uint8_t bit_mapping_vr_sensor_num[8];
} vr_error_callback_info;

bool vr_error_callback(aegis_cpld_info *cpld_info, uint8_t *current_cpld_value);

// clang-format off
aegis_cpld_info aegis_cpld_info_table[] = {
	{ VR_POWER_FAULT_1_REG, 			0x00, 0x08, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },//remove
	// { VR_POWER_FAULT_1_REG, 			0x00, 0x00, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_POWER_FAULT_2_REG, 			0x00, 0x10, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },//remove
	// { VR_POWER_FAULT_2_REG, 			0x00, 0x00, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_POWER_FAULT_3_REG, 			0x00, 0x00, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_POWER_FAULT_4_REG, 			0x00, 0x00, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_POWER_FAULT_5_REG, 			0x00, 0x00, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_SMBUS_ALERT_1_REG, 			0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_SMBUS_ALERT_2_REG, 			0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ ASIC_OC_WARN_REG, 				0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ SYSTEM_ALERT_FAULT_REG, 			0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_HOT_FAULT_1_REG, 				0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_HOT_FAULT_2_REG, 				0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ TEMPERATURE_IC_OVERT_FAULT_REG, 	0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_POWER_INPUT_FAULT_1_REG, 		0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_POWER_INPUT_FAULT_2_REG, 		0xFF, 0xFF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ LEAK_DETCTION_REG, 				0xDF, 0xDF, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback },
};
// clang-format on

// enum VR_UBC_INDEX_E {
// 	UBC_1 = 0,
// 	UBC_2,
// 	VR_1,
// 	VR_2,
// 	VR_3,
// 	VR_4,
// 	VR_5,
// 	VR_6,
// 	VR_7,
// 	VR_8,
// 	VR_9,
// 	VR_10,
// 	VR_11,
// 	VR_MAX,
// };

// typedef struct _vr_ubc_device_table_ {
// 	uint8_t index;
// 	uint8_t sensor_num_1;
// 	uint8_t sensor_num_2;
// } vr_ubc_device_table;

// vr_ubc_device_table vr_device_table[] = {
// 	{ UBC_1, SENSOR_NUM_UBC_1_TEMP_C },
// 	{ UBC_2, SENSOR_NUM_UBC_2_TEMP_C },
// 	{ VR_1, SENSOR_NUM_OSFP_P3V3_TEMP_C },
// 	{ VR_2, SENSOR_NUM_CPU_P0V85_PVDD_TEMP_C },
// 	{ VR_3, SENSOR_NUM_CPU_P0V75_PVDD_CH_N_TEMP_C, SENSOR_NUM_CPU_P0V75_MAX_PHY_N_TEMP_C },
// 	{ VR_4, SENSOR_NUM_CPU_P0V75_PVDD_CH_S_TEMP_C, SENSOR_NUM_CPU_P0V75_MAX_PHY_S_TEMP_C },
// 	{ VR_5, SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_TEMP_C, SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_TEMP_C },
// 	{ VR_6, SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_TEMP_C,
// 	  SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_TEMP_C },
// 	{ VR_7, SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_TEMP_C,
// 	  SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_TEMP_C },
// 	{ VR_8, SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_TEMP_C, SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_TEMP_C },
// 	{ VR_9, SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_TEMP_C, SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_TEMP_C },
// 	{ VR_10, SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_TEMP_C,
// 	  SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_TEMP_C },
// 	{ VR_11, SENSOR_NUM_CPU_P0V8_VDDA_PCIE_TEMP_C, SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_TEMP_C },
// };

// vr_error_callback_info vr_error_callback_info_table[] = {
// 	{ VR_POWER_FAULT_1_REG, 0x7E, { 0x00, VR_5, VR_6, UBC_2, UBC_1, VR_4, VR_3, 0x00 } },
// 	{ VR_POWER_FAULT_2_REG, 0xDF, { VR_10, VR_7, VR_8, VR_5, VR_11, 0x00, VR_8, VR_9 } },
// 	{ VR_POWER_FAULT_3_REG, 0xD7, { VR_4, VR_3, VR_10, 0x00, VR_7, 0x00, VR_9, VR_6 } },
// 	{ VR_POWER_FAULT_4_REG, 0x80, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, VR_2 } },
// 	{ VR_POWER_FAULT_5_REG, 0x48, { 0x00, 0x00, 0x00, VR_1, 0x00, 0x00, VR_11, 0x00 } },
// 	{ VR_SMBUS_ALERT_1_REG, 0xFF, { VR_1, VR_10, VR_7, VR_8, VR_9, VR_2, VR_4, VR_3 } },
// 	{ VR_SMBUS_ALERT_2_REG, 0xF8, { 0x00, 0x00, 0x00, VR_11, VR_5, VR_6, UBC_1, UBC_2 } },
// 	{ ASIC_OC_WARN_REG, 0x00, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
// 	{ SYSTEM_ALERT_FAULT_REG, 0x00, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
// 	{ VR_HOT_FAULT_1_REG, 0xFF, { VR_10, VR_7, VR_6, VR_4, VR_5, VR_3, VR_9, VR_8 } },
// 	{ VR_HOT_FAULT_2_REG, 0xC0, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, VR_11, VR_1 } },
// 	{ TEMPERATURE_IC_OVERT_FAULT_REG, 0x00, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
// 	{ VR_POWER_INPUT_FAULT_1_REG, 0xFF, { VR_10, VR_7, VR_6, VR_4, VR_5, VR_3, VR_9, VR_8 } },
// 	{ VR_POWER_INPUT_FAULT_2_REG, 0xC0, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, VR_11, VR_1 } },
// 	{ LEAK_DETCTION_REG, 0x00, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
// };

bool ubc_enabled_delayed_status = false;
bool is_dc_status_changing = false;

void set_dc_status_changing_status(bool status)
{
	is_dc_status_changing = status;
}

void check_ubc_delayed(struct k_timer *timer)
{
	/* FM_PLD_UBC_EN_R
	 * 1 -> UBC is enabled
	 * 0 -> UBC is disabled
	 */
	bool is_ubc_enabled = (gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH);
	bool is_dc_on = is_mb_dc_on();

	if (is_ubc_enabled != is_dc_on) {
		error_log_event(is_ubc_enabled ? DC_ON_STATUS_FAULT : DC_OFF_STATUS_FAULT,
				LOG_ASSERT);
		set_dc_status_changing_status(false);
		return;
	}

	ubc_enabled_delayed_status = is_ubc_enabled;
}

bool vr_error_callback(aegis_cpld_info *cpld_info, uint8_t *current_cpld_value)
{
	CHECK_NULL_ARG_WITH_RETURN(cpld_info, false);
	CHECK_NULL_ARG_WITH_RETURN(current_cpld_value, false);

	// Get the expected value based on the current UBC status
	uint8_t expected_val =
		ubc_enabled_delayed_status ? cpld_info->dc_on_defaut : cpld_info->dc_off_defaut;

	// Calculate current faults and new faults
	uint8_t current_fault = *current_cpld_value ^ expected_val;
	uint8_t new_fault = current_fault & ~cpld_info->is_fault_bit_map;

	if (!new_fault) {
		return true; // No new faults, return early
	}

	LOG_DBG("CPLD register 0x%02X has fault 0x%02X", cpld_info->cpld_offset, new_fault);

	// Iterate through each bit in new_fault to handle the corresponding VR
	for (uint8_t bit = 0; bit < 8; bit++) {
		if (!(new_fault & (1 << bit))) {
			continue; // Skip if the current bit has no new fault
		}

		// Dynamically generate the error code
		uint16_t error_code = 0x8000 | (bit << 8) | cpld_info->cpld_offset;

		LOG_ERR("Generated error code: 0x%04X (bit %d, CPLD offset 0x%02X)", error_code,
			bit, cpld_info->cpld_offset);

		// Perform additional operations if needed
		error_log_event(error_code, LOG_ASSERT);
	}

	return true;
}

void poll_cpld_registers()
{
	uint8_t data = 0;

	if (is_dc_status_changing == false) {
		ubc_enabled_delayed_status = is_mb_dc_on();
	}

	while (1) {
		/* Sleep for the polling interval */
		k_msleep(CPLD_POLLING_INTERVAL_MS);

		/* Check if any status is changing */
		if (is_dc_status_changing)
			continue;

		for (size_t i = 0; i < ARRAY_SIZE(aegis_cpld_info_table); i++) {
			uint8_t expected_val = ubc_enabled_delayed_status ?
						       aegis_cpld_info_table[i].dc_on_defaut :
						       aegis_cpld_info_table[i].dc_off_defaut;

			// Read from CPLD
			if (!plat_read_cpld(aegis_cpld_info_table[i].cpld_offset, &data)) {
				LOG_ERR("Failed to read CPLD register 0x%02X",
					aegis_cpld_info_table[i].cpld_offset);
				continue;
			}

			if (!aegis_cpld_info_table[i].is_fault_log)
				continue;

			uint8_t new_fault_map = (data ^ expected_val);

			// get unrecorded fault bit map
			uint8_t unrecorded_fault_map =
				new_fault_map & ~aegis_cpld_info_table[i].is_fault_bit_map;

			if (unrecorded_fault_map) {
				if (aegis_cpld_info_table[i].status_changed_cb) {
					aegis_cpld_info_table[i].status_changed_cb(
						&aegis_cpld_info_table[i], &data);
				}
				// update map
				aegis_cpld_info_table[i].is_fault_bit_map = new_fault_map;

				aegis_cpld_info_table[i].last_polling_value = data;
			}
		}
	}
}

void init_cpld_polling(void)
{
	cpld_polling_tid = k_thread_create(
		&cpld_polling_thread, cpld_polling_stack, K_THREAD_STACK_SIZEOF(cpld_polling_stack),
		poll_cpld_registers, NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0,
		K_MSEC(3000)); //sleep for 3 seconds to prevent dc status changing when reboot BIC
	k_thread_name_set(&cpld_polling_thread, "cpld_polling_thread");
}
