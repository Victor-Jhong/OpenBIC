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

LOG_MODULE_REGISTER(plat_event);

#define POWER_AND_RESET_BUTTON_REG 0x00
#define VR_AND_CLK_ENABLE_PIN_READING_REG 0x01
#define VR_ENABLE_PIN_READING_1_REG 0x02
#define VR_ENABLE_PIN_READING_2_REG 0x03
#define VR_ENABLE_PIN_READING_3_REG 0x04
#define VR_ENABLE_PIN_READING_4_REG 0x05
#define MB_POWER_GOOD_AND_PERST_PIN_READING_REG 0x06
#define VR_POWER_GOOD_PIN_READING_1_REG 0x07
#define VR_POWER_GOOD_PIN_READING_2_REG 0x08
#define VR_POWER_GOOD_PIN_READING_3_REG 0x09
#define VR_POWER_GOOD_PIN_READING_4_REG 0x0A
#define VR_POWER_GOOD_PIN_READING_5_REG 0x0B
#define RSVD_1_REG 0x0C
#define VR_POWER_FAULT_1_REG 0x0D
#define VR_POWER_FAULT_2_REG 0x0E
#define VR_POWER_FAULT_3_REG 0x0F
#define VR_POWER_FAULT_4_REG 0x10
#define VR_POWER_FAULT_5_REG 0x11
#define RSVD_2_REG 0x12
#define RSVD_3_REG 0x13
#define OSFP_PRSNT_PIN_READING_1_REG 0x14
#define OSFP_PRSNT_PIN_READING_2_REG 0x15
#define OSFP_POWER_ENABLE_PIN_READING_1_REG 0x16
#define OSFP_POWER_ENABLE_PIN_READING_2_REG 0x17
#define OSFP_POWER_ENABLE_PIN_READING_3_REG 0x18
#define OSFP_POWER_ENABLE_PIN_READING_4_REG 0x19
#define BOARD_TYPE_REG 0x1A
#define BOARD_REV_ID_REG 0x1B
#define VR_VENDOR_TYPE_REG 0x1C
#define OWL_JTAG_SEL_MUX_REG 0x1D
#define ATH_JTAG_SEL_MUX_REG 0x1E
#define OWL_UART_SEL_MUX_REG 0x1F
#define AEGIS_JTAG_SWITCH_REG 0x20
#define ATH_BOOT_SOURCE_REG 0x21
#define S_OWL_BOOT_SOURCE_REG 0x22
#define N_OWL_BOOT_SOURCE_REG 0x23
#define VR_SMBUS_ALERT_1_REG 0x24
#define VR_SMBUS_ALERT_2_REG 0x25
#define RSVD_4_REG 0x26
#define ASIC_OC_WARN_REG 0x27
#define SYSTEM_ALERT_FAULT_REG 0x28
#define VR_HOT_FAULT_1_REG 0x29
#define VR_HOT_FAULT_2_REG 0x2A
#define TEMPERATURE_IC_OVERT_FAULT_REG 0x2B
#define VR_POWER_INPUT_FAULT_1_REG 0x2C
#define VR_POWER_INPUT_FAULT_2_REG 0x2D
#define LEAK_DETCTION_REG 0x2E
#define RESET_PIN_TO_ICS_STATUS_REG 0x2F
#define CRD_STATUS_REG 0x30
#define CMN_STATUS_REG 0x31
#define RSVD_GPIO_STATUS_REG 0x32
#define UART_IC_STATUS_REG 0x33
#define UBC_MODULE_OC_WARNING_REG 0x34
#define CPLD_EEPROM_STATUS_REG 0x35
#define MTIA_N_OWL_TEST_PIN_STATUS_REG 0x36
#define MTIA_S_OWL_TEST_PIN_STATUS_REG 0x37
#define MTIA_ATH_TEST_PIN_STATUS_REG 0x38
#define MTIA_VQPS_TO_EFUSE_PROGRAMMING_REG 0x39
#define BUFFER_100M_CLK_LOSE_OF_INPUT_SIGNAL_REG 0x3A
#define MTIA_QSPI_BOOT_DISABLE_REG 0x3B
#define ATH_RSVD_GPIO_REG 0x3C

typedef struct _aegis_cpld_info_ {
	uint8_t cpld_offset;
	uint8_t dc_off_defaut;
	uint8_t dc_on_defaut;
	bool is_fault_log; // if true, check the value is defaut or not
	uint8_t is_fault_bit_map; //flag for fault

	//flag for 1st polling
	bool is_first_polling;

	//flag for 1st polling after changing DC status
	bool is_first_polling_after_dc_change;

	//temp data for last polling
	uint8_t last_polling_value;

	bool (*status_changed_cb)(uint8_t *, uint8_t *, uint8_t *, uint8_t *);

} aegis_cpld_info;

typedef struct _vr_error_callback_info_ {
	uint8_t cpld_offset;
	uint8_t vr_status_word_access_map;
	uint8_t bit_0_mapping_vr_sensor_num;
	uint8_t bit_1_mapping_vr_sensor_num;
	uint8_t bit_2_mapping_vr_sensor_num;
	uint8_t bit_3_mapping_vr_sensor_num;
	uint8_t bit_4_mapping_vr_sensor_num;
	uint8_t bit_5_mapping_vr_sensor_num;
	uint8_t bit_6_mapping_vr_sensor_num;
	uint8_t bit_7_mapping_vr_sensor_num;
} vr_error_callback_info;

bool vr_error_callback(uint8_t *cpld_offset, uint8_t *current_cpld_value, uint8_t *excepted_cpld_value, uint8_t *is_fault_bit_map);

aegis_cpld_info aegis_cpld_info_table[] = {
	{ VR_POWER_FAULT_1_REG, 0x00, 0x00, true, false, 			false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_POWER_FAULT_2_REG, 0x00, 0x00, true, false, 			false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_POWER_FAULT_3_REG, 0x00, 0x00, true, false, 			false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_POWER_FAULT_4_REG, 0x00, 0x00, true, false, 			false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_POWER_FAULT_5_REG, 0x00, 0x00, true, false, 			false, false, 0x00,  .status_changed_cb = vr_error_callback },

	{ VR_SMBUS_ALERT_1_REG, 0xFF, 0xFF, true, false,			false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_SMBUS_ALERT_2_REG, 0xFF, 0xFF, true, false, 			false, false, 0x00,  .status_changed_cb = vr_error_callback },

	{ ASIC_OC_WARN_REG, 0xFF, 0xFF, true, false, 				false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ SYSTEM_ALERT_FAULT_REG, 0xFF, 0xFF, true, false, 			false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_HOT_FAULT_1_REG, 0xFF, 0xFF, true, false, 				false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_HOT_FAULT_2_REG, 0xFF, 0xFF, true, false, 				false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ TEMPERATURE_IC_OVERT_FAULT_REG, 0xFF, 0xFF, true, false, 	false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_POWER_INPUT_FAULT_1_REG, 0xFF, 0xFF, true, false, 		false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ VR_POWER_INPUT_FAULT_2_REG, 0xFF, 0xFF, true, false, 		false, false, 0x00,  .status_changed_cb = vr_error_callback },
	{ LEAK_DETCTION_REG, 0xDF, 0xDF, true, false, 				false, false, 0x00,  .status_changed_cb = vr_error_callback },
};


#define CPLD_POLLING_INTERVAL_MS 1000  // 1 second polling interval

bool is_ubc_enabled() {
	uint8_t data = 0;
	if (!plat_read_cpld(VR_ENABLE_PIN_READING_4_REG, &data)) {
		LOG_ERR("Failed to read CPLD register 0x%02X", VR_ENABLE_PIN_READING_4_REG);
		return 0;
	}
	
	/*Bit2: FM_PLD_UBC_EN*/
	return data & 0x02;

}

//TODO: init ubc_enabled_delayed_status when AC on


K_MUTEX_DEFINE(trigger_dc_on_status_changing_mutex);
K_MUTEX_DEFINE(trigger_dc_off_status_changing_mutex);

bool ubc_enabled_delayed_status = false;
bool is_dc_on_status_changing = false;
bool is_dc_off_status_changing = false;


/* RST_ATH_PWR_ON_PLD_R1_N is low active,
* 1 -> power on
* 0 -> power off
*/
void set_ubc_enabled_delayed(struct k_timer *timer)
{
	ubc_enabled_delayed_status = true;
	is_dc_on_status_changing = false;

	if (gpio_get(RST_ATH_PWR_ON_PLD_R1_N) != 1)
		error_log_event( DC_STATUS_FAULT , LOG_ASSERT);
} 

void set_ubc_disabled_delayed(struct k_timer *timer)
{
	ubc_enabled_delayed_status = false;
	is_dc_off_status_changing = false;

	if (gpio_get(RST_ATH_PWR_ON_PLD_R1_N) != 0)
		error_log_event( DC_STATUS_FAULT , LOG_ASSERT);
} 

K_TIMER_DEFINE(set_ubc_enabled_delayed_timer, set_ubc_enabled_delayed, NULL);
K_TIMER_DEFINE(set_ubc_disabled_delayed_timer, set_ubc_disabled_delayed, NULL);

void check_ubc_status()
{
	static bool last_ubc_status = false;
	
	/* Clear all aegis_cpld_info_table is_fault_bit_map when DC or UBC status changed 
	* RST_ATH_PWR_ON_PLD_R1_N could be ready before ubc_enabled_delayed_status(3S after UBC is enabled)
	*/
	if ((last_ubc_status != ubc_enabled_delayed_status) || (gpio_get(RST_ATH_PWR_ON_PLD_R1_N) != ubc_enabled_delayed_status)){
		for (int i = 0; i < sizeof(aegis_cpld_info_table) / sizeof(aegis_cpld_info_table[0]); i++) {
			aegis_cpld_info_table[i].is_fault_bit_map = 0;
		}
	};

	last_ubc_status = ubc_enabled_delayed_status;
	
	/* DC on process should be done in 3s after UBC is enabled */	
	if (is_ubc_enabled() && ubc_enabled_delayed_status != true){
		if (k_mutex_lock(&trigger_dc_on_status_changing_mutex, K_NO_WAIT)) {
			k_timer_start(&set_ubc_enabled_delayed_timer, K_NO_WAIT, K_MSEC(3000));
			is_dc_on_status_changing = true;
		}
	
	/* DC off process should be done in 3s after UBC is enabled */
	} else if (!is_ubc_enabled() && ubc_enabled_delayed_status != false){
		if (k_mutex_lock(&trigger_dc_off_status_changing_mutex, K_NO_WAIT)) {
			k_timer_start(&set_ubc_disabled_delayed_timer, K_NO_WAIT, K_MSEC(3000));
			is_dc_off_status_changing = true;
		}		
	}
}

// clang-format off
vr_error_callback_info vr_error_callback_info_table	[] = {
	{ VR_POWER_FAULT_1_REG, 			0x7E, 	0x00, 											SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_TEMP_C,		SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_TEMP_C,	SENSOR_NUM_UBC_2_TEMP_C, 					SENSOR_NUM_UBC_1_TEMP_C, 					SENSOR_NUM_CPU_P0V75_PVDD_CH_S_TEMP_C,		SENSOR_NUM_CPU_P0V75_MAX_PHY_N_TEMP_C,		0x00 },
	{ VR_POWER_FAULT_2_REG, 			0xDF, 	SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_TEMP_C,	SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_TEMP_C,	SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_TEMP_C,	SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_TEMP_C,	SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_TEMP_C,		0x00,										SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_TEMP_C,		SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_TEMP_C },
	{ VR_POWER_FAULT_3_REG, 			0xD7, 	SENSOR_NUM_CPU_P0V75_PVDD_CH_S_TEMP_C,			SENSOR_NUM_CPU_P0V75_PVDD_CH_N_TEMP_C,			SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_TEMP_C,	0x00,										SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_TEMP_C,	0x00,										SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_TEMP_C,	SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_TEMP_C },
	{ VR_POWER_FAULT_4_REG, 			0x80, 	0x00,											0x00,											0x00,										0x00,										0x00,										0x00,										0x00,										SENSOR_NUM_CPU_P0V85_PVDD_TEMP_C },
	{ VR_POWER_FAULT_5_REG, 			0x48, 	0x00,											0x00,											0x00,										SENSOR_NUM_OSFP_P3V3_TEMP_C,				0x00,										0x00,										SENSOR_NUM_CPU_P0V8_VDDA_PCIE_TEMP_C,		0x00 },
	{ VR_SMBUS_ALERT_1_REG, 			0xFF, 	SENSOR_NUM_OSFP_P3V3_TEMP_C,					SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_TEMP_C,		SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_TEMP_C,	SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_TEMP_C,		SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_TEMP_C,		SENSOR_NUM_CPU_P0V85_PVDD_TEMP_C,			SENSOR_NUM_CPU_P0V75_PVDD_CH_S_TEMP_C,		SENSOR_NUM_CPU_P0V75_PVDD_CH_N_TEMP_C },
	{ VR_SMBUS_ALERT_2_REG, 			0xF8, 	0x00,											0x00,											0x00,										SENSOR_NUM_CPU_P0V8_VDDA_PCIE_TEMP_C,		SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_TEMP_C,	SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_TEMP_C,	SENSOR_NUM_UBC_1_TEMP_C,						SENSOR_NUM_UBC_2_TEMP_C },
	{ ASIC_OC_WARN_REG, 				0x00, 	0x00,											0x00,											0x00,										0x00,										0x00,										0x00,										0x00,										0x00 },
	{ SYSTEM_ALERT_FAULT_REG, 			0x00, 	0x00,											0x00,											0x00,										0x00,										0x00,										0x00,										0x00,										0x00 },
	{ VR_HOT_FAULT_1_REG, 				0xFF, 	SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_TEMP_C,		SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_TEMP_C,		SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_TEMP_C,	SENSOR_NUM_CPU_P0V75_PVDD_CH_S_TEMP_C,		SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_TEMP_C,	SENSOR_NUM_CPU_P0V75_PVDD_CH_N_TEMP_C,		SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_TEMP_C,		SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_TEMP_C },
	{ VR_HOT_FAULT_2_REG, 				0xC0, 	0x00,											0x00,											0x00,										0x00,										0x00,										0x00,										SENSOR_NUM_CPU_P0V8_VDDA_PCIE_TEMP_C,		SENSOR_NUM_OSFP_P3V3_TEMP_C },
	{ TEMPERATURE_IC_OVERT_FAULT_REG, 	0x00, 	0x00,											0x00,											0x00,										0x00,										0x00,										0x00,										0x00,										0x00 },
	{ VR_POWER_INPUT_FAULT_1_REG, 		0xFF, 	SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_TEMP_C,		SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_TEMP_C,		SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_TEMP_C,	SENSOR_NUM_CPU_P0V75_PVDD_CH_S_TEMP_C,		SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_TEMP_C	,	SENSOR_NUM_CPU_P0V75_PVDD_CH_N_TEMP_C,		SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_TEMP_C,		SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_TEMP_C },
	{ VR_POWER_INPUT_FAULT_2_REG, 		0xC0, 	0x00,											0x00,											0x00,										0x00,										0x00,										0x00,										SENSOR_NUM_CPU_P0V8_VDDA_PCIE_TEMP_C,		SENSOR_NUM_OSFP_P3V3_TEMP_C },
	{ LEAK_DETCTION_REG, 				0x00, 	0x00,											0x00,											0x00,										0x00,										0x00,										0x00,										0x00,										0x00 },
};	
// clang-format on

bool vr_error_callback(uint8_t *cpld_offset, uint8_t *current_cpld_value, uint8_t *excepted_cpld_value, uint8_t *is_fault_bit_map)
{

	




    return true; 
}

void poll_cpld_registers() 
{
    uint8_t data = 0;

    while (1) {
		/* Sleep for the polling interval */
        k_msleep(CPLD_POLLING_INTERVAL_MS);		
		
		check_ubc_status();

		/* Check if any status is changing */
		if (is_dc_on_status_changing || is_dc_off_status_changing)
			continue;
		
        for (size_t i = 0; i < ARRAY_SIZE(aegis_cpld_info_table); i++) {
            uint8_t expected_val = ubc_enabled_delayed_status ? aegis_cpld_info_table[i].dc_on_defaut : aegis_cpld_info_table[i].dc_off_defaut;

            // Read from CPLD
            if (!plat_read_cpld(aegis_cpld_info_table[i].cpld_offset, &data)) {
                LOG_ERR("Failed to read CPLD register 0x%02X", aegis_cpld_info_table[i].cpld_offset);
                continue;
            }

            if (aegis_cpld_info_table[i].is_fault_log) {
                uint8_t new_fault_map = (data ^ expected_val);
                
                // get unrecorded fault bit map
                uint8_t unrecorded_fault_map = new_fault_map & ~aegis_cpld_info_table[i].is_fault_bit_map;

                if (unrecorded_fault_map) {
                   if (aegis_cpld_info_table[i].status_changed_cb) {
						aegis_cpld_info_table[i].status_changed_cb(
							&aegis_cpld_info_table[i].cpld_offset,    
							&data,                                    
							&expected_val,                            
							&aegis_cpld_info_table[i].is_fault_bit_map 
						);
					}
                    // update map
                    aegis_cpld_info_table[i].is_fault_bit_map = new_fault_map;

                    aegis_cpld_info_table[i].last_polling_value = data;
                }
            }
        }
    }
}

