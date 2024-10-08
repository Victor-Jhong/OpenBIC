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

typedef struct aegis_cpld_info {
	uint8_t cpld_offset;
	uint8_t dc_off_defaut;
	uint8_t dc_on_defaut;
	bool is_fault_log;
	bool is_bic_record;
	bool is_fault; //flag for fault
} aegis_cpld_info;

aegis_cpld_info aegis_cpld_info_table[] = {
	{ POWER_AND_RESET_BUTTON_REG, 0xFF, 0xFF, false, false, true },
	{ VR_AND_CLK_ENABLE_PIN_READING_REG, 0x60, 0xBF, false, true, true },
	{ VR_ENABLE_PIN_READING_1_REG, 0x00, 0xFF, false, true, true },
	{ VR_ENABLE_PIN_READING_2_REG, 0x00, 0xFF, false, true, true },
	{ VR_ENABLE_PIN_READING_3_REG, 0x00, 0xFF, false, true, true },
	{ VR_ENABLE_PIN_READING_4_REG, 0x00, 0xFC, false, true, true },
	{ MB_POWER_GOOD_AND_PERST_PIN_READING_REG, 0x00, 0xC0, false, true, true },
	{ VR_POWER_GOOD_PIN_READING_1_REG, 0x00, 0xFE, false, true, true },
	{ VR_POWER_GOOD_PIN_READING_2_REG, 0x00, 0xFF, false, true, true },
	{ VR_POWER_GOOD_PIN_READING_3_REG, 0x00, 0xFF, false, true, true },
	{ VR_POWER_GOOD_PIN_READING_4_REG, 0x00, 0xFF, false, true, true },
	{ VR_POWER_GOOD_PIN_READING_5_REG, 0x00, 0xFE, false, true, true },
	{ RSVD_1_REG, 0x00, NULL, false, false, true },
	{ VR_POWER_FAULT_1_REG, 0x00, 0x00, true, true, true },
	{ VR_POWER_FAULT_2_REG, 0x00, 0x00, true, true, true },
	{ VR_POWER_FAULT_3_REG, 0x00, 0x00, true, true, true },
	{ VR_POWER_FAULT_4_REG, 0x00, 0x00, true, true, true },
	{ VR_POWER_FAULT_5_REG, 0x00, 0x00, true, true, true },
	{ RSVD_2_REG, 0x00, NULL, false, false, true },
	{ RSVD_3_REG, 0x00, NULL, false, false, true },
	{ OSFP_PRSNT_PIN_READING_1_REG, 0xFF, 0xFF, false, true, true },
	{ OSFP_PRSNT_PIN_READING_2_REG, 0xFF, 0xFF, false, true, true },
	{ OSFP_POWER_ENABLE_PIN_READING_1_REG, 0x00, 0x00, false, false, true },
	{ OSFP_POWER_ENABLE_PIN_READING_2_REG, 0x00, 0x00, false, false, true },
	{ OSFP_POWER_ENABLE_PIN_READING_3_REG, 0x00, 0x00, false, false, true },
	{ OSFP_POWER_ENABLE_PIN_READING_4_REG, 0x00, 0x00, false, false, true },
	{ BOARD_TYPE_REG, 0x00, 0x00, false, true, true },
	{ BOARD_REV_ID_REG, 0x00, 0x00, false, true, true },
	{ VR_VENDOR_TYPE_REG, 0x00, 0x05, false, true, true },
	{ OWL_JTAG_SEL_MUX_REG, 0x00, 0x00, false, false, true },
	{ ATH_JTAG_SEL_MUX_REG, 0x00, 0x00, false, false, true },
	{ OWL_UART_SEL_MUX_REG, 0x00, 0x00, false, false, true },
	{ AEGIS_JTAG_SWITCH_REG, 0x00, 0x00, false, false, true },
	{ ATH_BOOT_SOURCE_REG, 0x00, 0x00, false, false, true },
	{ S_OWL_BOOT_SOURCE_REG, 0x00, 0x00, false, false, true },
	{ N_OWL_BOOT_SOURCE_REG, 0x00, 0x00, false, false, true },
	{ VR_SMBUS_ALERT_1_REG, 0xFF, 0xFF, true, true, true },
	{ VR_SMBUS_ALERT_2_REG, 0xFF, 0xFF, true, true, true },
	{ RSVD_4_REG, 0xFF, NULL, false, false, true },
	{ ASIC_OC_WARN_REG, 0xFF, 0xFF, true, true, true },
	{ SYSTEM_ALERT_FAULT_REG, 0xFF, 0xFF, true, true, true },
	{ VR_HOT_FAULT_1_REG, 0xFF, 0xFF, true, true, true },
	{ VR_HOT_FAULT_2_REG, 0xFF, 0xFF, true, true, true },
	{ TEMPERATURE_IC_OVERT_FAULT_REG, 0xFF, 0xFF, true, true, true },
	{ VR_POWER_INPUT_FAULT_1_REG, 0xFF, 0xFF, true, true, true },
	{ VR_POWER_INPUT_FAULT_2_REG, 0xFF, 0xFF, true, true, true },
	{ LEAK_DETCTION_REG, 0xDF, 0xDF, true, true, true },
	{ RESET_PIN_TO_ICS_STATUS_REG, 0xFF, 0xFF, false, false, true },
	{ CRD_STATUS_REG, 0xFB, 0x27, false, true, true },
	{ CMN_STATUS_REG, 0x3F, 0xEF, false, true, true },
	{ RSVD_GPIO_STATUS_REG, 0xC0, 0xC0, false, false, true },
	{ UART_IC_STATUS_REG, 0xFF, 0x5F, false, false, true },
	{ UBC_MODULE_OC_WARNING_REG, 0xFF, NULL, false, true, true },
};


#define POLLING_INTERVAL_MS 1000  // 1 second polling interval

void cpld_event_func(plat_err_log *log) 
{
	CHECK_NULL_ARG(log);

    LOG_ERR("CPLD status change detected. Log index: %d, err_code: %02X",
            log->index, log->err_code);
    // You can add more detailed logging or processing here if needed


	LOG_HEXDUMP_INFO(log->data, 40, "CPLD data: ");
}

void poll_cpld_registers() {
    uint8_t data = 0;
    bool is_dc_on = is_mb_dc_on();
    plat_err_log temp_log = {0};
    uint8_t temp_buffer[40] = {0};  // Buffer for recording is_bic_record registers
    size_t buffer_index = 0;
    bool status_changed = false;  // Track if any status changed in this cycle

    while (1) {
        status_changed = false;  // Reset status change flag for each polling cycle

        for (size_t i = 0; i < ARRAY_SIZE(aegis_cpld_info_table); i++) {
            uint8_t expected_val = is_dc_on ? aegis_cpld_info_table[i].dc_on_defaut : aegis_cpld_info_table[i].dc_off_defaut;

            // Read from CPLD
            if (!plat_read_cpld(aegis_cpld_info_table[i].cpld_offset, &data)) {
                LOG_ERR("Failed to read CPLD register 0x%02X", aegis_cpld_info_table[i].cpld_offset);
                continue;
            }

            // Store data in temp buffer if is_bic_record is true
            if (aegis_cpld_info_table[i].is_bic_record) {
                if (buffer_index < sizeof(temp_buffer)) {
                    temp_buffer[buffer_index++] = data;
                }
            }

            // Only check for mismatch if is_fault_log is true
            if (aegis_cpld_info_table[i].is_fault_log) {
                if (data != expected_val) {
                    // If it's a fault and wasn't previously marked, record it
                    if (!aegis_cpld_info_table[i].is_fault) {
                        status_changed = true;
                        aegis_cpld_info_table[i].is_fault = true;  // Mark as fault
                        temp_log.err_code = 0x01;  // Set error code for this fault (could vary depending on the register)
                    }
                } else {
                    // If it's back to normal, clear the fault status
                    if (aegis_cpld_info_table[i].is_fault) {
                        status_changed = true;
                        aegis_cpld_info_table[i].is_fault = false;  // Clear the fault
                        temp_log.err_code = 0x02;  // Set error code for fault recovery
                    }
                }
            }
        }

        // If any status change is detected, log all is_bic_record registers
        if (status_changed) {
            // Fill the plat_err_log structure
            temp_log.index = log_index++;  // Increment log index
            temp_log.uptime = k_uptime_get();  // Get current uptime

            // Copy all is_bic_record register data to the cpld_dump field
            memcpy(temp_log.cpld_dump, temp_buffer, buffer_index);

            // Call the event function with the full log
            cpld_event_func(&temp_log);

            // Clear buffer for the next iteration
            memset(temp_buffer, 0, sizeof(temp_buffer));
            buffer_index = 0;
        }

        // Sleep for the polling interval
        k_msleep(POLLING_INTERVAL_MS);
    }
}

