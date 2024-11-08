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

#include <kernel.h>
#include <stdlib.h>
#include "plat_log.h"
#include <logging/log.h>
#include <libutil.h>
#include "plat_sensor_table.h"
#include "fru.h"
#include "plat_fru.h"
#include "plat_i2c.h"

LOG_MODULE_REGISTER(plat_log);

#define LOG_MAX_INDEX 0x0FFF // recount when log index > 0x0FFF
#define LOG_MAX_NUM 100 // total log amount: 100
#define AEGIS_FRU_LOG_START 0x0000 // log offset: 0KB
#define AEGIS_CPLD_REGISTER_START_OFFSET 0x00
#define AEGIS_CPLD_REGISTER_MAX_OFFSET 0x3C
#define EEPROM_MAX_WRITE_TIME 5  // the BR24G512 eeprom max write time is 3.5 ms
#define AEGIS_CPLD_ADDR (0x4C >> 1)
#define I2C_BUS_CPLD I2C_BUS5
#define AEGIS_CPLD_VR_VENDOR_TYPE_REG 0x1C

static plat_err_log_mapping err_log_data[LOG_MAX_NUM];
static uint8_t err_sensor_caches[32];

const err_sensor_mapping sensor_err_codes[] = {
	{ LEAK_CHASSIS_0, 0x01 },
	{ LEAK_CHASSIS_1, 0x02 },
	{ LEAK_CHASSIS_2, 0x03 },
};

const err_sensor_mapping sensor_normal_codes[] = {
	{ PUMP_1_SPEED_RECOVER, 0x04 },
	{ PUMP_2_SPEED_RECOVER, 0x05 },
	{ PUMP_3_SPEED_RECOVER, 0x06 },
};

// Return the number of error logs. It ensures valid index checking.
uint16_t error_log_count(void)
{
	for (uint16_t i = 0; i < LOG_MAX_NUM; i++) {
		// Valid index range check to avoid data corruption
		if (err_log_data[i].index == 0xFFFF || err_log_data[i].index > LOG_MAX_INDEX) {
			return i;
		}
	}
	return LOG_MAX_NUM;
}

/* 
(order = N of "the Nth newest event")
input order to get the position in log array(err_log_data)
*/
static uint16_t get_log_position_by_time_order(uint8_t order)
{
	if (order > LOG_MAX_NUM) {
		LOG_ERR("Invalid LOG count %d", order);
		return 0;
	}

	uint16_t i = 0;

	// Check if the log indices are continuous or if they've wrapped around
	for (i = 0; i < LOG_MAX_NUM - 1; i++) {
		if ((err_log_data[i + 1].index == (err_log_data[i].index + 1)) ||
		    (err_log_data[i + 1].index == 1 && err_log_data[i].index == LOG_MAX_INDEX)) {
			continue;
		}
		break;
	}

	// Calculate position based on the requested order and current log layout
	return (i + LOG_MAX_NUM - (order - 1)) % LOG_MAX_NUM;
}

// Read logs
void plat_log_read(uint8_t *log_data, uint8_t cmd_size, uint16_t order)
{
	CHECK_NULL_ARG(log_data);

	uint16_t log_position = get_log_position_by_time_order(order);
	memcpy(log_data, &err_log_data[log_position], sizeof(uint8_t) * cmd_size);

	plat_err_log_mapping *p = (plat_err_log_mapping *)log_data;

	LOG_HEXDUMP_INF(log_data, sizeof(uint8_t) * cmd_size, "plat_log_read_before");

	if (p->index == 0xFFFF) {
		memset(log_data, 0x00, sizeof(uint8_t) * cmd_size); // Clear invalid data
	}

	LOG_HEXDUMP_INF(log_data, sizeof(uint8_t) * cmd_size, "plat_log_read_after");
}

// Clear logs from memory and EEPROM with error handling
void plat_clear_log()
{
	memset(err_log_data, 0xFF, sizeof(err_log_data));
	memset(err_sensor_caches, 0, sizeof(err_sensor_caches));

	for (uint8_t i = 0; i < LOG_MAX_NUM; i++) {
				if (!plat_eeprom_write(AEGIS_FRU_LOG_START + sizeof(plat_err_log_mapping) * i,
				       (uint8_t *)err_log_data, sizeof(plat_err_log_mapping))) {
			LOG_ERR("Clear EEPROM Log failed at index %d", i);
		}
		k_msleep(EEPROM_MAX_WRITE_TIME); // the eeprom max write time is 10 ms
	}
}

bool plat_dump_cpld(uint8_t offset, uint8_t length, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = I2C_BUS_CPLD;
	i2c_msg.target_addr = AEGIS_CPLD_ADDR;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = length;
	i2c_msg.data[0] = offset;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read CPLD register 0x%02X", offset);
		return false;
	}

	memcpy(data, i2c_msg.data, length);
	return true;
}

// Handle error log events and record them if necessary
void error_log_event(uint8_t sensor_num, bool val_normal)
{
	bool log_todo = false;
	uint16_t err_code = 0;
	// uint8_t vr_status_word = 0;
	uint8_t dump_data[AEGIS_CPLD_REGISTER_MAX_OFFSET - AEGIS_CPLD_REGISTER_START_OFFSET + 1];
	
	if (val_normal) {
		for (uint8_t i = 0; i < ARRAY_SIZE(err_sensor_caches); i++) {
			if (sensor_num == err_sensor_caches[i]) {
				log_todo = true;
				err_sensor_caches[i] = 0;
				for (uint8_t j = 0; j < ARRAY_SIZE(sensor_normal_codes); j++) {
					if (sensor_num == sensor_normal_codes[j].sen_num)
						err_code = sensor_normal_codes[j].err_code;
				}
				break;
			}
		}
	} else {
		log_todo = true;
		for (uint8_t i = 0; i < ARRAY_SIZE(err_sensor_caches); i++) {
			if (sensor_num == err_sensor_caches[i]) {
				log_todo = false; // Duplicate log, skip it
				break;
			}
		}
		if (log_todo) {
			for (uint8_t i = 0; i < ARRAY_SIZE(err_sensor_caches); i++) {
				if (err_sensor_caches[i] == 0) {
					err_sensor_caches[i] = sensor_num;
					for (uint8_t j = 0; j < ARRAY_SIZE(sensor_err_codes); j++) {
						if (sensor_num == sensor_err_codes[j].sen_num)
							err_code = sensor_err_codes[j].err_code;
					}
					break;
				}
			}
		}
	}

	// Record error log if necessary
	if (log_todo) {
		uint16_t newest_count = get_log_position_by_time_order(1);
		uint16_t fru_count = (err_log_data[newest_count].index == 0xFFFF) ?
					     newest_count :
					     ((newest_count + 1) % LOG_MAX_NUM);

		// Update index with proper wrapping around LOG_MAX_INDEX
		err_log_data[fru_count].index =
			(err_log_data[newest_count].index == LOG_MAX_INDEX ||
			 err_log_data[newest_count].index == 0xFFFF) ?
				1 :
				(err_log_data[newest_count].index + 1);
		err_log_data[fru_count].err_code = sensor_num; // Use the sensor number as error code for now
		err_log_data[fru_count].sys_time = k_uptime_get();

		// // Fill error_data with VR status word if it's a VR error
		// if (sensor_num >= VR_ERROR_CODE_START && sensor_num <= VR_ERROR_CODE_END) {
		// 	if (get_vr_status_word(&vr_status_word)) {
		// 		memset(err_log_data[fru_count].error_data, 0, sizeof(err_log_data[fru_count].error_data));
		// 		err_log_data[fru_count].error_data[0] = vr_status_word;
		// 	} else {
		// 		LOG_ERR("Failed to get VR status word for sensor 0x%02X", sensor_num);
		// 	}
		// }

		// Dump CPLD data and store it in cpld_dump
		if (plat_dump_cpld(AEGIS_CPLD_REGISTER_START_OFFSET,
				   (AEGIS_CPLD_REGISTER_MAX_OFFSET - AEGIS_CPLD_REGISTER_START_OFFSET + 1),
				   dump_data)) {
			memcpy(err_log_data[fru_count].cpld_dump, dump_data, sizeof(dump_data));
		} else {
			LOG_ERR("Failed to dump CPLD data \n");
		}

		// Write log to EEPROM with error handling
		if (!plat_eeprom_write(
			    (AEGIS_FRU_LOG_START + fru_count * sizeof(plat_err_log_mapping)),
			    (uint8_t *)&err_log_data[fru_count], sizeof(plat_err_log_mapping))) {
			LOG_ERR("Write Log failed with Error code: %02x", err_code);
		} else {
			k_msleep(EEPROM_MAX_WRITE_TIME); // wait 5ms to write EEPROM
		}
	}
}

// Load logs from EEPROM into memory during initialization
void init_load_eeprom_log(void)
{
	memset(err_log_data, 0xFF, sizeof(err_log_data));
	uint16_t log_len = sizeof(plat_err_log_mapping);
	for (uint8_t i = 0; i < LOG_MAX_NUM; i++) {
		if (!plat_eeprom_read(AEGIS_FRU_LOG_START + i * log_len, (uint8_t *)&err_log_data[i],
				      log_len)) {
			LOG_ERR("READ Event %d failed from EEPROM", i + 1);
		}
	}
}
