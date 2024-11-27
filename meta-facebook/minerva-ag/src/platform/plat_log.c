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
#include <logging/log.h>
#include <libutil.h>
#include "plat_sensor_table.h"
#include "fru.h"
#include "plat_fru.h"
#include "plat_i2c.h"
#include "plat_log.h"

LOG_MODULE_REGISTER(plat_log);

#define LOG_MAX_INDEX 0x0FFF // recount when log index > 0x0FFF
#define LOG_MAX_NUM 5 // total log amount: 100  //remove
// #define LOG_MAX_NUM 100 // total log amount: 100
#define AEGIS_FRU_LOG_START 0x0000 // log offset: 0KB
#define AEGIS_CPLD_REGISTER_START_OFFSET 0x00
#define AEGIS_CPLD_REGISTER_MAX_OFFSET 0x3C
#define EEPROM_MAX_WRITE_TIME 5 // the BR24G512 eeprom max write time is 3.5 ms
#define AEGIS_CPLD_ADDR (0x4C >> 1)
#define I2C_BUS_CPLD I2C_BUS5
#define AEGIS_CPLD_VR_VENDOR_TYPE_REG 0x1C

static plat_err_log_mapping err_log_data[LOG_MAX_NUM];
static uint8_t err_sensor_caches[32];
static uint16_t next_log_position = 0; // Next position to write in the circular buffer
static uint16_t next_index = 0; // Next global index to use for logs

const err_sensor_mapping minerva_ag_sensor_err_codes[] = {
	{ VR_FAULT_ASSERT, SENSOR_NUM_OSFP_P3V3_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P0V85_PVDD_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P0V75_PVDD_CH_N_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P0V75_MAX_PHY_N_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P0V75_PVDD_CH_S_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P0V75_MAX_PHY_S_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P0V8_VDDA_PCIE_TEMP_C },
	{ VR_FAULT_ASSERT, SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_TEMP_C },
	{ DC_ON_STATUS_FAULT_ASSERT, DC_ON_STATUS_FAULT },
	{ DC_OFF_STATUS_FAULT_ASSERT, DC_OFF_STATUS_FAULT },
};

const err_sensor_mapping minerva_ag_sensor_normal_codes[] = {
	{ VR_FAULT_DEASSERT, SENSOR_NUM_OSFP_P3V3_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P0V85_PVDD_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P0V75_PVDD_CH_N_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P0V75_MAX_PHY_N_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P0V75_PVDD_CH_S_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P0V75_MAX_PHY_S_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P0V75_TRVDD_ZONEA_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P1V8_VPP_HBM0_2_4_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P0V75_TRVDD_ZONEB_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P0V4_VDDQL_HBM0_2_4_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P1V1_VDDC_HBM0_2_4_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P0V75_VDDPHY_HBM0_2_4_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P0V9_TRVDD_ZONEA_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P1V8_VPP_HBM1_3_5_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P0V9_TRVDD_ZONEB_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P0V4_VDDQL_HBM1_3_5_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P1V1_VDDC_HBM1_3_5_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P0V75_VDDPHY_HBM1_3_5_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P0V8_VDDA_PCIE_TEMP_C },
	{ VR_FAULT_DEASSERT, SENSOR_NUM_CPU_P1V2_VDDHTX_PCIE_TEMP_C },
	{ DC_ON_STATUS_FAULT_DEASSERT, DC_ON_STATUS_FAULT },
	{ DC_OFF_STATUS_FAULT_DEASSERT, DC_OFF_STATUS_FAULT },
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
position is 1 based
// */
// static uint16_t get_log_position_by_time_order(uint8_t order)
// {
// 	if (order > LOG_MAX_NUM) {
// 		LOG_ERR("Invalid LOG count %d", order);
// 		return 0;
// 	}

// 	uint16_t i = 0;

// 	// Check if the log indices are continuous or if they've wrapped around
// 	for (i = 0; i < LOG_MAX_NUM - 1; i++) {
// 		if ((err_log_data[i + 1].index == (err_log_data[i].index + 1)) ||
// 		    (err_log_data[i + 1].index == 1 && err_log_data[i].index == LOG_MAX_INDEX)) {
// 			continue;
// 		}
// 		break;
// 	}

// 	// Calculate position based on the requested order and current log layout
// 	return (i + LOG_MAX_NUM - (order - 1)) % LOG_MAX_NUM;
// }

// Read logs
// void plat_log_read(uint8_t *log_data, uint8_t cmd_size, uint16_t order)
// {
// 	CHECK_NULL_ARG(log_data);

// 	uint16_t log_position = get_log_position_by_time_order(order);
// 	memcpy(log_data, &err_log_data[log_position], sizeof(uint8_t) * cmd_size);

// 	plat_err_log_mapping *p = (plat_err_log_mapping *)log_data;

// 	LOG_HEXDUMP_INF(log_data, sizeof(uint8_t) * cmd_size, "plat_log_read_before");

// 	if (p->index == 0xFFFF) {
// 		memset(log_data, 0x00, sizeof(uint8_t) * cmd_size); // Clear invalid data
// 	}

// 	LOG_HEXDUMP_INF(log_data, sizeof(uint8_t) * cmd_size, "plat_log_read_after");
// }
void plat_log_read(uint8_t *log_data, uint8_t cmd_size, uint16_t order)
{
	CHECK_NULL_ARG(log_data);

	// Calculate the target log position based on next_log_position
	uint16_t log_position = ((next_log_position - 1) + LOG_MAX_NUM - order) % LOG_MAX_NUM;

	uint16_t eeprom_address = AEGIS_FRU_LOG_START + log_position * sizeof(plat_err_log_mapping);

	LOG_INF("order: %d, log_position: %d, eeprom_address: 0x%X", order, log_position,
		eeprom_address); //remove

	plat_err_log_mapping log_entry;

	if (!plat_eeprom_read(eeprom_address, (uint8_t *)&log_entry,
			      sizeof(plat_err_log_mapping))) {
		LOG_ERR("Failed to read log from EEPROM at position %d (address: 0x%X)", order,
			eeprom_address);
		memset(log_data, 0x00, cmd_size);
		return;
	}

	memcpy(log_data, &log_entry, cmd_size);

	plat_err_log_mapping *p = (plat_err_log_mapping *)log_data;

	// LOG_HEXDUMP_INF(log_data, cmd_size, "plat_log_read_before");

	if (p->index == 0xFFFF) {
		memset(log_data, 0x00, cmd_size);
	}

	// LOG_HEXDUMP_INF(log_data, cmd_size, "plat_log_read_after");
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

bool get_vr_status_word(uint8_t bus, uint8_t addr, uint8_t *vr_status_word)
{
	CHECK_NULL_ARG_WITH_RETURN(vr_status_word, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = 0x79;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read VR status word");
		return false;
	}

	memcpy(vr_status_word, i2c_msg.data, 2);
	return true;
}

bool vr_fault_get_error_data(uint8_t error_num, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	uint8_t bus;
	uint8_t addr;
	uint8_t sensor_dev;
	uint8_t vr_status_word[2] = { 0 };

	find_vr_addr_and_bus_and_sensor_dev_by_sensor_id(error_num, &bus, &addr, &sensor_dev);

	if (!get_vr_status_word(bus, addr, vr_status_word)) {
		LOG_ERR("Failed to get VR status word, error_num: 0x%x , bus: 0x%x, addr: 0x%x",
			error_num, bus, addr);
		return false;
	}

	memcpy(data, vr_status_word, sizeof(vr_status_word));
	return true;
}

bool get_error_data(uint8_t error_num, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	uint8_t error_code = ERROR_TYPE_MAX;

	//find err_code in err_sensor_mapping
	for (uint8_t i = 0; i < ARRAY_SIZE(minerva_ag_sensor_err_codes); i++) {
		if (error_num == minerva_ag_sensor_err_codes[i].err_num) {
			error_code = minerva_ag_sensor_err_codes[i].err_code;
			break;
		}
	}

	if (error_code == ERROR_TYPE_MAX) {
		return false;
	}

	switch (error_code) {
	case VR_FAULT_ASSERT:
		if (!vr_fault_get_error_data(error_num, data))
			return false;
		return true;
	case VR_FAULT_DEASSERT:
	case DC_ON_STATUS_FAULT_ASSERT:
	case DC_ON_STATUS_FAULT_DEASSERT:
	case DC_OFF_STATUS_FAULT_ASSERT:
	case DC_OFF_STATUS_FAULT_DEASSERT:
	case ASIC_FAULT_ASSERT:
	case ASIC_FAULT_DEASSERT:
		return true;
	default:
		return false;
	}
}

// Handle error log events and record them if necessary
void error_log_event(uint8_t sensor_num, bool log_status)
{
	LOG_INF("error_log_event, sensor_num: 0x%x, log_status: %d", sensor_num, log_status);

	bool log_todo = false;
	uint16_t err_code = 0;
	// uint8_t vr_status_word = 0;
	uint8_t dump_data[AEGIS_CPLD_REGISTER_MAX_OFFSET - AEGIS_CPLD_REGISTER_START_OFFSET + 1];

	if (log_status == LOG_DEASSERT) {
		for (uint8_t i = 0; i < ARRAY_SIZE(err_sensor_caches); i++) {
			if (sensor_num == err_sensor_caches[i]) {
				log_todo = true;
				err_sensor_caches[i] = 0;
				for (uint8_t j = 0; j < ARRAY_SIZE(minerva_ag_sensor_normal_codes);
				     j++) {
					if (sensor_num == minerva_ag_sensor_normal_codes[j].err_num)
						err_code =
							minerva_ag_sensor_normal_codes[j].err_code;
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
					for (uint8_t j = 0;
					     j < ARRAY_SIZE(minerva_ag_sensor_err_codes); j++) {
						if (sensor_num ==
						    minerva_ag_sensor_err_codes[j].err_num)
							err_code = minerva_ag_sensor_err_codes[j]
									   .err_code;
					}
					break;
				}
			}
		}
	}

	// Record error log if necessary
	if (log_todo) {
		LOG_INF("Record error, sensor_num: 0x%x", sensor_num);

		uint16_t fru_count = next_log_position;

		// Update the log entry's index
		err_log_data[fru_count].index = next_index;
		next_index = (next_index % LOG_MAX_INDEX) + 1;
		// Update log error code and timestamp
		err_log_data[fru_count].err_code = sensor_num;
		err_log_data[fru_count].sys_time = k_uptime_get();

		get_error_data(sensor_num, err_log_data[fru_count].error_data);

		// Dump CPLD data and store it in cpld_dump
		if (plat_dump_cpld(
			    AEGIS_CPLD_REGISTER_START_OFFSET,
			    (AEGIS_CPLD_REGISTER_MAX_OFFSET - AEGIS_CPLD_REGISTER_START_OFFSET + 1),
			    dump_data)) {
			memcpy(err_log_data[fru_count].cpld_dump, dump_data, sizeof(dump_data));
		} else {
			LOG_ERR("Failed to dump CPLD data");
		}

		//dump err_log_data for debug
		LOG_HEXDUMP_INF(&err_log_data[fru_count], sizeof(plat_err_log_mapping),
				"err_log_data");

		// 1 base fru_count, write_address is 0 base
		uint16_t write_address =
			AEGIS_FRU_LOG_START + (fru_count - 1) * sizeof(plat_err_log_mapping);

		// Write log to EEPROM with error handling
		if (!plat_eeprom_write(write_address, (uint8_t *)&err_log_data[fru_count],
				       sizeof(plat_err_log_mapping))) {
			LOG_ERR("Write Log failed with Error code: %02x", err_code);
		} else {
			k_msleep(EEPROM_MAX_WRITE_TIME); // wait 5ms to write EEPROM
		}

		// Update the next log position
		next_log_position = (fru_count % LOG_MAX_NUM) + 1;
	}
}

void find_last_log_position()
{
	uint16_t max_index = 0; // Highest valid index found
	uint16_t last_position = 0; // Position of the highest valid index
	bool all_empty = true; // Flag to detect if all entries are empty
	plat_err_log_mapping log_entry;

	for (uint16_t i = 0; i < LOG_MAX_NUM; i++) {
		uint16_t eeprom_address = AEGIS_FRU_LOG_START + i * sizeof(plat_err_log_mapping);

		if (!plat_eeprom_read(eeprom_address, (uint8_t *)&log_entry,
				      sizeof(plat_err_log_mapping))) {
			LOG_ERR("Failed to read log at position %d (address: 0x%X)", i,
				eeprom_address);
			continue;
		}

		// Check if the entry is valid
		if (log_entry.index != 0xFFFF && log_entry.index <= LOG_MAX_INDEX) {
			all_empty = false; // At least one entry is valid
			if (log_entry.index > max_index) {
				max_index = log_entry.index; // Update max index
				last_position = i + 1; // Update last position, 1 base
			}
		}
	}

	// All entries are empty
	if (all_empty) {
		LOG_INF("All entries are empty. Initializing next_log_position and next_index to 1.");
		next_log_position = 1;
		next_index = 1;
		return;
	}

	next_log_position = (last_position % LOG_MAX_NUM) + 1;
	next_index = (max_index % LOG_MAX_INDEX) + 1;
	LOG_INF("Next log position: %d, next index: %d", next_log_position, next_index);
}

// Load logs from EEPROM into memory during initialization
void init_load_eeprom_log(void)
{
	memset(err_log_data, 0xFF, sizeof(err_log_data));
	uint16_t log_len = sizeof(plat_err_log_mapping);
	for (uint8_t i = 0; i < LOG_MAX_NUM; i++) {
		if (!plat_eeprom_read(AEGIS_FRU_LOG_START + i * log_len,
				      (uint8_t *)&err_log_data[i], log_len)) {
			LOG_ERR("READ Event %d failed from EEPROM", i + 1);
		}
	}

	// Determine the next log position
	find_last_log_position();
}
