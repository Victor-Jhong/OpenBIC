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
#include <string.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"
#include "mp2971.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(mp2971);

#define PAGE 0x00
#define MFR_RESO_SET 0xC7

bool get_page(uint8_t *val, uint8_t sensor_num)
{
	if ((val == NULL) || (sensor_num > SENSOR_NUM_MAX)) {
		return -1;
	}

	I2C_MSG msg;
	uint8_t retry = 5;

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = PAGE;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("i2c read failed.\n");
		return SENSOR_FAIL_TO_ACCESS;
	}

	//Victor test
	//LOG_WRN("get_page  = %x  \n \n \n ", msg.data[0]);

	*val = msg.data[0];
	return 0;
}

bool get_mfr_resolution_set(uint16_t *val, uint8_t sensor_num)
{
	if ((val == NULL) || (sensor_num > SENSOR_NUM_MAX)) {
		return -1;
	}

	I2C_MSG msg;
	uint8_t retry = 5;

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = MFR_RESO_SET;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("i2c read failed.\n");
		return SENSOR_FAIL_TO_ACCESS;
	}

	*val = (msg.data[1] << 8) | msg.data[0];
	return 0;
}

float get_resolution(uint8_t sensor_num)
{
	uint8_t page = 0;
	uint16_t mfr_reso_set = 0;
	float res = 0;

	I2C_MSG msg;
	uint8_t retry = 5;

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = PAGE;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("i2c read failed.\n");
		return SENSOR_FAIL_TO_ACCESS;
	}

	page = msg.data[0];
	//Victor test
	LOG_WRN("page = %d  \n", page);

	//Victor test check if it is need to do twice
	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = MFR_RESO_SET;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("i2c read failed.\n");
		return SENSOR_FAIL_TO_ACCESS;
	}

	mfr_reso_set = (msg.data[1] << 8) | msg.data[0];
	//Victor test
	LOG_WRN("page = %x  \n", mfr_reso_set);

	printf("[%s] not support sensor number: 0x%x\n", __func__, sensor_num);
	switch (sensor_num) {
	case PMBUS_READ_VOUT:
		if (page == 0) {
			res = 0.001;
		} else if (page == 1) {
			res = 0.001;
		} else {
			printf("[%s] not support page: 0x%d\n", __func__, page);
		}
		break;
	case PMBUS_READ_IOUT:
		if (page == 0) {
			res = 1;
		} else if (page == 1) {
			res = 0.25;
		} else {
			printf("[%s] not support page: 0x%d\n", __func__, page);
		}
		break;
	case PMBUS_READ_IIN:
		if (page == 0) {
			res = 0.25;
		} else if (page == 1) {
			res = 0.125;
		} else {
			printf("[%s] not support page: 0x%d\n", __func__, page);
		}
		break;
	case PMBUS_READ_TEMPERATURE_1:
		if (page == 0) {
			res = 1;
		} else if (page == 1) {
			res = 1;
		} else {
			printf("[%s] not support page: 0x%d\n", __func__, page);
		}
		break;
	case PMBUS_READ_POUT:
		if (page == 0) {
			res = 2;
		} else if (page == 1) {
			res = 0.25;
		} else {
			printf("[%s] not support page: 0x%d\n", __func__, page);
		}
		break;
	default:
		printf("[%s] not support sensor number: 0x%x\n", __func__, sensor_num);
		ret = false;
		break;
	}

	return res;
}

bool vr_adjust_of_twos_complement(uint8_t offset, int *val)
{
	if (val == NULL) {
		printf("[%s] input value is NULL\n", __func__);
		return false;
	}
	int adjust_val = *val;
	bool is_negative_val = ((adjust_val & TWO_COMPLEMENT_NEGATIVE_BIT) == 0 ? false : true);
	bool ret = true;

	switch (offset) {
	case PMBUS_READ_IOUT:
		// Convert two's complement to usigned integer
		if (is_negative_val == true) {
			adjust_val = ~(adjust_val - 1);
		}

		// Set reading value to 0 if reading value in range -0.2A ~ 0.2A
		// Because register report unit is 0.1A , set reading value to 0 if register report value is lower than 2 units
		if (adjust_val < ADJUST_IOUT_RANGE) {
			*val = 0;
		}
		break;
	case PMBUS_READ_POUT:
		// Set reading value to 0 if reading value is negative
		if (is_negative_val == true) {
			*val = 0;
		}
		break;
	default:
		printf("[%s] not support offset: 0x%x\n", __func__, offset);
		ret = false;
		break;
	}
	return ret;
}

uint8_t mp2971_read(uint8_t sensor_num, int *reading)
{
	if (reading == NULL || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	//Victor test
	//LOG_WRN("mp2971_read sensor_num = %x  \n", sensor_num);

	//get page
	// bool page_ret = false;
	// uint8_t page = 0;
	// page_ret = get_page(&page, sensor_num);
	// if (page_ret != 0) {
	// 	return SENSOR_UNSPECIFIED_ERROR;
	// }

	//Victor test
	//page = 1;
	//LOG_WRN("mp2971_read page = %x  \n", page);

	//get mfr resolution
	// bool res_ret = false;
	// uint16_t mfr_resolution = 0;
	// res_ret = get_mfr_resolution_set(&mfr_resolution, sensor_num);
	// if (res_ret != 0) {
	// 	return SENSOR_UNSPECIFIED_ERROR;
	// }
	// //Victor test
	// LOG_WRN("mfr resolution = %x  \n", mfr_resolution);

	bool ret = false;
	uint8_t retry = 5;
	int val = 0;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = sensor_config[sensor_config_index_map[sensor_num]].offset;

	if (i2c_master_read(&msg, retry)) {
		/* read fail */
		return SENSOR_FAIL_TO_ACCESS;
	}

	//Victor test
	//LOG_WRN("mp2971_read val = %x  \n", val);

	uint8_t offset = sensor_config[sensor_config_index_map[sensor_num]].offset;
	val = (msg.data[1] << 8) | msg.data[0];

	// float reso = 0;
	sval->integer = (int16_t)(val / (1 / get_resolution(sensor_num)));
	sval->fraction = (int16_t)(val - (sval->integer * (1 / get_resolution(sensor_num)))) *
			 (get_resolution(sensor_num) / 0.001);

	switch (offset) {
	case PMBUS_READ_VOUT:
		/* 1 mV/LSB, unsigned integer */
		val = val & BIT_MASK(12);
		sval->integer = (int16_t)(val / (1 / get_resolution(sensor_num)));
		sval->fraction =
			(int16_t)(val - (sval->integer * (1 / get_resolution(sensor_num)))) *
			(get_resolution(sensor_num) / 0.001);
		break;
	case PMBUS_READ_IOUT:
		val = val & BIT_MASK(11);
		ret = vr_adjust_of_twos_complement(offset, &val);
		if (ret == false) {
			printf("[%s] adjust reading IOUT value failed - sensor number: 0x%x\n",
			       __func__, sensor_num);
			return SENSOR_UNSPECIFIED_ERROR;
		}

		sval->integer = (int16_t)(val / (1 / get_resolution(sensor_num)));
		sval->fraction =
			(int16_t)(val - (sval->integer * (1 / get_resolution(sensor_num)))) *
			(get_resolution(sensor_num) / 0.001);
		break;
	case PMBUS_READ_IIN:
		val = val & BIT_MASK(11);
		sval->integer = (int16_t)(val / (1 / get_resolution(sensor_num)));
		sval->fraction =
			(int16_t)(val - (sval->integer * (1 / get_resolution(sensor_num)))) *
			(get_resolution(sensor_num) / 0.001);

		break;
	case PMBUS_READ_TEMPERATURE_1:
		val = val & BIT_MASK(8);
		sval->integer = (int16_t)(val / (1 / get_resolution(sensor_num)));
		sval->fraction =
			(int16_t)(val - (sval->integer * (1 / get_resolution(sensor_num)))) *
			(get_resolution(sensor_num) / 0.001);
		break;
	case PMBUS_READ_POUT:
		val = val & BIT_MASK(11);
		ret = vr_adjust_of_twos_complement(offset, &val);
		if (ret == false) {
			printf("[%s] adjust reading POUT value failed - sensor number: 0x%x\n",
			       __func__, sensor_num);
			return SENSOR_UNSPECIFIED_ERROR;
		}

		sval->integer = (int16_t)(val / (1 / get_resolution(sensor_num)));
		sval->fraction =
			(int16_t)(val - (sval->integer * (1 / get_resolution(sensor_num)))) *
			(get_resolution(sensor_num) / 0.001);

		break;
	default:
		printf("[%s] not support offset: 0x%x\n", __func__, offset);
		return SENSOR_FAIL_TO_ACCESS;
		break;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t mp2971_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	//Victor test

	sensor_config[sensor_config_index_map[sensor_num]].read = mp2971_read;
	return SENSOR_INIT_SUCCESS;
}