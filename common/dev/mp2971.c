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

float get_resolution(uint8_t sensor_num)
{
	uint8_t page = 0;
	uint16_t mfr_reso_set = 0;

	I2C_MSG msg;
	uint8_t retry = 5;

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = PAGE;

	//get page
	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("i2c read failed.\n");
		return SENSOR_FAIL_TO_ACCESS;
	}

	page = msg.data[0];

	//Victor test, check if it is need to do twice
	//msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	//msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	//msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = MFR_RESO_SET;

	//get reso set
	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("i2c read failed.\n");
		return SENSOR_FAIL_TO_ACCESS;
	}

	mfr_reso_set = (msg.data[1] << 8) | msg.data[0];

	uint8_t vout_reso_set;
	uint8_t iout_reso_set;
	uint8_t iin_reso_set;
	uint8_t pout_reso_set;

	float vout_reso = 0;
	float iout_reso = 0;
	float iin_reso = 0;
	float pout_reso = 0;
	float temp_reso = 1;

	//get reso from MFR_RESO_SET(C7h)
	if (page == 0) {
		vout_reso_set = (mfr_reso_set & GENMASK(7, 6)) >> 6;
		iout_reso_set = (mfr_reso_set & GENMASK(5, 4)) >> 4;
		iin_reso_set = (mfr_reso_set & GENMASK(3, 2)) >> 2;
		pout_reso_set = (mfr_reso_set & GENMASK(1, 0));

		if (vout_reso_set == 1) {
			vout_reso = 0.001;
		} else if (vout_reso_set == 2) {
			vout_reso = 0.001;
		} else {
			LOG_WRN("not supported vout_reso_set: 0x%x\n", vout_reso_set);
		}

		if (iout_reso_set == 0) {
			iout_reso = 2;
		} else if (iout_reso_set == 1) {
			iout_reso = 1;
		} else if (iout_reso_set == 2) {
			iout_reso = 0.5;
		} else {
			LOG_WRN("not supported vout_reso_set: 0x%x\n", iout_reso_set);
		}

		if (iin_reso_set == 0) {
			iin_reso = 0.5;
		} else if (iin_reso_set == 1) {
			iin_reso = 0.25;
		} else if (iin_reso_set == 2) {
			iin_reso = 0.125;
		} else {
			LOG_WRN("not supported vout_reso_set: 0x%x\n", iin_reso_set);
		}

		if (pout_reso_set == 0) {
			pout_reso = 2;
		} else if (pout_reso_set == 1) {
			pout_reso = 1;
		} else if (pout_reso_set == 2) {
			pout_reso = 0.5;
		} else {
			LOG_WRN("not supported vout_reso_set: 0x%x\n", pout_reso_set);
		}

	} else if (page == 1) {
		vout_reso_set = (mfr_reso_set & GENMASK(4, 3)) >> 3;
		iout_reso_set = (mfr_reso_set & GENMASK(2, 2)) >> 2;
		//iin_reso_set = 0;
		pout_reso_set = (mfr_reso_set & GENMASK(0, 0));

		if (vout_reso_set == 2 || vout_reso_set == 3) {
			vout_reso = 0.001;
		} else {
			LOG_WRN("not supported vout_reso_set: 0x%x\n", vout_reso_set);
		}

		if (iout_reso_set == 0) {
			iout_reso = 0.5;
		} else if (iout_reso_set == 1) {
			iout_reso = 0.25;
		} else {
			LOG_WRN("not supported vout_reso_set: 0x%x\n", iout_reso_set);
		}

		//Victor test, must be checked
		if (1) {
			iin_reso = 0.125;
		} else {
			LOG_WRN("not supported vout_reso_set: 0x%x\n", iin_reso_set);
		}

		if (pout_reso_set == 0) {
			pout_reso = 0.5;
		} else if (pout_reso_set == 1) {
			pout_reso = 0.25;
		} else {
			LOG_WRN("not supported vout_reso_set: 0x%x\n", pout_reso_set);
		}
	} else {
		LOG_WRN("not support page: 0x%d\n", page);
	}

	uint8_t offset = sensor_config[sensor_config_index_map[sensor_num]].offset;

	switch (offset) {
	case PMBUS_READ_VOUT:
		return vout_reso;
		break;
	case PMBUS_READ_IOUT:
		return iout_reso;
		break;
	case PMBUS_READ_IIN:
		return iin_reso;
		break;
	case PMBUS_READ_TEMPERATURE_1:
		return temp_reso;
		break;
	case PMBUS_READ_POUT:
		return pout_reso;
		break;
	default:
		LOG_WRN("not support offset: 0x%x\n", offset);
		break;
	}
	return 0;
}

bool vr_adjust_of_twos_complement(uint8_t offset, int *val)
{
	if (val == NULL) {
		LOG_WRN("input value is NULL\n");
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
		LOG_WRN("not support offset: 0x%x\n", offset);
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
			LOG_WRN("adjust reading IOUT value failed - sensor number: 0x%x\n",
				sensor_num);
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
			LOG_WRN("adjust reading POUT value failed - sensor number: 0x%x\n",
				sensor_num);
			return SENSOR_UNSPECIFIED_ERROR;
		}

		sval->integer = (int16_t)(val / (1 / get_resolution(sensor_num)));
		sval->fraction =
			(int16_t)(val - (sval->integer * (1 / get_resolution(sensor_num)))) *
			(get_resolution(sensor_num) / 0.001);

		break;
	default:
		LOG_WRN("not support offset: 0x%x\n", offset);
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

	sensor_config[sensor_config_index_map[sensor_num]].read = mp2971_read;
	return SENSOR_INIT_SUCCESS;
}