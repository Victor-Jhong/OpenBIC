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

#define IS_NORMAL_VAL true
#define IS_ABNORMAL_VAL false

uint16_t error_log_count(void);
void init_load_eeprom_log(void);

void plat_log_read(uint8_t *log_data, uint8_t cmd_size, uint16_t order);
void error_log_event(uint8_t sensor_num, bool val_normal);
void plat_clear_log();

typedef struct __attribute__((packed)) _plat_err_log_mapping {
	uint16_t index;
	uint16_t err_code;	
	uint64_t sys_time;
	uint8_t error_data[20];
	uint8_t cpld_dump[96];
} plat_err_log_mapping;

enum LOG_ERROR_CODE {
	LEAK_CHASSIS_0 = 0x0A,
	LEAK_CHASSIS_1 = 0x0B,
	LEAK_CHASSIS_2 = 0x0C,
	LEAK_CHASSIS_3 = 0x0D,
	LEAK_RPU_INT = 0x0E,
	LEAK_RPU_EXT = 0x0F,
	LEAK_MAN_HOT = 0x10,
	LEAK_MAN_COLD = 0x11,
	LEAK_MAN_PAN_GPO = 0x6E,
	LEAK_MAN_FLOOR_GPO = 0x6F,
	LEAK_MAN_PAN_RELAY = 0x70,
	LEAK_MAN_FLOOR_RELAY = 0x71,
	LOW_WATER_LEVEL = 0x02,
	PUMP_1_SPEED_ABNORMAL = 0x1F,
	PUMP_2_SPEED_ABNORMAL = 0x20,
	PUMP_3_SPEED_ABNORMAL = 0x21,
	PUMP_1_SPEED_RECOVER = 0x29,
	PUMP_2_SPEED_RECOVER = 0x2A,
	PUMP_3_SPEED_RECOVER = 0x2B,
	HIGH_PRESS_DETECTED = 0xA0,
	FLOW_RATE_SENSOR_TRIGGERED = 0xA1,
	EMERGENCY_BUTTON_TRIGGERED = 0xA2,
};

typedef struct _err_sensor_mapping {
	uint16_t err_code;
	uint8_t sen_num;
} err_sensor_mapping;