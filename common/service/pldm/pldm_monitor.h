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

#ifndef _PLDM_MONITOR_H
#define _PLDM_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pldm.h"

/* command number of pldm type 0x02 : PLDM for platform monitor and control */
#define PLDM_MONITOR_CMD_CODE_GET_SENSOR_READING 0x11
#define PLDM_MONITOR_CMD_CODE_SET_STATE_EFFECTER_STATES 0x39
#define PLDM_MONITOR_CMD_CODE_GET_STATE_EFFECTER_STATES 0x3A

/* define size of request */
#define PLDM_GET_SENSOR_READING_REQ_BYTES 3

#define PLDM_MONITOR_SENSOR_SUPPORT_MAX 0xFF

enum pldm_sensor_readings_data_type {
	PLDM_SENSOR_DATA_SIZE_UINT8,
	PLDM_SENSOR_DATA_SIZE_SINT8,
	PLDM_SENSOR_DATA_SIZE_UINT16,
	PLDM_SENSOR_DATA_SIZE_SINT16,
	PLDM_SENSOR_DATA_SIZE_UINT32,
	PLDM_SENSOR_DATA_SIZE_SINT32
};

enum pldm_sensor_operational_state {
	PLDM_SENSOR_ENABLED,
	PLDM_SENSOR_DISABLED,
	PLDM_SENSOR_UNAVAILABLE,
	PLDM_SENSOR_STATUSUNKOWN,
	PLDM_SENSOR_FAILED,
	PLDM_SENSOR_INITIALIZING,
	PLDM_SENSOR_SHUTTINGDOWN,
	PLDM_SENSOR_INTEST
};

enum pldm_sensor_present_state {
	PLDM_SENSOR_UNKNOWN = 0x0,
	PLDM_SENSOR_NORMAL = 0x01,
	PLDM_SENSOR_WARNING = 0x02,
	PLDM_SENSOR_CRITICAL = 0x03,
	PLDM_SENSOR_FATAL = 0x04,
	PLDM_SENSOR_LOWERWARNING = 0x05,
	PLDM_SENSOR_LOWERCRITICAL = 0x06,
	PLDM_SENSOR_LOWERFATAL = 0x07,
	PLDM_SENSOR_UPPERWARNING = 0x08,
	PLDM_SENSOR_UPPERCRITICAL = 0x09,
	PLDM_SENSOR_UPPERFATAL = 0x0a
};

enum pldm_sensor_event_message_enable {
	PLDM_NO_EVENT_GENERATION,
	PLDM_EVENTS_DISABLED,
	PLDM_EVENTS_ENABLED,
	PLDM_OP_EVENTS_ONLY_ENABLED,
	PLDM_STATE_EVENTS_ONLY_ENABLED
};

enum pldm_platform_completion_codes {
	PLDM_PLATFORM_INVALID_SENSOR_ID = 0x80,
	PLDM_PLATFORM_REARM_UNAVAILABLE_IN_PRESENT_STATE = 0x81,

};

struct pldm_get_sensor_reading_req {
	uint16_t sensor_id;
	uint8_t rearm_event_state;
} __attribute__((packed));

struct pldm_get_sensor_reading_resp {
	uint8_t completion_code;
	uint8_t sensor_data_size;
	uint8_t sensor_operational_state;
	uint8_t sensor_event_message_enable;
	uint8_t present_state;
	uint8_t previous_state;
	uint8_t event_state;
	uint8_t present_reading[1];
} __attribute__((packed));

uint8_t pldm_monitor_handler_query(uint8_t code, void **ret_fn);

//******************************************************************//
//Victor test 														//
//******************************************************************//

//******************************************************************//
//SetStateEffecterStates command									//
//******************************************************************//

/* define size of request */
#define PLDM_SET_STATE_EFFECTER_ENABLES_REQ_NO_STATE_FIELD_BYTES 3
#define PLDM_SET_STATE_EFFECTER_ENABLES_REQ_STATE_FIELD_BYTES 2

#define PLDM_GET_STATE_EFFECTER_STATES_REQ_BYTES 2

#define PLDM_STATE_EFFECTER_FIRST_GPIO_PIN 0
#define PLDM_STATE_EFFECTER_LAST_GPIO_PIN 167

#define PLDM_STATE_EFFECTER_GPIO_COMPOSITE_EFFECTER_COUNT 2

enum pldm_platform_effecter_completion_codes {
	INVALID_EFFECTER_ID = 0x80,
	INVALID_STATE_VALUE = 0x81,
	UNSUPPORTED_EFFECTERSTATE = 0x82
};

enum oem_gpio_dir_set_request {
	GPIO_DIR_SET_REQUEST_NO_CHANGE = 0x00,
	GPIO_DIR_SET_REQUEST_REQUEST_SET = 0x01
};

enum oem_gpio_value_set_request {
	GPIO_VALUE_SET_REQUEST_NO_CHANGE = 0x00,
	GPIO_VALUE_SET_REQUEST_REQUEST_SET = 0x01
};

enum oem_gpio_dir_effecter_states {
	GPIO_DIR_EFFECTER_UNKNOWN_STATE = 0x00,
	GPIO_DIR_EFFECTER_INPUT = 0xF0,
	GPIO_DIR_EFFECTER_OUTPUT = 0xF1
};

enum oem_gpio_value_effecter_states {
	GPIO_VALUE_EFFECTER_UNKNOWN_STATE = 0x00,
	GPIO_VALUE_EFFECTER_LOW = 0xF0,
	GPIO_VALUE_EFFECTER_HIGH = 0xF1
};

typedef struct set_state_field {
	uint8_t pldm_set_request;
	uint8_t pldm_effecter_states;
} set_state_field_t;

struct pldm_set_state_effecter_states_req {
	uint16_t effecter_id;
	uint8_t composite_effecter_count;
	set_state_field_t pldm_state_field[1];
} __attribute__((packed));

struct pldm_set_state_effecter_states_resp {
	uint8_t completion_code;
} __attribute__((packed));

//******************************************************************//
//GetStateEffecterStates command									//
//******************************************************************//

enum effecter_operational_state {
	enabled_updatePending,
	enabled_noUpdatePending,
	disabled,
	unavailable,
	statusUnknown,
	failed,
	initializing,
	shuttingDown,
	inTest
};

typedef struct get_state_field {
	uint8_t pldm_effecter_operational_state;
	uint8_t pldm_pending_state;
	uint8_t pldm_present_state;
} get_state_field_t;

struct pldm_get_state_effecter_states_req {
	uint16_t effecter_id;
} __attribute__((packed));

struct pldm_get_state_effecter_states_resp {
	uint8_t completion_code;
	uint8_t composite_effecter_count;
	get_state_field_t pldm_state_field[1];
} __attribute__((packed));

#ifdef __cplusplus
}
#endif

#endif /* _PLDM_MONITOR_H */