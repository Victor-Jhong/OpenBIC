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
#include <string.h>
#include <stdio.h>
#include "sensor.h"
#include "pldm_monitor.h"
#include "hal_gpio.h"

uint8_t pldm_get_sensor_reading(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t *resp,
				uint16_t *resp_len, void *ext_params)
{
	if (!mctp_inst || !buf || !resp || !resp_len)
		return PLDM_ERROR;

	struct pldm_get_sensor_reading_req *req_p = (struct pldm_get_sensor_reading_req *)buf;
	struct pldm_get_sensor_reading_resp *res_p = (struct pldm_get_sensor_reading_resp *)resp;

	if (len != PLDM_GET_SENSOR_READING_REQ_BYTES) {
		res_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		res_p->sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;
		goto ret;
	}
	/* Only support one byte range of sensor number */
	if (req_p->sensor_id > PLDM_MONITOR_SENSOR_SUPPORT_MAX) {
		res_p->completion_code = PLDM_PLATFORM_INVALID_SENSOR_ID;
		res_p->sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;
		goto ret;
	}

	uint8_t sensor_number = (uint8_t)req_p->sensor_id;
	uint8_t status = -1;
	int reading = 0;

	status = get_sensor_reading(sensor_number, &reading, GET_FROM_CACHE);

	switch (status) {
	case SENSOR_READ_SUCCESS:
	case SENSOR_READ_ACUR_SUCCESS:
	case SENSOR_READ_4BYTE_ACUR_SUCCESS:
		res_p->completion_code = PLDM_SUCCESS;
		res_p->sensor_operational_state = PLDM_SENSOR_ENABLED;
		break;
	case SENSOR_NOT_ACCESSIBLE:
	case SENSOR_INIT_STATUS:
		res_p->completion_code = PLDM_SUCCESS;
		res_p->sensor_operational_state = PLDM_SENSOR_INITIALIZING;
		break;
	case SENSOR_POLLING_DISABLE:
		res_p->completion_code = PLDM_SUCCESS;
		res_p->sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;
		break;
	case SENSOR_NOT_FOUND:
		// request sensor number not found
		res_p->completion_code = PLDM_PLATFORM_INVALID_SENSOR_ID;
		res_p->sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;
		break;
	case SENSOR_FAIL_TO_ACCESS:
	case SENSOR_UNSPECIFIED_ERROR:
	default:
		res_p->completion_code = PLDM_SUCCESS;
		res_p->sensor_operational_state = PLDM_SENSOR_FAILED;
		break;
	}

ret:
	/* Only support 4-bytes unsinged sensor data */
	res_p->sensor_data_size = PLDM_SENSOR_DATA_SIZE_UINT32;
	res_p->sensor_event_message_enable = PLDM_EVENTS_DISABLED;
	res_p->previous_state =
		(res_p->completion_code == PLDM_SUCCESS) ? PLDM_SENSOR_NORMAL : PLDM_SENSOR_UNKNOWN;
	res_p->present_state =
		(res_p->completion_code == PLDM_SUCCESS) ? PLDM_SENSOR_NORMAL : PLDM_SENSOR_UNKNOWN;
	res_p->event_state =
		(res_p->completion_code == PLDM_SUCCESS) ? PLDM_SENSOR_NORMAL : PLDM_SENSOR_UNKNOWN;

	if ((res_p->completion_code != PLDM_SUCCESS) ||
	    (res_p->sensor_operational_state != PLDM_SENSOR_ENABLED))
		reading = -1;

	memcpy(res_p->present_reading, &reading, sizeof(reading));
	*resp_len = sizeof(struct pldm_get_sensor_reading_resp) + res_p->sensor_data_size - 1;
	return PLDM_SUCCESS;
}

static pldm_cmd_handler pldm_monitor_cmd_tbl[] = {
	{ PLDM_MONITOR_CMD_CODE_GET_SENSOR_READING, pldm_get_sensor_reading },
};

uint8_t pldm_monitor_handler_query(uint8_t code, void **ret_fn)
{
	if (!ret_fn)
		return PLDM_ERROR;

	pldm_cmd_proc_fn fn = NULL;
	uint8_t i;

	for (i = 0; i < ARRAY_SIZE(pldm_monitor_cmd_tbl); i++) {
		if (pldm_monitor_cmd_tbl[i].cmd_code == code) {
			fn = pldm_monitor_cmd_tbl[i].fn;
			break;
		}
	}

	*ret_fn = (void *)fn;
	return fn ? PLDM_SUCCESS : PLDM_ERROR;
}

//******************************************************************//
//Victor test 														//
//******************************************************************//

//******************************************************************//
//SetStateEffecterStates command									//
//******************************************************************//
uint8_t pldm_set_state_effecter_states(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t *resp,
				       uint16_t *resp_len, void *ext_params)
{
	if (!mctp_inst || !buf || !resp || !resp_len)
		return PLDM_ERROR;

	struct pldm_set_state_effecter_states_req *req_p =
		(struct pldm_set_state_effecter_states_req *)buf;
	struct pldm_set_state_effecter_states_resp *res_p =
		(struct pldm_set_state_effecter_states_resp *)resp;

	uint8_t pldm_set_state_effecter_enables_req_bytes =
		PLDM_SET_STATE_EFFECTER_ENABLES_REQ_NO_STATE_FIELD_BYTES +
		PLDM_SET_STATE_EFFECTER_ENABLES_REQ_STATE_FIELD_BYTES *
			req_p->composite_effecter_count;

#if 0
	if (len != pldm_set_state_effecter_enables_req_bytes ||
	    req_p->composite_effecter_count == 0) {
		res_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		goto ret;
	}
#endif

#if 0
	uint8_t composite_effecter_count = req_p->composite_effecter_count;
#endif
#if 0	
	struct set_state_field_t *req_field_p = buf->pldm_state_field[1];
#endif

	set_state_field_t oem_gpio_direction_set = req_p->state_field[0];

	if (oem_gpio_direction_set->pldm_set_request == no_change)
		goto field_2;
		//return >> direction can't set by this command

#if 0
	//set gpio dir
	if (oem_gpio_direction_set->pldm_effecter_states == unknown)
		goto 123123123;
	if (oem_gpio_direction_set->pldm_effecter_states == state_0)
		gpio_conf(gpio_num, GPIO_INPUT);
	else if (oem_gpio_direction_sett->pldm_effecter_states == state_1)
		gpio_conf(gpio_num, GPIO_OUTPUT);
		
	123123123:
#endif

field_2:
	set_state_field_t oem_gpio_value_state_set = req_p->state_field[1];

	if (oem_gpio_value_state_set->pldm_set_request == no_change ||
	    oem_gpio_value_state_set->pldm_effecter_states == unknown)
		goto ret;

	gpio_set(req_p->effecter_id - 0xFF00,
		 oem_gpio_value_state_set->pldm_effecter_states - 0xF0);

ret:
}

//******************************************************************//
//GetStateEffecterStates command									//
//******************************************************************//
uint8_t pldm_get_state_effecter_states()
{
}