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

uint8_t pldm_set_state_effecter_states(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t *resp,
				       uint16_t *resp_len, void *ext_params)
{
	if (!mctp_inst || !buf || !resp || !resp_len)
		return PLDM_ERROR;

	struct pldm_set_state_effecter_states_req *req_p =
		(struct pldm_set_state_effecter_states_req *)buf;
	struct pldm_set_state_effecter_states_resp *res_p =
		(struct pldm_set_state_effecter_states_resp *)resp;

	//check effecter type is gpio (0xFF00~0xFFA8 represents GPIO 0~167)
	if ((req_p->effecter_id < 0xFF00) || (req_p->effecter_id > 0xFFA7)) {
		res_p->completion_code = INVALID_EFFECTER_ID;
		goto ret;
	}

	//check count
	if (req_p->composite_effecter_count != PLDM_STATE_EFFECTER_GPIO_COMPOSITE_EFFECTER_COUNT) {
		res_p->completion_code = INVALID_STATE_VALUE;
		goto ret;
	}

	//check length
	uint8_t pldm_set_state_effecter_enables_req_bytes =
		PLDM_SET_STATE_EFFECTER_ENABLES_REQ_NO_STATE_FIELD_BYTES +
		PLDM_SET_STATE_EFFECTER_ENABLES_REQ_STATE_FIELD_BYTES *
			req_p->composite_effecter_count;
	if ((len != pldm_set_state_effecter_enables_req_bytes) ||
	    (req_p->composite_effecter_count == 0)) {
		res_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		goto ret;
	}

	//check field 0
	if (req_p->pldm_state_field[0].pldm_set_request != GPIO_VALUE_SET_REQUEST_NO_CHANGE) {
		res_p->completion_code = UNSUPPORTED_EFFECTERSTATE;
		goto ret;
	}

	if ((req_p->pldm_state_field[0].pldm_effecter_states !=
	     GPIO_VALUE_EFFECTER_UNKNOWN_STATE) &&
	    (req_p->pldm_state_field[0].pldm_effecter_states != GPIO_DIR_EFFECTER_INPUT) &&
	    (req_p->pldm_state_field[0].pldm_effecter_states != GPIO_DIR_EFFECTER_OUTPUT)) {
		res_p->completion_code = UNSUPPORTED_EFFECTERSTATE;
		goto ret;
	}

	//check field 1
	if ((req_p->pldm_state_field[1].pldm_set_request != GPIO_VALUE_SET_REQUEST_NO_CHANGE) &&
	    (req_p->pldm_state_field[1].pldm_set_request != GPIO_VALUE_SET_REQUEST_REQUEST_SET)) {
		res_p->completion_code = UNSUPPORTED_EFFECTERSTATE;
		goto ret;
	}

	if ((req_p->pldm_state_field[1].pldm_effecter_states !=
	     GPIO_VALUE_EFFECTER_UNKNOWN_STATE) &&
	    (req_p->pldm_state_field[1].pldm_effecter_states != GPIO_VALUE_EFFECTER_LOW) &&
	    (req_p->pldm_state_field[1].pldm_effecter_states != GPIO_VALUE_EFFECTER_HIGH)) {
		res_p->completion_code = UNSUPPORTED_EFFECTERSTATE;
		goto ret;
	}

	if (req_p->pldm_state_field[1].pldm_set_request == GPIO_VALUE_SET_REQUEST_NO_CHANGE) {
		res_p->completion_code = PLDM_SUCCESS;
		goto ret;
	}

	if (req_p->pldm_state_field[1].pldm_effecter_states == GPIO_VALUE_EFFECTER_UNKNOWN_STATE) {
		res_p->completion_code = PLDM_SUCCESS;
		goto ret;
	}

	//check gpio
	uint8_t gpio_pin = req_p->effecter_id - 0xFF00;
	if ((gpio_pin < PLDM_STATE_EFFECTER_FIRST_GPIO_PIN) ||
	    (gpio_pin > PLDM_STATE_EFFECTER_LAST_GPIO_PIN)) {
		res_p->completion_code = INVALID_EFFECTER_ID;
		goto ret;
	}
	uint8_t gpio_dir = gpio_get_direction(gpio_pin);

	//check not to write input pin
	if (req_p->pldm_state_field[1].pldm_set_request == GPIO_VALUE_SET_REQUEST_REQUEST_SET &&
	    gpio_dir == GPIO_DIR_EFFECTER_INPUT - 0xF0) {
		res_p->completion_code = UNSUPPORTED_EFFECTERSTATE;
		goto ret;
	}

	//check gpio setting value is 1 or 0
	uint8_t gpio_value_effecter_state = req_p->pldm_state_field[1].pldm_effecter_states - 0xF0;
	if ((gpio_value_effecter_state != GPIO_VALUE_EFFECTER_LOW - 0xF0) &&
	    (gpio_value_effecter_state != GPIO_VALUE_EFFECTER_HIGH - 0xF0)) {
		res_p->completion_code = UNSUPPORTED_EFFECTERSTATE;
		goto ret;
	}

	//set gpio pin high low
	gpio_set(gpio_pin, gpio_value_effecter_state);
	res_p->completion_code = PLDM_SUCCESS;

ret:
	*resp_len = sizeof(struct pldm_set_state_effecter_states_resp);
	return PLDM_SUCCESS;
}

uint8_t pldm_get_state_effecter_states(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t *resp,
				       uint16_t *resp_len, void *ext_params)
{
	if (!mctp_inst || !buf || !resp || !resp_len)
		return PLDM_ERROR;

	struct pldm_get_state_effecter_states_req *req_p =
		(struct pldm_get_state_effecter_states_req *)buf;
	struct pldm_get_state_effecter_states_resp *res_p =
		(struct pldm_get_state_effecter_states_resp *)resp;

	//check length
	if (len != PLDM_GET_STATE_EFFECTER_STATES_REQ_BYTES) {
		//res_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		goto ret;
	}

	//check effecter type is gpio (0xFF00~0xFFA8 represents GPIO 0~167)
	if ((req_p->effecter_id < 0xFF00) || (req_p->effecter_id > 0xFFA7)) {
		res_p->completion_code = INVALID_EFFECTER_ID;
		printf("pldm_set completion_code = %x \n", res_p->completion_code);
		goto ret;
	}

	res_p->composite_effecter_count = PLDM_STATE_EFFECTER_GPIO_COMPOSITE_EFFECTER_COUNT;

	//gpio effecter's state is always enabled_noUpdatePending
	res_p->pldm_state_field[0].pldm_effecter_operational_state = enabled_noUpdatePending;
	res_p->pldm_state_field[1].pldm_effecter_operational_state = enabled_noUpdatePending;

	//no pending state in gpio effecter
	//check gpio
	uint8_t gpio_pin = req_p->effecter_id - 0xFF00;
	if ((gpio_pin < PLDM_STATE_EFFECTER_FIRST_GPIO_PIN) ||
	    (gpio_pin > PLDM_STATE_EFFECTER_LAST_GPIO_PIN)) {
		res_p->completion_code = INVALID_EFFECTER_ID;
		goto ret;
	}

	res_p->pldm_state_field[0].pldm_pending_state = gpio_get_direction(gpio_pin) + 0xF0;
	res_p->pldm_state_field[0].pldm_present_state = gpio_get_direction(gpio_pin) + 0xF0;

	res_p->pldm_state_field[1].pldm_pending_state = gpio_get(gpio_pin) + 0xF0;
	res_p->pldm_state_field[1].pldm_present_state = gpio_get(gpio_pin) + 0xF0;
	res_p->completion_code = PLDM_SUCCESS;

ret:

	*resp_len = sizeof(struct pldm_get_state_effecter_states_resp) +
		    sizeof(struct get_state_field) * res_p->composite_effecter_count - 3;

	if (res_p->completion_code != PLDM_SUCCESS) {
		*resp_len = sizeof(res_p->completion_code);
	}
	return PLDM_SUCCESS;
}

static pldm_cmd_handler pldm_monitor_cmd_tbl[] = {
	{ PLDM_MONITOR_CMD_CODE_GET_SENSOR_READING, pldm_get_sensor_reading },
	{ PLDM_MONITOR_CMD_CODE_SET_STATE_EFFECTER_STATES, pldm_set_state_effecter_states },
	{ PLDM_MONITOR_CMD_CODE_GET_STATE_EFFECTER_STATES, pldm_get_state_effecter_states },
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