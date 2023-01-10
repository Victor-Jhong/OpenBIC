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
#include <stdlib.h>
#include "libutil.h"
#include "pldm.h"
#include "plat_class.h"
#include "plat_pldm_monitor.h"

LOG_MODULE_REGISTER(plat_pldm_monitor);

#define PLDM_PLATFORM_POWER_LED_EFFECTER_STATE_FIELD_COUNT 1
#define PLDM_PLATFORM_FAULT_LED_EFFECTER_STATE_FIELD_COUNT 1

void plat_set_effecter_power_led_handler(uint8_t *buf, uint16_t len, uint8_t *resp,
					 uint16_t *resp_len)
{
	CHECK_NULL_ARG(buf);
	CHECK_NULL_ARG(resp);
	CHECK_NULL_ARG(resp_len);

	struct pldm_set_state_effecter_states_req *req_p =
		(struct pldm_set_state_effecter_states_req *)buf;
	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	if (req_p->composite_effecter_count != PLDM_PLATFORM_POWER_LED_EFFECTER_STATE_FIELD_COUNT) {
		LOG_ERR("Unsupport power led effecter count, (%d)",
			req_p->composite_effecter_count);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return;
	}

	set_effecter_state_field_t *power_led_val_state = &req_p->field[0];

	if (power_led_val_state->set_request >= PLDM_SET_REQUEST_MAX) {
		LOG_ERR("Unsupport power led effecter set request");
		*completion_code_p = PLDM_PLATFORM_UNSUPPORTED_EFFECTERSTATE;
		return;
	}

	if (power_led_val_state->effecter_state >= EFFECTER_STATE_PLAT_LED_VALUE_MAX) {
		LOG_ERR("Unsupport power led effecter state");
		*completion_code_p = PLDM_PLATFORM_INVALID_STATE_VALUE;
		return;
	}

	if (power_led_val_state->set_request == PLDM_NO_CHANGE) {
		*completion_code_p = PLDM_SUCCESS;
	} else {
		if (power_led_val_state->effecter_state == EFFECTER_STATE_PLAT_LED_VALUE_UNKNOWN) {
			*completion_code_p = PLDM_OEM_GPIO_EFFECTER_VALUE_UNKNOWN;
		} else {
			uint8_t power_led_val = ((power_led_val_state->effecter_state ==
						  EFFECTER_STATE_PLAT_LED_VALUE_OFF) ?
							 SYS_LED_OFF :
							 SYS_LED_ON);
			set_system_led(POWER_LED, power_led_val, SYS_LED_USER_BMC);
			*completion_code_p = PLDM_SUCCESS;
		}
	}
	return;
}

void plat_set_effecter_fault_led_handler(uint8_t *buf, uint16_t len, uint8_t *resp,
					 uint16_t *resp_len)
{
	CHECK_NULL_ARG(buf);
	CHECK_NULL_ARG(resp);
	CHECK_NULL_ARG(resp_len);

	struct pldm_set_state_effecter_states_req *req_p =
		(struct pldm_set_state_effecter_states_req *)buf;
	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	if (req_p->composite_effecter_count != PLDM_PLATFORM_FAULT_LED_EFFECTER_STATE_FIELD_COUNT) {
		LOG_ERR("Unsupport power led effecter count, (%d)",
			req_p->composite_effecter_count);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return;
	}

	set_effecter_state_field_t *fault_led_val_state = &req_p->field[0];

	if (fault_led_val_state->set_request >= PLDM_SET_REQUEST_MAX) {
		LOG_ERR("Unsupport power led effecter set request");
		*completion_code_p = PLDM_PLATFORM_UNSUPPORTED_EFFECTERSTATE;
		return;
	}

	if (fault_led_val_state->effecter_state >= EFFECTER_STATE_PLAT_LED_VALUE_MAX) {
		LOG_ERR("Unsupport power led effecter state");
		*completion_code_p = PLDM_PLATFORM_INVALID_STATE_VALUE;
		return;
	}

	if (fault_led_val_state->set_request == PLDM_NO_CHANGE) {
		*completion_code_p = PLDM_SUCCESS;
	} else {
		if (fault_led_val_state->effecter_state == EFFECTER_STATE_PLAT_LED_VALUE_UNKNOWN) {
			*completion_code_p = PLDM_OEM_GPIO_EFFECTER_VALUE_UNKNOWN;
		} else {
			uint8_t fault_led_val = ((fault_led_val_state->effecter_state ==
						  EFFECTER_STATE_PLAT_LED_VALUE_OFF) ?
							 SYS_LED_OFF :
							 SYS_LED_ON);
			set_system_led(FAULT_LED, fault_led_val, SYS_LED_USER_BMC);
			*completion_code_p = PLDM_SUCCESS;
		}
	}
	return;
}

void oem_set_effecter_type_plat_handler(uint8_t *buf, uint16_t len, uint8_t *resp,
					uint16_t *resp_len)
{
	CHECK_NULL_ARG(buf);
	CHECK_NULL_ARG(resp);
	CHECK_NULL_ARG(resp_len);

	struct pldm_set_state_effecter_states_req *req_p =
		(struct pldm_set_state_effecter_states_req *)buf;
	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	uint8_t plat_effecter_id = req_p->effecter_id & BIT_MASK(8);

	switch (plat_effecter_id) {
	case PLAT_EFFECTER_ID_POWER_LED:
		plat_set_effecter_power_led_handler(buf, len, resp, resp_len);
		break;
	case PLAT_EFFECTER_ID_FAULT_LED:
		plat_set_effecter_fault_led_handler(buf, len, resp, resp_len);
		break;
	default:
		LOG_ERR("Unsupport plat effecter ID, (%d)", plat_effecter_id);
		*completion_code_p = PLDM_PLATFORM_INVALID_EFFECTER_ID;
		break;
	}
}
