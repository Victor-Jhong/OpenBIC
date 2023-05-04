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

#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <shell/shell.h>

#include <zephyr.h>
#include <sys/printk.h>
#include <logging/log.h>
#include <logging/log_ctrl.h>
#include <stdlib.h>
#include "mctp.h"
#include "mctp_ctrl.h"
#include "pldm.h"
#include "ipmi.h"
#include "sensor.h"
#include "plat_hook.h"
#include "plat_mctp.h"
#include "plat_gpio.h"

#include "mctp_shell.h"

LOG_MODULE_REGISTER(mctp_shell);

/*MCTP control*/
#define SET_EID_RESP_BUF_SIZE 4
#define GET_EID_RESP_BUF_SIZE 4
// #define GET_MCTP_VERSION_SUPPORT_RESP_BUF_SIZE 1
// #define SET_MESSAGE_TYPE_SUPPORT_RESP_BUF_SIZE 1

/*PLDM base*/
#define SET_TID_RESP_BUF_SIZE 1
#define GET_TID_RESP_BUF_SIZE 2
#define GET_PLDM_TYPES_RESP_BUF_SIZE 9
#define GET_PLDM_COMMANDS_RESP_BUF_SIZE 33

/*PLDM monitor and control*/
#define SET_EVENT_RECEIVER_RESP_BUF_SIZE 1
#define EVENT_MESSAGE_BUFFER_SIZE_RESP_BUF_SIZE 3
#define GET_SENSOR_READING_RESP_BUF_SIZE 12

typedef enum {
	SET_EID = 0x01,
	GET_EID = 0x02,
	GET_MCTP_VERSION_SUPPORT = 0x04,
	SET_MESSAGE_TYPE_SUPPORT = 0x05,
} MCTP_CONTROL_COMMAND;

typedef enum {
	/*PLDM base*/
	SET_TID = 0x01,
	GET_TID = 0x02,
	GET_PLDM_TYPES = 0x04,
	GET_PLDM_COMMANDS = 0x05,

	/*PLDM monitor and control*/
	SET_EVENT_RECEIVER = 0x04,
	EVENT_MESSAGE_BUFFER_SIZE = 0x0D,
	GET_SENSOR_READING = 0x11,
	GET_PDR = 0x51,
} PLDM_COMMAND;

struct event_message_buffer_size_req {
	uint16_t event_receiver_max_buffer_size;
} __attribute__((packed));

struct event_message_buffer_size_resp {
	uint8_t completion_code;
	uint16_t terminus_max_buffer_size;
} __attribute__((packed));

struct pldm_set_event_receiver_resp {
	uint8_t completion_code;
} __attribute__((packed));

void cmd_mctp_set_eid(const struct shell *shell, size_t argc, char **argv)
{
	// if (argc != 3) {
	// 	shell_warn(shell, "[%s]: input parameter count is invalid", __func__);
	// 	return;
	// }
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(shell, "argv[%d]", cnt);
		shell_hexdump(shell, argv[cnt], strlen(argv[cnt]));
	}

	uint8_t eid = strtol(argv[1], NULL, 16);
	printf("set eid = %d ", eid);
	// shell_set_eid();
}

void cmd_mctp_get_eid(const struct shell *shell, size_t argc, char **argv)
{
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(shell, "argv[%d]", cnt);
		shell_hexdump(shell, argv[cnt], strlen(argv[cnt]));
	}
	return;
}

void cmd_mctp_get_mctp_version_support(const struct shell *shell, size_t argc, char **argv)
{
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(shell, "argv[%d]", cnt);
		shell_hexdump(shell, argv[cnt], strlen(argv[cnt]));
	}
	return;
}

void cmd_mctp_set_message_type_support(const struct shell *shell, size_t argc, char **argv)
{
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(shell, "argv[%d]", cnt);
		shell_hexdump(shell, argv[cnt], strlen(argv[cnt]));
	}
	return;
}

void cmd_pldm_base_set_tid(const struct shell *shell, size_t argc, char **argv)
{
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(shell, "argv[%d]", cnt);
		shell_hexdump(shell, argv[cnt], strlen(argv[cnt]));
	}
	return;
}

void cmd_pldm_base_get_tid(const struct shell *shell, size_t argc, char **argv)
{
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(shell, "argv[%d]", cnt);
		shell_hexdump(shell, argv[cnt], strlen(argv[cnt]));
	}
	return;
}

void cmd_pldm_base_get_pldm_types(const struct shell *shell, size_t argc, char **argv)
{
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(shell, "argv[%d]", cnt);
		shell_hexdump(shell, argv[cnt], strlen(argv[cnt]));
	}
	return;
}

void cmd_pldm_base_get_pldm_commands(const struct shell *shell, size_t argc, char **argv)
{
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(shell, "argv[%d]", cnt);
		shell_hexdump(shell, argv[cnt], strlen(argv[cnt]));
	}
	return;
}

void cmd_pldm_mc_set_event_receiver(const struct shell *shell, size_t argc, char **argv)
{
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(shell, "argv[%d]", cnt);
		shell_hexdump(shell, argv[cnt], strlen(argv[cnt]));
	}
	return;
}

void cmd_pldm_mc_event_message_buffer_size(const struct shell *shell, size_t argc, char **argv)
{
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(shell, "argv[%d]", cnt);
		shell_hexdump(shell, argv[cnt], strlen(argv[cnt]));
	}

	return;
}

void cmd_pldm_mc_get_sensor_reading(const struct shell *shell, size_t argc, char **argv)
{
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(shell, "argv[%d]", cnt);
		shell_hexdump(shell, argv[cnt], strlen(argv[cnt]));
	}

	uint8_t eid = strtol(argv[1], NULL, 16);
	uint16_t sensor_id_buf = strtol(argv[2], NULL, 16);
	uint8_t rearm_event_state_buf = strtol(argv[3], NULL, 16); //0

	shell_fprintf(shell, SHELL_NORMAL, "eid = %d \n", eid);

	uint8_t ret = MCTP_ERROR;
	uint8_t resp_len = 0;
	pldm_msg pmsg = { 0 };
	uint8_t resp_buf[GET_SENSOR_READING_RESP_BUF_SIZE] = { 0 };
	mctp *mctp_inst = NULL;

	ret = get_mctp_route_info(eid, (void **)&mctp_inst, &pmsg.ext_params);
	if (ret != MCTP_SUCCESS) {
		LOG_ERR("Invalid EID: 0x%x, unable to get route information", eid);
		return;
	}

	// Set PLDM header
	pmsg.hdr.msg_type = MCTP_MSG_TYPE_PLDM;
	pmsg.hdr.pldm_type = PLDM_TYPE_PLAT_MON_CTRL;
	pmsg.hdr.cmd = GET_SENSOR_READING;
	pmsg.hdr.rq = PLDM_REQUEST;

	//set req data
	struct pldm_get_sensor_reading_req req = { 0 };
	req.sensor_id = sensor_id_buf;
	req.rearm_event_state = rearm_event_state_buf;

	pmsg.buf = (uint8_t *)&req;
	pmsg.len = sizeof(req);

	// Send request to PLDM/MCTP thread and get response
	resp_len = mctp_pldm_read(mctp_inst, &pmsg, resp_buf, sizeof(resp_buf));
	if (resp_len == 0) {
		LOG_ERR("mctp pldm read fail, event message buffer size no response");
		return;
	}

	LOG_HEXDUMP_INF(resp_buf, resp_len, "get_sensor_reading response");
	struct pldm_get_sensor_reading_resp *resp = (struct pldm_get_sensor_reading_resp *)resp_buf;
	LOG_INF("get_sensor_reading completion code = 0x%x", resp->completion_code);

	if (resp->completion_code != 0)
		return;

	LOG_HEXDUMP_INF(resp->present_reading, (resp_len - 7), "get_sensor_reading sensor reading");

	return;
}

struct get_pdr_req {
	uint32_t record_handle;
	uint32_t data_transfer_handle;
	uint8_t transfer_operation_flag;
	uint16_t request_count;
	uint16_t record_change_number;
} __attribute__((packed));

struct get_pdr_resp {
	uint8_t completion_code;
} __attribute__((packed));

void cmd_pldm_mc_get_pdr(const struct shell *shell, size_t argc, char **argv)
{
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(shell, "argv[%d]", cnt);
		shell_hexdump(shell, argv[cnt], strlen(argv[cnt]));
	}

	uint8_t eid = strtol(argv[1], NULL, 16);
	uint32_t record_handle_buf = strtol(argv[2], NULL, 16);
	uint32_t data_transfer_handle_buf = strtol(argv[3], NULL, 16);
	uint8_t transfer_operation_flag_buf = strtol(argv[4], NULL, 16);
	uint16_t request_count_buf = strtol(argv[5], NULL, 16);
	uint16_t record_change_number_buf = strtol(argv[6], NULL, 16);

	shell_fprintf(shell, SHELL_NORMAL, "eid = %d \n", eid);

	uint8_t ret = MCTP_ERROR;
	uint8_t resp_len = 0;
	pldm_msg pmsg = { 0 };
	uint8_t resp_buf[256] = { 0 };
	mctp *mctp_inst = NULL;

	ret = get_mctp_route_info(eid, (void **)&mctp_inst, &pmsg.ext_params);
	if (ret != MCTP_SUCCESS) {
		LOG_ERR("Invalid EID: 0x%x, unable to get route information", eid);
		return;
	}

	// Set PLDM header
	pmsg.hdr.msg_type = MCTP_MSG_TYPE_PLDM;
	pmsg.hdr.pldm_type = PLDM_TYPE_PLAT_MON_CTRL;
	pmsg.hdr.cmd = GET_PDR;
	pmsg.hdr.rq = PLDM_REQUEST;

	//set req data
	struct get_pdr_req req = { 0 };
	req.record_handle = record_handle_buf;
	req.data_transfer_handle = data_transfer_handle_buf;
	req.transfer_operation_flag = transfer_operation_flag_buf;
	req.request_count = request_count_buf;
	req.record_change_number = record_change_number_buf;

	pmsg.buf = (uint8_t *)&req;
	pmsg.len = sizeof(req);

	// Send request to PLDM/MCTP thread and get response
	resp_len = mctp_pldm_read(mctp_inst, &pmsg, resp_buf, sizeof(resp_buf));
	if (resp_len == 0) {
		LOG_ERR("mctp pldm read fail, event message buffer size no response");
		return;
	}

	LOG_HEXDUMP_INF(resp_buf, resp_len, "get_sensor_reading response");

	// struct pldm_get_sensor_reading_resp *resp = (struct pldm_get_sensor_reading_resp *)resp_buf;
	// LOG_INF("get_sensor_reading completion code = 0x%x", resp->completion_code);
	// LOG_HEXDUMP_INF(resp->present_reading, (resp_len - 7), "get_sensor_reading sensor reading");

	return;
}

struct oem_check_pldm_mc_support_command_req {
	uint8_t test_req;
} __attribute__((packed));

struct oem_check_pldm_mc_support_command_resp {
	uint8_t completion_code;
} __attribute__((packed));

#define PLDM_MC_SUPPORT_COMMAND_MAX 0x20

void cmd_pldm_oem_check_pldm_mc_support_command(const struct shell *shell, size_t argc, char **argv)
{
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(shell, "argv[%d]", cnt);
		shell_hexdump(shell, argv[cnt], strlen(argv[cnt]));
	}

	uint8_t eid = strtol(argv[1], NULL, 16);

	shell_fprintf(shell, SHELL_NORMAL, "eid = %d \n", eid);

	uint8_t ret = MCTP_ERROR;
	uint8_t resp_len = 0;
	pldm_msg pmsg = { 0 };
	uint8_t resp_buf[10] = { 0 };
	mctp *mctp_inst = NULL;

	ret = get_mctp_route_info(eid, (void **)&mctp_inst, &pmsg.ext_params);
	if (ret != MCTP_SUCCESS) {
		LOG_ERR("Invalid EID: 0x%x, unable to get route information", eid);
		return;
	}

	// Set PLDM header
	pmsg.hdr.msg_type = MCTP_MSG_TYPE_PLDM;
	pmsg.hdr.pldm_type = PLDM_TYPE_PLAT_MON_CTRL;
	pmsg.hdr.rq = PLDM_REQUEST;

	//set req data
	struct oem_check_pldm_mc_support_command_req req = { 0 };
	req.test_req = 0x00;

	pmsg.buf = (uint8_t *)&req;
	pmsg.len = sizeof(req);

	for (uint8_t command_id = 0; command_id < PLDM_MC_SUPPORT_COMMAND_MAX; command_id++) {
		pmsg.hdr.cmd = command_id;

		// Send request to PLDM/MCTP thread and get response
		resp_len = mctp_pldm_read(mctp_inst, &pmsg, resp_buf, sizeof(resp_buf));
		if (resp_len == 0) {
			LOG_ERR("mctp pldm read fail, event message buffer size no response");
			return;
		}

		struct oem_check_pldm_mc_support_command_resp *resp =
			(struct oem_check_pldm_mc_support_command_resp *)resp_buf;

		// LOG_HEXDUMP_INF(resp_buf, 1, "response");

		if (resp->completion_code == 0) {
			shell_fprintf(shell, SHELL_NORMAL, "Command ID %d support \n", command_id);
		} else {
			shell_fprintf(shell, SHELL_NORMAL, "Command ID %d not support \n",
				      command_id);
		}
	}

	return;
}