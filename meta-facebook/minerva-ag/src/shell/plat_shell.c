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

#include <stdlib.h>
#include "plat_sensor_polling_shell.h"
#include "log_shell.h"
#include "cpld_shell.h"
#include "plat_pldm_fw_version_shell.h"
#include "mctp.h"
#include "pldm.h"
#include "plat_aegis_power_control_shell.h"
#include "hal_i2c.h"
#include "plat_i2c.h"

// HBM data offset constants
#define HBM_BASE_OFFSET 0xD07C4000
#define HBM0_DATA_OFFSET 0x0EED
#define HBM1_DATA_OFFSET 0x1327
#define HBM2_DATA_OFFSET 0x1761
#define HBM3_DATA_OFFSET 0x1B9B
#define HBM4_DATA_OFFSET 0x1FD5
#define HBM5_DATA_OFFSET 0x240F

// I2C configuration for HBM
#define HBM_I2C_BUS I2C_BUS6
#define HBM_I2C_ADDR (0xD4 >> 1) // 7-bit address 0x6A
#define HBM_CHANNEL_DATA_SIZE 8 // 8 bytes per channel
#define HBM_MAX_CHANNELS 16
#define HBM_MAX_HBMS 6

void pldm_cmd(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 4) {
		shell_warn(shell, "Help: pldm <eid> <pldm_type> <pldm_cmd> <pldm_data>");
		return;
	}

	const uint8_t eid = strtol(argv[1], NULL, 16);

	uint8_t resp_buf[PLDM_MAX_DATA_SIZE] = { 0 };
	pldm_msg pmsg = { 0 };
	pmsg.hdr.msg_type = MCTP_MSG_TYPE_PLDM;
	pmsg.hdr.pldm_type = strtol(argv[2], NULL, 16);
	pmsg.hdr.cmd = strtol(argv[3], NULL, 16);
	pmsg.hdr.rq = PLDM_REQUEST;
	pmsg.len = argc - 4;
	uint8_t req_buf[pmsg.len];
	pmsg.buf = req_buf;

	for (int i = 0; i < pmsg.len; i++)
		pmsg.buf[i] = strtol(argv[i + 4], NULL, 16);

	mctp *mctp_inst = NULL;
	if (get_mctp_info_by_eid(eid, &mctp_inst, &pmsg.ext_params) == false) {
		shell_print(shell, "Failed to get mctp info by eid 0x%x", eid);
		return;
	}

	uint16_t resp_len = mctp_pldm_read(mctp_inst, &pmsg, resp_buf, sizeof(resp_buf));
	if (resp_len == 0) {
		shell_print(shell, "Failed to get mctp-pldm response");
		return;
	}

	shell_print(shell, "RESP");
	shell_hexdump(shell, resp_buf, resp_len);

	return;
}

/* Sub-command Level 1 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_test_cmds,
			       SHELL_CMD(sensor, &sub_plat_sensor_polling_cmd,
					 "set/get platform sensor polling command", NULL),
			       SHELL_CMD(log, &sub_plat_log_cmd, "platform log command", NULL),
			       SHELL_CMD(cpld, &sub_cpld_cmd, "cpld command", NULL),
			       SHELL_CMD(get_fw_version, &sub_get_fw_version_cmd,
					 "get fw version command", NULL),
			       SHELL_CMD(pldm, NULL, "send pldm to bmc", pldm_cmd),
			       SHELL_CMD(aegis_power, &sub_aegis_power_cmds, "aegis power commands",
					 NULL),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(test, &sub_test_cmds, "Test commands for AG", NULL);

// HBM offset lookup table
static const uint16_t hbm_data_offsets[HBM_MAX_HBMS] = { HBM0_DATA_OFFSET, HBM1_DATA_OFFSET,
							 HBM2_DATA_OFFSET, HBM3_DATA_OFFSET,
							 HBM4_DATA_OFFSET, HBM5_DATA_OFFSET };

/**
 * @brief Read HBM data from a specific HBM and channel
 * @param hbm HBM number (0-5)
 * @param channel Channel number (0-15)
 * @param data Buffer to store the 8 bytes of data
 * @return 0 on success, negative error code on failure
 */
static int read_hbm_data(uint8_t hbm, uint8_t channel, uint8_t *data)
{
	if (hbm >= HBM_MAX_HBMS || channel >= HBM_MAX_CHANNELS || !data) {
		return -1;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	uint32_t full_offset;

	// Calculate the full offset address
	full_offset = HBM_BASE_OFFSET + hbm_data_offsets[hbm] + (channel * HBM_CHANNEL_DATA_SIZE);

	// Setup I2C message for writing offset address
	i2c_msg.bus = HBM_I2C_BUS;
	i2c_msg.target_addr = HBM_I2C_ADDR;
	i2c_msg.tx_len = 4; // 4 bytes for 32-bit address
	i2c_msg.rx_len = 0;

	// Pack the 32-bit offset address into bytes (little-endian)
	i2c_msg.data[0] = (full_offset >> 0) & 0xFF;
	i2c_msg.data[1] = (full_offset >> 8) & 0xFF;
	i2c_msg.data[2] = (full_offset >> 16) & 0xFF;
	i2c_msg.data[3] = (full_offset >> 24) & 0xFF;

	// Write offset address to device
	if (i2c_master_write(&i2c_msg, retry)) {
		return -2; // Write offset failed
	}

	// Setup I2C message for reading data
	i2c_msg.tx_len = 0;
	i2c_msg.rx_len = HBM_CHANNEL_DATA_SIZE;

	// Read the 8 bytes of data
	if (i2c_master_read(&i2c_msg, retry)) {
		return -3; // Read data failed
	}

	// Copy data to output buffer
	memcpy(data, i2c_msg.data, HBM_CHANNEL_DATA_SIZE);

	return 0;
}

/**
 * @brief Print HBM data in decimal format
 * @param shell Shell instance
 * @param hbm HBM number
 * @param channel Channel number
 * @param data Raw data bytes
 */
static void print_hbm_data(const struct shell *shell, uint8_t hbm, uint8_t channel, uint8_t *data)
{
	shell_print(shell, "HBM%d Channel%d:", hbm, channel);
	shell_print(shell, "  max_temp_current: %d", data[0]);
	shell_print(shell, "  temp_current: %d", data[1]);
	shell_print(shell, "  min_temp_history: %d", data[2]);
	shell_print(shell, "  max_temp_history: %d", data[3]);
	shell_print(shell, "  sid0: %d", data[4]);
	shell_print(shell, "  sid1: %d", data[5]);
	shell_print(shell, "  sid2: %d", data[6]);
	shell_print(shell, "  sid3: %d", data[7]);
}

void athena_read_cmd(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(shell, "Usage: athena read <hbm 0~5> <chan 0~15>");
		shell_error(shell, "       athena read all");
		return;
	}

	if (strcmp(argv[1], "all") == 0) {
		// Read all HBMs and channels
		shell_print(shell, "Reading all HBM data...");

		for (uint8_t hbm = 0; hbm < HBM_MAX_HBMS; hbm++) {
			for (uint8_t channel = 0; channel < HBM_MAX_CHANNELS; channel++) {
				uint8_t data[HBM_CHANNEL_DATA_SIZE];
				int ret = read_hbm_data(hbm, channel, data);

				if (ret == 0) {
					print_hbm_data(shell, hbm, channel, data);
				} else {
					shell_error(shell,
						    "Failed to read HBM%d Channel%d (error: %d)",
						    hbm, channel, ret);
				}
			}
		}
	} else {
		// Read specific HBM and channel
		if (argc != 3) {
			shell_error(shell, "Usage: athena read <hbm 0~5> <chan 0~15>");
			return;
		}

		// Parse HBM number
		uint8_t hbm = strtol(argv[1], NULL, 10);
		if (hbm >= HBM_MAX_HBMS) {
			shell_error(shell, "Invalid HBM number. Must be 0~5");
			return;
		}

		// Parse channel number
		uint8_t channel = strtol(argv[2], NULL, 10);
		if (channel >= HBM_MAX_CHANNELS) {
			shell_error(shell, "Invalid channel number. Must be 0~15");
			return;
		}

		// Read the data
		uint8_t data[HBM_CHANNEL_DATA_SIZE];
		int ret = read_hbm_data(hbm, channel, data);

		if (ret == 0) {
			print_hbm_data(shell, hbm, channel, data);
		} else {
			const char *error_msg;
			switch (ret) {
			case -1:
				error_msg = "Invalid parameters";
				break;
			case -2:
				error_msg = "Failed to write offset address";
				break;
			case -3:
				error_msg = "Failed to read data";
				break;
			default:
				error_msg = "Unknown error";
				break;
			}
			shell_error(shell, "Failed to read HBM%d Channel%d: %s", hbm, channel,
				    error_msg);
		}
	}
}

/* Sub-command Level 1 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_athena_cmds,
			       SHELL_CMD(read, NULL, "read athena command", athena_read_cmd),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(athena, &sub_athena_cmds, "Athena commands for AG", NULL);