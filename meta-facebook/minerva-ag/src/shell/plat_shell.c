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
#include "plat_log.h"
#include <stdio.h>
#include "cpld_shell.h"
#include "plat_pldm_fw_version_shell.h"
#include "plat_fru.h"

void cmd_set_event(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: test log set_event <num> <status>");
		return;
	}

	int num = strtol(argv[1], NULL, 16);
	int status = strtol(argv[2], NULL, 16);

	error_log_event(num, status);

	return;
}

void cmd_log_dump(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: test log log_dump <order>, order start from 1");
		return;
	}

	// int cmd_size = strtol(argv[1], NULL, 16);
	int order = strtol(argv[1], NULL, 16);

	if (order < 1) {
		shell_warn(shell, "Help: test log log_dump <order>, order start from 1");
		return;
	}

	uint8_t log_data[128] = { 0 };
	plat_log_read(log_data, AEGIS_FRU_LOG_SIZE, order);

	shell_hexdump(shell, log_data, sizeof(uint8_t) * AEGIS_FRU_LOG_SIZE);

	return;
}

void cmd_test_read(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 4) {
		shell_warn(shell, "Help: test log test_read <offset_1> <offset_2> <length>");
		return;
	}

	uint16_t offset = ((strtol(argv[1], NULL, 16)) << 8) | (strtol(argv[2], NULL, 16));
	printf("offset = 0x%04X\n", offset);

	int length = strtol(argv[3], NULL, 16);

	uint8_t log_data[128] = { 0 };
	plat_eeprom_read(offset, log_data, length);
	printf("AEGIS_FRU_LOG_SIZE = %d\n", AEGIS_FRU_LOG_SIZE);

	shell_hexdump(shell, log_data, sizeof(uint8_t) * AEGIS_FRU_LOG_SIZE);

	return;
}

void cmd_log_clear(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: test log log_clear");
		return;
	}

	k_msleep(1000);

	plat_clear_log();
	shell_print(shell, "plat_clear_log finished!");

	return;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_plat_log_cmd,
			       SHELL_CMD(set_event, NULL, "set_event", cmd_set_event),
			       SHELL_CMD(dump, NULL, "log_dump", cmd_log_dump),
			       SHELL_CMD(clear, NULL, "log_clear", cmd_log_clear),
			       SHELL_CMD(test_read, NULL, "test_read", cmd_test_read),
			       SHELL_SUBCMD_SET_END);

/* Sub-command Level 1 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_test_cmds,
			       SHELL_CMD(sensor, &sub_plat_sensor_polling_cmd,
					 "set/get platform sensor polling command", NULL),
			       SHELL_CMD(cpld, &sub_cpld_cmd, "cpld command", NULL),
			       SHELL_CMD(get_fw_version, &sub_get_fw_version_cmd,
					 "get fw version command", NULL),
			       SHELL_CMD(log, &sub_plat_log_cmd, "platform log command", NULL),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(test, &sub_test_cmds, "Test commands for AG", NULL);
