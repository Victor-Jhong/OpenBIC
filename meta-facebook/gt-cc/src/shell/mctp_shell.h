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

#ifndef _MCTP_SHELL_H
#define _MCTP_SHELL_H

#ifdef __cplusplus
extern "C" {
#endif

/* shell command test */

void cmd_mctp_set_eid(const struct shell *shell, size_t argc, char **argv);
void cmd_mctp_get_eid(const struct shell *shell, size_t argc, char **argv);
void cmd_mctp_get_mctp_version_support(const struct shell *shell, size_t argc, char **argv);
void cmd_mctp_set_message_type_support(const struct shell *shell, size_t argc, char **argv);

void cmd_pldm_base_set_tid(const struct shell *shell, size_t argc, char **argv);
void cmd_pldm_base_get_tid(const struct shell *shell, size_t argc, char **argv);
void cmd_pldm_base_get_pldm_types(const struct shell *shell, size_t argc, char **argv);
void cmd_pldm_base_get_pldm_commands(const struct shell *shell, size_t argc, char **argv);

void cmd_pldm_mc_set_event_receiver(const struct shell *shell, size_t argc, char **argv);
void cmd_pldm_mc_event_message_buffer_size(const struct shell *shell, size_t argc, char **argv);
void cmd_pldm_mc_get_sensor_reading(const struct shell *shell, size_t argc, char **argv);
void cmd_pldm_mc_get_pdr(const struct shell *shell, size_t argc, char **argv);
void cmd_pldm_oem_check_pldm_mc_support_command(const struct shell *shell, size_t argc,
						char **argv);

/* Sub-command Level 2 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_mctp_control_cmds,
			       SHELL_CMD(set_eid, NULL, "set eid", cmd_mctp_set_eid),
			       SHELL_CMD(get_eid, NULL, "get eid", cmd_mctp_get_eid),
			       SHELL_CMD(get_mctp_version_support, NULL, "get mctp version support",
					 cmd_mctp_get_mctp_version_support),
			       SHELL_CMD(set_message_type_support, NULL, "set message type support",
					 cmd_mctp_set_message_type_support),
			       SHELL_SUBCMD_SET_END);

/* Sub-command Level 3 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_pldm_base_cmds, SHELL_CMD(set_tid, NULL, "set tid", cmd_pldm_base_set_tid),
	SHELL_CMD(get_tid, NULL, "get tid", cmd_pldm_base_get_tid),
	SHELL_CMD(get_pldm_types, NULL, "get pldm types", cmd_pldm_base_get_pldm_types),
	SHELL_CMD(get_pldm_commands, NULL, "get pldm commands", cmd_pldm_base_get_pldm_commands),
	SHELL_SUBCMD_SET_END);

/* Sub-command Level 3 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_pldm_monitor_control_cmds,
	SHELL_CMD(set_event_receiver, NULL, "set event receiver", cmd_pldm_mc_set_event_receiver),
	SHELL_CMD(event_message_buffer_size, NULL, "event message buffer size",
		  cmd_pldm_mc_event_message_buffer_size),
	SHELL_CMD(get_sensor_reading, NULL, "set event receiver", cmd_pldm_mc_get_sensor_reading),
	SHELL_CMD(get_pdr, NULL, "set event receiver", cmd_pldm_mc_get_pdr),
	SHELL_CMD(support_command, NULL, "set event receiver",
		  cmd_pldm_oem_check_pldm_mc_support_command),
	SHELL_SUBCMD_SET_END);

/* Sub-command Level 2 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_pldm_cmds,
			       SHELL_CMD(pldm_base, &sub_pldm_base_cmds,
					 "sub_pldm_base_cmds related command", NULL),
			       SHELL_CMD(pldm_monitor_control, &sub_pldm_monitor_control_cmds,
					 "pldm_monitor_control command", NULL),
			       SHELL_SUBCMD_SET_END);

#ifdef __cplusplus
}
#endif

#endif /* _MCTP_SHELL_H */