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
#include <shell/shell.h>
#include "plat_shell_e1s.h"
#include "mctp_shell.h"

/* Sub-command Level 2 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_e1s_cmds,
			       SHELL_CMD(power, NULL, "Stress E1S power consumption",
					 cmd_stress_e1s_pwr),
			       SHELL_SUBCMD_SET_END);

/* Sub-command Level 1 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_test_cmds,
			       SHELL_CMD(e1s, &sub_e1s_cmds, "E1S related command", NULL),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(test, &sub_test_cmds, "Test commands for GT", NULL);

/* shell command test */

/* Sub-command Level 1 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_mctp_cmds,
			       SHELL_CMD(mctp_control, &sub_mctp_control_cmds,
					 "mctp_control related command", NULL),
			       SHELL_CMD(pldm, &sub_pldm_cmds, "pldm related command", NULL),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(mctp, &sub_mctp_cmds, "mctp commands for GT", NULL);
