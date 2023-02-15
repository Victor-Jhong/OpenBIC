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

#ifndef PLAT_LED_H
#define PLAT_LED_H

#define IO_EXPENDER_BUS I2C_BUS5
#define IO_EXPENDER_ADDRESS (0xEE >> 1)
#define IO_EXPENDER_PORT1_OUTPUT_OFFSET 0x03
#define IO_EXPENDER_PORT1_CONFIG_OFFSET 0x07
#define IO_EXPENDER_PORT1_FAULT_LED_PIN 6
#define IO_EXPENDER_PORT1_POWER_LED_PIN 7

enum led_type {
	POWER_LED = 0,
	FAULT_LED = 1,
	LED_TYPE_MAX = 2,
};

enum system_led_status {
	SYS_LED_OFF = 0,
	SYS_LED_ON = 1,
	SYS_LED_MAX = 2,
};

enum system_led_user {
	SYS_LED_USER_BIC = 0,
	SYS_LED_USER_BMC = 1,
	SYS_LED_USER_MAX = 2,
};

void system_led_init();

bool fault_led_set(uint8_t func_user, uint8_t set_status);
bool power_led_set(uint8_t func_user, uint8_t set_status);
bool set_system_led(uint8_t led_type, uint8_t set_status, uint8_t func_user);

bool is_system_fault();
void check_light_fault_led();
void check_light_dc_on_led();

#endif