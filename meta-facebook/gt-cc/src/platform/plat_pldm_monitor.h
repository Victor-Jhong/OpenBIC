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

#ifndef PLAT_SDR_TABLE_H
#define PLAT_SDR_TABLE_H

//Victor test
enum oem_effecter_states_plat_led_value {
	EFFECTER_STATE_PLAT_LED_VALUE_UNKNOWN = 0x00,
	EFFECTER_STATE_PLAT_LED_VALUE_OFF,
	EFFECTER_STATE_PLAT_LED_VALUE_ON,
	EFFECTER_STATE_PLAT_LED_VALUE_MAX,
};
enum pldm_plat_effecter_id {
	PLAT_EFFECTER_ID_POWER_LED = 0x00,
	PLAT_EFFECTER_ID_FAULT_LED = 0x01,
};

#endif
