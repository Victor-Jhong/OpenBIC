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

#ifndef _PLDM_MONITOR_H
#define _PLDM_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pldm.h"

uint8_t pldm_monitor_handler_query(uint8_t code, void **ret_fn);

/*
uint8_t oem_set_effecter_type_gpio_handler(uint8_t *buf, uint16_t len, uint8_t *resp,
					   uint16_t *resp_len);

uint8_t oem_get_effecter_type_gpio_handler(uint8_t *buf, uint16_t len, uint8_t *resp,
					   uint16_t *resp_len);

*/

#ifdef __cplusplus
}
#endif

#endif /* _PLDM_MONITOR_H */