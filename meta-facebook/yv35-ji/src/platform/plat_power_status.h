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

#ifndef PLAT_POWER_STATUS_H
#define PLAT_POWER_STATUS_H

#include <stdint.h>
#include <stdlib.h>

void handle_post_status(bool status, bool need_change);
void handle_post_action();
void reset_post_end_work_status();
void handle_tda38741_work_around();
void power_status_monitor();
bool satmc_access(uint8_t sensor_num);
void set_satmc_status(bool status);
bool get_satmc_status();
bool retimer_access(uint8_t sensor_num);
bool get_retimer_status();
uint8_t scan_retimer_addr();
void retimer_addr_loss();
bool e1s_access(uint8_t sensor_num);

#endif
