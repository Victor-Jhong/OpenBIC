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

#ifndef _PLAT_MCTP_h
#define _PLAT_MCTP_h

#include "storage_handler.h"

#define MCTP_RESP_DATA_INDEX 4
/* MCTP CC, Netfn, cmd, IPMI CC */
#define MCTP_RESP_HEADER_COUNT 4

/* init the mctp moduel for platform */
void plat_mctp_init(void);
void send_cmd_to_dev(struct k_timer *timer);
void send_cmd_to_dev_handler(struct k_work *work);

#endif /* _PLAT_MCTP_h */
