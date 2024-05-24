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

#include "fru.h"
#include "hal_i2c.h"
#include "hal_i3c.h"
#include "hal_wdt.h"
#include "ipmi.h"
#include "plat_def.h"

#ifdef CONFIG_IPMI_KCS_ASPEED
#include "kcs.h"
#endif

#include "sensor.h"
#ifdef ENABLE_PLDM_SENSOR
#include "pldm_sensor.h"
#endif
#ifdef ENABLE_PLDM
#include "pldm_smbios.h"
#endif
#include "timer.h"
#include "usb.h"
#include <logging/log.h>
#include <logging/log_ctrl.h>

__weak void pal_pre_init_i2c_test()
{
	return;
}

__weak void pal_pre_init()
{
	return;
}

__weak void pal_post_init()
{
	return;
}

__weak void pal_device_init()
{
	return;
}

__weak void pal_set_sys_status()
{
	return;
}

void main(void)
{
	printf("Hello, welcome to %s %s %x%x.%x.%x\n", PLATFORM_NAME, PROJECT_NAME, BIC_FW_YEAR_MSB,
	       BIC_FW_YEAR_LSB, BIC_FW_WEEK, BIC_FW_VER);

	wdt_init();
	util_init_timer();
	util_init_I2C();
	pal_pre_init_i2c_test();
	
		// while (1)
		// k_msleep(1000);

	// util_init_i3c();
	// printf("i3c init done\n");
	pal_pre_init();
	printf("pal pre init done\n");
	// sensor_init();
	// printf("sensor init done\n");
#ifdef ENABLE_PLDM_SENSOR
	// pldm_sensor_monitor_init();
	// printf("pldm sensor monitor init done\n");
#endif
#ifdef ENABLE_PLDM
	// pldm_smbios_init_structures();
	// printf("pldm smbios init done\n");
#endif
	// FRU_init();
	// printf("FRU init done\n");
	ipmi_init();
	printf("ipmi init done\n");
#ifdef CONFIG_USB
	// usb_dev_init();
	// printf("usb init done\n");
#endif
	// pal_device_init();
	// printf("pal device init done\n");
	// pal_set_sys_status();
	// printf("pal set sys status done\n");
	pal_post_init();
	printf("pal post init done\n");
}
