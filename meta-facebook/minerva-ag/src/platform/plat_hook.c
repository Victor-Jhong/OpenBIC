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

#include <stdio.h>
#include <string.h>
#include "libutil.h"
#include "ast_adc.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "plat_gpio.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_hook);

#define VR_MUTEX_LOCK_TIMEOUT_MS 1000

static struct k_mutex vr_mutex[VR_MAX_NUM];

vr_pre_proc_arg vr_pre_read_args[] = {
	// mutex, vr_page
	// P0V85
	[0] = { vr_mutex, 0x0 },
	[1] = { vr_mutex, 0x1 },

	// P0V75_CH_N
	[2] = { vr_mutex + 1, 0x0 },
	[3] = { vr_mutex + 1, 0x1 },

	// P0V75_CH_S
	[4] = { vr_mutex + 2, 0x0 },
	[5] = { vr_mutex + 2, 0x1 },

	// P0V75_TRVDD_ZONEA
	[6] = { vr_mutex + 3, 0x0 },
	[7] = { vr_mutex + 3, 0x1 },

	// P0V75_TRVDD_ZONEB
	[8] = { vr_mutex + 4, 0x0 },
	[9] = { vr_mutex + 4, 0x1 },

	// P1V1_VDDC_HBM0_HBM2_HBM4
	[10] = { vr_mutex + 5, 0x0 },
	[11] = { vr_mutex + 5, 0x1 },

	// P0V9_TRVDD_ZONEA
	[12] = { vr_mutex + 6, 0x0 },
	[13] = { vr_mutex + 6, 0x1 },

	// P0V9_TRVDD_ZONEB
	[14] = { vr_mutex + 7, 0x0 },
	[15] = { vr_mutex + 7, 0x1 },

	// P1V1_VDDC_HBM1_HBM3_HBM5
	[16] = { vr_mutex + 8, 0x0 },
	[17] = { vr_mutex + 8, 0x1 },

	// VDDA_PCIE
	[18] = { vr_mutex + 9, 0x0 },
	[19] = { vr_mutex + 9, 0x1 },

	// OSFP_P3V3
	[20] = { vr_mutex + 10, 0x0 },
	[21] = { vr_mutex + 10, 0x1 }
};

mp2971_init_arg mp2971_init_args[] = {
	[0] = { .vout_scale_enable = true },
};

isl69259_init_arg isl69259_init_args[] = {
	[0] = { .vout_scale_enable = true, .vout_scale = (499 / 798.8) },
};

void *vr_mutex_get(enum VR_INDEX_E vr_index)
{
	if (vr_index >= VR_MAX_NUM) {
		LOG_ERR("vr_mutex_get, invalid vr_index %d", vr_index);
		return NULL;
	}

	return vr_mutex + vr_index;
}

bool pre_vr_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)args;
	uint8_t retry = 5;
	I2C_MSG msg;

	/* mutex lock */
	if (pre_proc_args->mutex) {
		LOG_DBG("sen %x, mutex lock %p", cfg->num, pre_proc_args->mutex);
		if (k_mutex_lock(pre_proc_args->mutex, K_MSEC(VR_MUTEX_LOCK_TIMEOUT_MS))) {
			LOG_ERR("pre_vr_read, mutex lock fail");
			return false;
		}
	}

	/* set page */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = pre_proc_args->vr_page;
	if (i2c_master_write(&msg, retry)) {
		k_mutex_unlock(pre_proc_args->mutex);
		LOG_ERR("pre_vr_read, set page fail");
		return false;
	}
	return true;
}

bool post_vr_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);
	ARG_UNUSED(reading);

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)args;
		
	/* mutex unlock */
	if (pre_proc_args->mutex) {
		LOG_DBG("sen %x, mutex unlock %p", cfg->num, pre_proc_args->mutex);
		if (k_mutex_unlock(pre_proc_args->mutex)) {
			LOG_ERR("post_vr_read, mutex unlock fail");
			return false;
		}
	}

	return true;
}

bool is_mb_dc_on()
{
	/* RST_ATH_PWR_ON_PLD_R1_N is low active,
   * 1 -> power on
   * 0 -> power off
   */
	return gpio_get(RST_ATH_PWR_ON_PLD_R1_N);
}

void vr_mutex_init(void)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(vr_mutex); i++) {
		k_mutex_init(vr_mutex + i);
		LOG_DBG("vr_mutex[%d] %p init", i, vr_mutex + i);
	}
}