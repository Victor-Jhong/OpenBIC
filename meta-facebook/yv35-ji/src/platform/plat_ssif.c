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

#include <stdint.h>
#include "plat_ssif.h"
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "plat_mctp.h"
#include "plat_power_status.h"
#include "plat_fru.h"
#include "power_status.h"
#include "ssif.h"
#include "libutil.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(plat_ssif);

struct ssif_init_cfg ssif_cfg_table[] = {
	{ SSIF_I2C_BUS, SSIF_I2C_ADDR, 0x0A },
};

void pal_ssif_alert_trigger(uint8_t status)
{
	LOG_DBG("trigger %d", status);
	gpio_set(I2C_SSIF_ALERT_L, status);
}

void ssif_init(void)
{
	ssif_device_init(ssif_cfg_table, ARRAY_SIZE(ssif_cfg_table));

	if (ssif_inst_get_by_bus(SSIF_I2C_BUS))
		LOG_INF("SSIF ready!");
}

void pal_bios_post_complete()
{
	/* Pull low virtual bios complete pin */
	handle_post_status(GPIO_LOW, true);
	handle_post_action();
}

const uint8_t clear_cmos_sel_data[] = { 0x00, 0x00, 0xfb, 0x28, 0x00, 0x00, 0x00, 0x00,
					0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
const uint16_t clear_cmos_sel_size = ARRAY_SIZE(clear_cmos_sel_data);

void pal_add_sel_handler(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != clear_cmos_sel_size)
		return;

	if (!memcmp(msg->data, clear_cmos_sel_data, clear_cmos_sel_size)) {
		/* check whether need to clear fake rtc clear flag */
		LOG_WRN("Clear CMOS SEL event received, clear CMOS CLR flag");
		access_rtc_clr_flag(RTC_CLR_DEASSERT);
	}

	return;
}
