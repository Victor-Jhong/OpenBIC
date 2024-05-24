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

#include "util_worker.h"
#include "ipmi.h"
#include "sensor.h"
#include "power_status.h"
#include "expansion_board.h"

#include "plat_sensor_table.h"
#include "plat_class.h"
#include "plat_hwmon.h"
#include "plat_m2.h"
#include "plat_isr.h"
#include "plat_power_seq.h"
#include "plat_led.h"
#include "plat_gpio.h"
#include "plat_util.h"
#include "plat_i2c.h"
#include "plat_i2c_target.h"
#include "plat_mctp.h"
#include "plat_pldm_monitor.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_init);

SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e6e2610, 0x0E000100 },
	{ 0x7e6e2614, 0x00006000 },
	{ 0x7e6e2634, 0x0000008F },
};

extern uint8_t ina230_init(sensor_cfg *cfg);
static void BICup5secTickHandler(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(up_1sec_handler, BICup1secTickHandler);
K_WORK_DELAYABLE_DEFINE(up_5sec_handler, BICup5secTickHandler);

void pal_pre_init_i2c_test()
{
	/* init i2c target */
	for (int index = 0; index < MAX_TARGET_NUM; index++) {
		if (I2C_TARGET_ENABLE_TABLE[index])
			i2c_target_control(
				index, (struct _i2c_target_config *)&I2C_TARGET_CONFIG_TABLE[index],
				1);
	}
}

void pal_pre_init()
{
	init_platform_config();
	init_e1s_config();
	init_worker(); // init util_worker
	scu_init(scu_cfg, ARRAY_SIZE(scu_cfg));
}

static void BICup5secTickHandler(struct k_work *work)
{
	if (!work) {
		LOG_ERR("BICup5secTickHandler get NULL work handler!");
		return;
	}

	if (!sensor_config) {
		LOG_ERR("sensor_config is NULL!");
		return;
	}

	uint8_t sensor_config_num = sensor_config_count;
	if (get_e1s_adc_config() == CONFIG_ADC_INA231) {
		/* config on-board INA231 */
		for (int i = 0; i < sensor_config_num; i++) {
			if (sensor_config[i].type != sensor_dev_ina230)
				continue;

			if (!m2_prsnt(m2_sensornum2idx(sensor_config[i].num)))
				continue;

			if (ina230_init(sensor_config + i) != SENSOR_INIT_SUCCESS) {
				LOG_ERR("sensor_config[%02x].num = %02x re-init ina230 failed!, retry it after 5 seconds",
					i, sensor_config[i].num);
				k_work_schedule((struct k_work_delayable *)work, K_SECONDS(5));
				return;
			}
		}
	} else if (get_e1s_adc_config() == CONFIG_ADC_ISL28022) {
		// Nothing to do here now
	} else {
		// Nothing to do here now
	}
}

void pal_set_sys_status()
{
	pwr_related_pin_init();

	SSDLEDInit();

	delay_function(100, plat_set_dc_status, FM_POWER_EN, 0);
	set_exp_pwrgd_pin();

	check_irq_fault();
	// BIC up 1 sec handler
	k_work_schedule(&up_1sec_handler, K_SECONDS(1));
	// BIC up 5 sec handler
	k_work_schedule(&up_5sec_handler, K_SECONDS(5));
}

struct k_thread i2c_master_write_read;
K_KERNEL_STACK_MEMBER(i2c_master_write_read_stack, 2048);

void i2c_master_write_read_handler(void *arug0, void *arug1, void *arug2)
{
	k_msleep(1000); // delay 1 second to wait for drivers ready before start sensor polling

	while (1) {
		I2C_MSG msg = { 0 };

		msg.bus = I2C_BUS1; //I2C5A
		msg.target_addr = 0x20;
		msg.tx_len = 7;
		msg.rx_len = 0;
		msg.data[0] = 0xaa;
		msg.data[1] = 0xbb;
		msg.data[2] = 0xcc;
		msg.data[3] = 0xdd;
		msg.data[4] = 0xee;
		msg.data[5] = 0xff;
		msg.data[6] = 0x00;

		if (i2c_master_write(&msg, 5)) {
			LOG_WRN("i2c_master_write_read test failed1.");
		} else {
			LOG_INF("i2c p1");
		}

		msg.bus = I2C_BUS1; //I2C5A
		msg.target_addr = 0x20;
		msg.tx_len = 1;
		msg.rx_len = 10;
		msg.data[0] = 0x00;

		if (i2c_master_read(&msg, 5)) {
			LOG_WRN("i2c_master_write_read test failed2.");
		} else {
			LOG_INF("i2c p2");
		}

		msg.bus = I2C_BUS1; //I2C5A
		msg.target_addr = 0x20;
		msg.tx_len = 7;
		msg.rx_len = 0;
		msg.data[0] = 0x11;
		msg.data[1] = 0x22;
		msg.data[2] = 0x33;
		msg.data[3] = 0x44;
		msg.data[4] = 0x55;
		msg.data[5] = 0x66;
		msg.data[6] = 0x00;

		if (i2c_master_write(&msg, 5)) {
			LOG_WRN("i2c_master_write_read test failed3.");
		} else {
			LOG_INF("i2c p3");
		}

		msg.bus = I2C_BUS1; //I2C5A
		msg.target_addr = 0x20;
		msg.tx_len = 1;
		msg.rx_len = 10;
		msg.data[0] = 0x00;

		if (i2c_master_read(&msg, 5)) {
			LOG_WRN("i2c_master_write_read test failed4.");
		} else {
			LOG_INF("i2c p4");
		}

		k_msleep(1000);
	}
}

void i2c_master_write_read_init()
{
	k_thread_create(&i2c_master_write_read, i2c_master_write_read_stack,
			K_THREAD_STACK_SIZEOF(i2c_master_write_read_stack),
			i2c_master_write_read_handler, NULL, NULL, NULL, 0, 0, K_NO_WAIT);
	k_thread_name_set(&i2c_master_write_read, "sensor_poll");
	return;
}

#define I2C_MASTER_WRITE_READ_STRESS 0

void pal_post_init()
{
	plat_mctp_init();
	pldm_load_state_effecter_table(MAX_STATE_EFFECTER_IDX);
	pldm_assign_gpio_effecter_id(PLAT_EFFECTER_ID_GPIO_HIGH_BYTE);

#if I2C_MASTER_WRITE_READ_STRESS
	i2c_master_write_read_init();
	LOG_INF("i2c_master_write_read_init");
#endif
}

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
