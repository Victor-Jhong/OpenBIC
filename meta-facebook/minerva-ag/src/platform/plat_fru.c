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

#include <string.h>
#include "plat_i2c.h"
#include <libutil.h>
#include <stdlib.h>
#include <logging/log.h>
#include "fru.h"
#include "plat_fru.h"

LOG_MODULE_REGISTER(plat_fru);

#define AEGIS_FRU_START 0x0000
#define AEGIS_FRU_SIZE 0x0400 // size 1KB

#define AEGIS_CPLD_FRU_START 0x0000
#define AEGIS_CPLD_FRU_SIZE 0x0800
const EEPROM_CFG plat_fru_config[] = {
	{
		ROHM_BR24G512,
		LOG_EEPROM_ID,
		I2C_BUS12,
		LOG_EEPROM_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		FRU_SIZE,
	},
	{
		ROHM_BR24G512, // using CPLD UFM
		CPLD_EEPROM_ID,
		I2C_BUS5,
		CPLD_EEPROM_ADDR,
		FRU_DEV_ACCESS_BYTE,
		FRU_START,
		AEGIS_CPLD_FRU_SIZE,
	},
};

#define CHASSIS_CUSTOM_DATA_MAX 24
#define BOARD_CUSTOM_DATA_MAX 4

typedef struct {
	uint8_t chassis_type;
	char chassis_part_number[32];
	char chassis_serial_number[32];
	char chassis_custom_data[CHASSIS_CUSTOM_DATA_MAX][32];
} ChassisInfo;

typedef struct {
	uint8_t language;
	char board_mfg_date[32];
	char board_mfg[32];
	char board_product[32];
	char board_serial[32];
	char board_part_number[32];
	char board_fru_id[32];
	char board_custom_data[BOARD_CUSTOM_DATA_MAX][32];
} BoardInfo;

typedef struct {
	uint8_t language;
	char product_manufacturer[32];
	char product_name[32];
	char product_part_number[32];
	char product_version[32];
	char product_serial[32];
	char product_asset_tag[32];
	char product_fru_id[32];
} ProductInfo;

void pal_load_fru_config(void)
{
	memcpy(&fru_config, &plat_fru_config, sizeof(plat_fru_config));
}

bool plat_eeprom_write(uint32_t offset, uint8_t *data, uint16_t data_len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	LOG_DBG("plat_eeprom_write, offset: 0x%x, data_len: %d", offset, data_len);

	EEPROM_ENTRY entry;

	entry.offset = offset;
	entry.data_len = data_len;

	uint8_t fru_index = 0;
	if (!find_FRU_ID(LOG_EEPROM_ID, &fru_index)) {
		LOG_ERR("find_FRU_ID fail when write eeprom 0x%x ", offset);
		return false;
	}

	memcpy(entry.data, data, data_len);
	memcpy(&entry.config, &fru_config[fru_index], sizeof(fru_config[fru_index]));

	if (!eeprom_write(&entry)) {
		LOG_ERR("write eeprom 0x%x fail", offset);
		return false;
	}

	return true;
}

bool plat_eeprom_read(uint32_t offset, uint8_t *data, uint16_t data_len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);
	EEPROM_ENTRY entry;

	entry.offset = offset;
	entry.data_len = data_len;

	uint8_t fru_index = 0;
	if (!find_FRU_ID(LOG_EEPROM_ID, &fru_index)) {
		LOG_ERR("find_FRU_ID fail when read eeprom 0x%x ", offset);
		return false;
	}

	memcpy(&entry.config, &fru_config[fru_index], sizeof(fru_config[fru_index]));
	memset(entry.data, 0xFF, sizeof(entry.data));

	if (!eeprom_read(&entry)) {
		LOG_ERR("read eeprom 0x%x fail", offset);
		return false;
	}

	memcpy(data, entry.data, data_len);

	return true;
}

bool plat_cpld_fru_read(uint32_t offset, uint8_t *data, uint16_t data_len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);
	EEPROM_ENTRY entry;

	entry.offset = offset;
	entry.data_len = data_len;

	uint8_t fru_index = 0;
	if (!find_FRU_ID(CPLD_EEPROM_ID, &fru_index)) {
		LOG_ERR("find_FRU_ID fail when read eeprom 0x%x ", offset);
		return false;
	}

	memcpy(&entry.config, &fru_config[fru_index], sizeof(fru_config[fru_index]));
	memset(entry.data, 0xFF, sizeof(entry.data));

	if (!eeprom_read(&entry)) {
		LOG_ERR("read eeprom 0x%x fail", offset);
		return false;
	}

	memcpy(data, entry.data, data_len);

	return true;
}

bool plat_get_cpld_fru_data(uint8_t *data)
{
	const uint32_t total_size = AEGIS_CPLD_FRU_SIZE; // 0x0400
	const uint32_t chunk_size = 0x80; // 0x80 bytes per read
	uint32_t offset = 0;

	while (offset < total_size) {
		if (!plat_cpld_fru_read(AEGIS_CPLD_FRU_START + offset, data + offset, chunk_size)) {
			LOG_ERR("Failed to read FRU chunk at offset 0x%x", offset);
			return false;
		}

		LOG_INF("fru_read 0x%x", AEGIS_CPLD_FRU_START + offset);
		offset += chunk_size;
	}

	return true;
}

// get fru info at init, and store all information in fru_info
/*expected fru data:
{
  "Chassis Info": {
    "Chassis Type": 23,
    "Chassis Part Number": "Chassis PN",
    "Chassis Serial Number": "Chassis SN",
    "Chassis Custom Data 1": "MAC ADDRESS 1",
    "Chassis Custom Data 2": "MAC ADDRESS 2",
    "Chassis Custom Data 3": "MAC ADDRESS 3",
    "Chassis Custom Data 4": "MAC ADDRESS 4",
    "Chassis Custom Data 5": "MAC ADDRESS 5",
    "Chassis Custom Data 6": "MAC ADDRESS 6",
    "Chassis Custom Data 7": "MAC ADDRESS 7",
    "Chassis Custom Data 8": "MAC ADDRESS 8",
    "Chassis Custom Data 9": "MAC ADDRESS 9",
    "Chassis Custom Data 10": "MAC ADDRESS 10",
    "Chassis Custom Data 11": "MAC ADDRESS 11",
    "Chassis Custom Data 12": "MAC ADDRESS 12",
    "Chassis Custom Data 13": "MAC ADDRESS 13",
    "Chassis Custom Data 14": "MAC ADDRESS 14",
    "Chassis Custom Data 15": "MAC ADDRESS 15",
    "Chassis Custom Data 16": "MAC ADDRESS 16",
    "Chassis Custom Data 17": "MAC ADDRESS 17",
    "Chassis Custom Data 18": "MAC ADDRESS 18",
    "Chassis Custom Data 19": "MAC ADDRESS 19",
    "Chassis Custom Data 20": "MAC ADDRESS 20",
    "Chassis Custom Data 21": "MAC ADDRESS 21",
    "Chassis Custom Data 22": "MAC ADDRESS 22",
    "Chassis Custom Data 23": "MAC ADDRESS 23",
    "Chassis Custom Data 24": "MAC ADDRESS 24"
  },
  "Board Info": {
    "Language": 25,
    "Board Mfg Date": "2025-02-13 14:35:00",
    "Board Mfg": "Quanta",
    "Board Product": "Minerva EVB DVT",
    "Board Serial": "ODM_DEFINE_SN",
    "Board Part Number": "3JF0MBB00E0",
    "Board FRU ID": "FRU Ver 0.06",
    "Board Custom Data 1": "19-100618",
    "Board Custom Data 2": "PCB vendor name",
    "Board Custom Data 3": "Batch ID",
    "Board Custom Data 4": "vr-mps-isl"
  },
  "Product Info": {
    "Language": 25,
    "Product Manufacturer": "Quanta",
    "Product Name": "T21 Minerva",
    "Product Part Number": "ODM L10 PN",
    "Product Version": "Product Ver",
    "Product Serial": "L10 SN",
    "Product Asset Tag": "L10 Asset Label",
    "Product FRU ID": "Compute Blade FBPN"
  }
}
*/

typedef struct {
	ChassisInfo chassis;
	BoardInfo board;
	ProductInfo product;
} FRU_INFO;

FRU_INFO *g_fru_info = NULL;

FRU_INFO *create_fru_info(void)
{
	FRU_INFO *info = malloc(sizeof(FRU_INFO));
	if (!info) {
		LOG_ERR("Failed to allocate memory for FRU_INFO");
		return NULL;
	}
	memset(info, 0, sizeof(FRU_INFO));
	return info;
}

static void decode_field(const uint8_t *src, int len, char *dest, int dest_size)
{
	int copy_len = (len < dest_size - 1) ? len : dest_size - 1;
	memcpy(dest, src, copy_len);
	dest[copy_len] = '\0';

	//print result
	LOG_HEXDUMP_INF(src, len, "decode_field result:");
}

bool init_fru_info(void)
{
	g_fru_info = create_fru_info();
	if (!g_fru_info) {
		return false;
	}

	uint8_t fru_data[AEGIS_CPLD_FRU_SIZE];

	/* Read FRU data from CPLD EEPROM */
	if (!plat_get_cpld_fru_data(fru_data)) {
		LOG_ERR("Failed to read CPLD FRU data");
		return false;
	}

	/* Ensure the binary data is large enough for the common header */
	if (AEGIS_CPLD_FRU_SIZE < 8) {
		LOG_ERR("FRU data size too small");
		return false;
	}

	/* Parse common header (first 8 bytes) */
	uint8_t common_header[8];
	memcpy(common_header, fru_data, 8);
	uint8_t chassis_offset = common_header[2] * 8;
	uint8_t board_offset = common_header[3] * 8;
	uint8_t product_offset = common_header[4] * 8;

	/* --------------------- Parse Chassis Area --------------------- */
	if (chassis_offset + 3 < AEGIS_CPLD_FRU_SIZE) {
		uint8_t area_len = fru_data[chassis_offset + 1] * 8;
		g_fru_info->chassis.chassis_type = fru_data[chassis_offset + 2];

		int offset = chassis_offset + 3;
		/* Field 1: Chassis Part Number */
		if (offset < chassis_offset + area_len - 1) {
			uint8_t tl = fru_data[offset++];
			if (tl != 0xC1) {
				int len = tl & 0x3F;
				decode_field(&fru_data[offset], len,
					     g_fru_info->chassis.chassis_part_number,
					     sizeof(g_fru_info->chassis.chassis_part_number));
				offset += len;
			}
		}
		/* Field 2: Chassis Serial Number */
		if (offset < chassis_offset + area_len - 1) {
			uint8_t tl = fru_data[offset++];
			if (tl != 0xC1) {
				int len = tl & 0x3F;
				decode_field(&fru_data[offset], len,
					     g_fru_info->chassis.chassis_serial_number,
					     sizeof(g_fru_info->chassis.chassis_serial_number));
				offset += len;
			}
		}
		/* Fields 3 ~ 26: Chassis Custom Data 1 ~ 24 */
		// for (int i = 0; i < CHASSIS_CUSTOM_DATA_MAX && offset < chassis_offset + area_len;
		//      i++) {
		for (int i = 0; i < 24; i++) {
			uint8_t tl = fru_data[offset++];

			LOG_INF("Chassis Custom Data field %d TL = 0x%02x", i + 1, tl);
			if (tl == 0xC1) {
				LOG_INF("Chassis Custom Data %d is empty", i + 1);
				break;
			}
			int len = tl & 0x3F;
			decode_field(&fru_data[offset], len,
				     g_fru_info->chassis.chassis_custom_data[i],
				     sizeof(g_fru_info->chassis.chassis_custom_data[i]));
			offset += len;
			//print CHASSIS_CUSTOM_DATA_MAX && offset < chassis_offset + area_len - 1
			LOG_INF("  i <  xxx %d", (CHASSIS_CUSTOM_DATA_MAX &&
						  offset < chassis_offset + area_len - 1));
		}
	} else {
		LOG_ERR("Invalid chassis offset");
		return false;
	}

	/* --------------------- Parse Board Area --------------------- */
	if (board_offset + 6 < AEGIS_CPLD_FRU_SIZE) {
		uint8_t area_len = fru_data[board_offset + 1] * 8;
		g_fru_info->board.language = fru_data[board_offset + 2];

		g_fru_info->board.language = fru_data[board_offset + 2];
		strncpy(g_fru_info->board.board_mfg_date, "N/A",
			sizeof(g_fru_info->board.board_mfg_date));

		int offset = board_offset + 6;
		char buffer[32];
		/* Field 1: Board Mfg */
		if (offset < board_offset + area_len - 1) {
			uint8_t tl = fru_data[offset++];
			if (tl != 0xC1) {
				int len = tl & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(g_fru_info->board.board_mfg, buffer,
					sizeof(g_fru_info->board.board_mfg));
				offset += len;
			}
		}
		/* Field 2: Board Product */
		if (offset < board_offset + area_len - 1) {
			uint8_t tl = fru_data[offset++];
			if (tl != 0xC1) {
				int len = tl & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(g_fru_info->board.board_product, buffer,
					sizeof(g_fru_info->board.board_product));
				offset += len;
			}
		}
		/* Field 3: Board Serial */
		if (offset < board_offset + area_len - 1) {
			uint8_t tl = fru_data[offset++];
			if (tl != 0xC1) {
				int len = tl & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(g_fru_info->board.board_serial, buffer,
					sizeof(g_fru_info->board.board_serial));
				offset += len;
			}
		}
		/* Field 4: Board Part Number */
		if (offset < board_offset + area_len - 1) {
			uint8_t tl = fru_data[offset++];
			if (tl != 0xC1) {
				int len = tl & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(g_fru_info->board.board_part_number, buffer,
					sizeof(g_fru_info->board.board_part_number));
				offset += len;
			}
		}
		/* Field 5: Board FRU ID */
		if (offset < board_offset + area_len - 1) {
			uint8_t tl = fru_data[offset++];
			if (tl != 0xC1) {
				int len = tl & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(g_fru_info->board.board_fru_id, buffer,
					sizeof(g_fru_info->board.board_fru_id));
				offset += len;
			}
		}
		/* Fields 6 ~ 9: Board Custom Data 1 ~ 4 */
		for (int i = 0; i < BOARD_CUSTOM_DATA_MAX && offset < board_offset + area_len - 1;
		     i++) {
			uint8_t tl = fru_data[offset++];
			if (tl == 0xC1)
				break;
			int len = tl & 0x3F;
			decode_field(&fru_data[offset], len, g_fru_info->board.board_custom_data[i],
				     sizeof(g_fru_info->board.board_custom_data[i]));
			offset += len;
		}
	} else {
		LOG_ERR("Invalid board offset");
		return false;
	}

	/* --------------------- Parse Product Area --------------------- */
	if (product_offset + 3 < AEGIS_CPLD_FRU_SIZE) {
		uint8_t area_len = fru_data[product_offset + 1] * 8;
		g_fru_info->product.language = fru_data[product_offset + 2];
		int offset = product_offset + 3;
		char buffer[32];
		/* Field 1: Product Manufacturer */
		if (offset < product_offset + area_len - 1) {
			uint8_t tl = fru_data[offset++];
			if (tl != 0xC1) {
				int len = tl & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(g_fru_info->product.product_manufacturer, buffer,
					sizeof(g_fru_info->product.product_manufacturer));
				offset += len;
			}
		}
		/* Field 2: Product Name */
		if (offset < product_offset + area_len - 1) {
			uint8_t tl = fru_data[offset++];
			if (tl != 0xC1) {
				int len = tl & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(g_fru_info->product.product_name, buffer,
					sizeof(g_fru_info->product.product_name));
				offset += len;
			}
		}
		/* Field 3: Product Part Number */
		if (offset < product_offset + area_len - 1) {
			uint8_t tl = fru_data[offset++];
			if (tl != 0xC1) {
				int len = tl & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(g_fru_info->product.product_part_number, buffer,
					sizeof(g_fru_info->product.product_part_number));
				offset += len;
			}
		}
		/* Field 4: Product Version */
		if (offset < product_offset + area_len - 1) {
			uint8_t tl = fru_data[offset++];
			if (tl != 0xC1) {
				int len = tl & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(g_fru_info->product.product_version, buffer,
					sizeof(g_fru_info->product.product_version));
				offset += len;
			}
		}
		/* Field 5: Product Serial */
		if (offset < product_offset + area_len - 1) {
			uint8_t tl = fru_data[offset++];
			if (tl != 0xC1) {
				int len = tl & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(g_fru_info->product.product_serial, buffer,
					sizeof(g_fru_info->product.product_serial));
				offset += len;
			}
		}
		/* Field 6: Product Asset Tag */
		if (offset < product_offset + area_len - 1) {
			uint8_t tl = fru_data[offset++];
			if (tl != 0xC1) {
				int len = tl & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(g_fru_info->product.product_asset_tag, buffer,
					sizeof(g_fru_info->product.product_asset_tag));
				offset += len;
			}
		}
		/* Field 7: Product FRU ID */
		if (offset < product_offset + area_len - 1) {
			uint8_t tl = fru_data[offset++];
			if (tl != 0xC1) {
				int len = tl & 0x3F;
				decode_field(&fru_data[offset], len, buffer, sizeof(buffer));
				strncpy(g_fru_info->product.product_fru_id, buffer,
					sizeof(g_fru_info->product.product_fru_id));
				offset += len;
			}
		}
	} else {
		LOG_ERR("Invalid product offset");
		return false;
	}

	/* FRU information has been successfully parsed and stored in g_fru_info */
	return true;
}

void print_fru_info(void)
{
	if (!g_fru_info) {
		LOG_ERR("FRU info not initialized");
		return;
	}

	printf("{\n");

	/* Print Chassis Info */
	printf("  \"Chassis Info\": {\n");
	printf("    \"Chassis Type\": %d,\n", g_fru_info->chassis.chassis_type);
	printf("    \"Chassis Part Number\": \"%s\",\n", g_fru_info->chassis.chassis_part_number);
	printf("    \"Chassis Serial Number\": \"%s\"", g_fru_info->chassis.chassis_serial_number);
	for (int i = 0; i < CHASSIS_CUSTOM_DATA_MAX; i++) {
		if (strlen(g_fru_info->chassis.chassis_custom_data[i]) > 0) {
			printf(",\n    \"Chassis Custom Data %d\": \"%s\"", i + 1,
			       g_fru_info->chassis.chassis_custom_data[i]);
		}
	}
	printf("\n  },\n");

	/* Print Board Info */
	printf("  \"Board Info\": {\n");
	printf("    \"Language\": %d,\n", g_fru_info->board.language);
	printf("    \"Board Mfg Date\": \"%s\",\n", g_fru_info->board.board_mfg_date);
	printf("    \"Board Mfg\": \"%s\",\n", g_fru_info->board.board_mfg);
	printf("    \"Board Product\": \"%s\",\n", g_fru_info->board.board_product);
	printf("    \"Board Serial\": \"%s\",\n", g_fru_info->board.board_serial);
	printf("    \"Board Part Number\": \"%s\",\n", g_fru_info->board.board_part_number);
	printf("    \"Board FRU ID\": \"%s\"", g_fru_info->board.board_fru_id);
	for (int i = 0; i < BOARD_CUSTOM_DATA_MAX; i++) {
		if (strlen(g_fru_info->board.board_custom_data[i]) > 0) {
			printf(",\n    \"Board Custom Data %d\": \"%s\"", i + 1,
			       g_fru_info->board.board_custom_data[i]);
		}
	}
	printf("\n  },\n");

	/* Print Product Info */
	printf("  \"Product Info\": {\n");
	printf("    \"Language\": %d,\n", g_fru_info->product.language);
	printf("    \"Product Manufacturer\": \"%s\",\n", g_fru_info->product.product_manufacturer);
	printf("    \"Product Name\": \"%s\",\n", g_fru_info->product.product_name);
	printf("    \"Product Part Number\": \"%s\",\n", g_fru_info->product.product_part_number);
	printf("    \"Product Version\": \"%s\",\n", g_fru_info->product.product_version);
	printf("    \"Product Serial\": \"%s\",\n", g_fru_info->product.product_serial);
	printf("    \"Product Asset Tag\": \"%s\",\n", g_fru_info->product.product_asset_tag);
	printf("    \"Product FRU ID\": \"%s\"\n", g_fru_info->product.product_fru_id);
	printf("  }\n");

	printf("}\n");
}
