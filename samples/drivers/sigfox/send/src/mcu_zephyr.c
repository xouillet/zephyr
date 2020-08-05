/*
 * Copyright (c) 2020 Gerson Fernando Budke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/sigfox.h>

#include <sigfox_types.h>
#include <sigfox_api.h>
#include <mcu_api.h>
#include <retriever_api.h>
#include <string.h>
#include <stdlib.h>

#include <tinycrypt/cbc_mode.h>
#include <tinycrypt/constants.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(sigfox_mcu, CONFIG_SIGFOX_SEND_LOG_LEVEL);

static uint8_t sfx_int_data[SFX_NVMEM_BLOCK_SIZE] = { 0 };

const uint8_t private_key[16] = {
	0x00, 0x11, 0x22, 0x33,  0x44, 0x55, 0x66, 0x77,
	0x88, 0x99, 0xaa, 0xbb,  0xcc, 0xdd, 0xee, 0xff,
};

#if defined(CONFIG_SIGFOX_ST_RETRIEVER)
static uint32_t _id;
static uint8_t  _pac[8];
static uint8_t  _rcz;
#endif

/**
 * SIGFOX MCU ZEPHYR API
 */
sfx_u8 MCU_API_malloc(sfx_u16 size, sfx_u8 **returned_pointer)
{
	/* Must be 4 bytes align */
	uint32_t *ptr = malloc(size / sizeof(uint32_t));

	LOG_DBG("Size: %d, Address: %08X", size, (uint32_t) ptr);

	if (ptr) {
		(*returned_pointer) = (sfx_u8 *) ptr;

		return SIGFOX_ERR_NONE;
	}

	return MCU_ERR_API_MALLOC;
}

sfx_u8 MCU_API_free(sfx_u8 *ptr)
{
	LOG_DBG("Address: %08X", (uint32_t) ptr);

	free((uint32_t *) ptr);

	return SIGFOX_ERR_NONE;
}

sfx_u8 MCU_API_get_voltage_temperature(sfx_u16 *voltage_idle,
				       sfx_u16 *voltage_tx,
				       sfx_s16 *temperature)
{
	(*voltage_idle) = 0;
	(*voltage_tx)   = 0;
	(*temperature)  = 0;

	LOG_DBG("MCU_API_get_voltage_temperature");

	return SIGFOX_ERR_NONE;
}

sfx_u8 MCU_API_delay(sfx_delay_t delay_type)
{
	LOG_DBG("Type: %d", delay_type);

	switch (delay_type) {
	case SFX_DLY_INTER_FRAME_TRX:
	case SFX_DLY_INTER_FRAME_TX:
	case SFX_DLY_CS_SLEEP:
		/**
		 * Since ramp consists of 72 samples (18ms for each ramp, it
		 * needs to be compensate 36 ms). Moreover, 6ms of silence
		 * (2 before and 4 after packet).
		 */
		/*(500 - 2 * ST_RF_API_get_ramp_duration());*/
		k_busy_wait(500000);
		break;
	case SFX_DLY_OOB_ACK:
		/*(2000-2*ST_RF_API_get_ramp_duration());*/
		k_busy_wait(2000000);
		break;
	}

	return SIGFOX_ERR_NONE;
}

sfx_u8 MCU_API_timer_start_carrier_sense(sfx_u16 time_duration_in_ms)
{
	LOG_DBG("Duration: %dms", time_duration_in_ms);

	return SIGFOX_ERR_NONE;
}

sfx_u8 MCU_API_timer_stop_carrier_sense(void)
{
	LOG_DBG("");

	return SIGFOX_ERR_NONE;
}

sfx_u8 MCU_API_timer_start(sfx_u32 time_duration_in_s)
{
	LOG_DBG("Duration: %lus", time_duration_in_s);

	return SIGFOX_ERR_NONE;
}

sfx_u8 MCU_API_timer_stop(void)
{
	LOG_DBG("");

	return SIGFOX_ERR_NONE;
}

sfx_u8 MCU_API_timer_wait_for_end(void)
{
	LOG_DBG("");

	return SIGFOX_ERR_NONE;
}

sfx_u8 MCU_API_report_test_result(sfx_bool status, sfx_s16 rssi)
{
	LOG_DBG("Status: %d, RSSI: %d", status, rssi);

	return SIGFOX_ERR_NONE;
}

sfx_u8 MCU_API_get_version(sfx_u8 **version, sfx_u8 *size)
{
	(*version) = (sfx_u8 *) CONFIG_SIGFOX_MCU_API_VERSION;
	(*size) = sizeof(CONFIG_SIGFOX_MCU_API_VERSION);

	LOG_DBG("Version: %s, Size: %d", *version, *size);

	return SIGFOX_ERR_NONE;
}

sfx_u8 MCU_API_get_nv_mem(sfx_u8 read_data[SFX_NVMEM_BLOCK_SIZE])
{
	memcpy((uint8_t *) read_data,
	       (uint8_t *) sfx_int_data,
	       SFX_NVMEM_BLOCK_SIZE);

	LOG_HEXDUMP_DBG(read_data, SFX_NVMEM_BLOCK_SIZE, "get NV mem");

	return SIGFOX_ERR_NONE;
}

sfx_u8 MCU_API_set_nv_mem(sfx_u8 data_to_write[SFX_NVMEM_BLOCK_SIZE])
{
	memcpy((uint8_t *) sfx_int_data,
	       (uint8_t *) data_to_write,
	       SFX_NVMEM_BLOCK_SIZE);

	LOG_HEXDUMP_DBG(data_to_write, SFX_NVMEM_BLOCK_SIZE, "set NV mem");

	return SIGFOX_ERR_NONE;
}

sfx_u8 MCU_API_aes_128_cbc_encrypt(sfx_u8 *encrypted_data,
				   sfx_u8 *data_to_encrypt,
				   sfx_u8 aes_block_len,
				   sfx_u8 key[16],
				   sfx_credentials_use_key_t use_key)
{
#if !defined(CONFIG_SIGFOX_ST_RETRIEVER)
	struct tc_aes_key_sched_struct aes;
	uint8_t iv[TC_AES_BLOCK_SIZE];
	uint8_t enc[TC_AES_BLOCK_SIZE + aes_block_len];
	const uint8_t *key_ptr;
	sfx_s8 ret;

	key_ptr = (use_key == CREDENTIALS_PRIVATE_KEY) ? private_key : key;

	LOG_HEXDUMP_DBG(data_to_encrypt, aes_block_len, "aes in");
	LOG_HEXDUMP_DBG(key_ptr, TC_AES_BLOCK_SIZE, "PK");
	LOG_HEXDUMP_DBG(key, TC_AES_BLOCK_SIZE, "Public Key");

	(void)tc_aes128_set_encrypt_key(&aes, key_ptr);
	(void)memset(iv, 0, TC_AES_BLOCK_SIZE);

	if (tc_cbc_mode_encrypt(enc,
				aes_block_len + TC_AES_BLOCK_SIZE,
				data_to_encrypt, aes_block_len,
				iv, &aes) == 0) {
		LOG_ERR("encrypting");

		ret = SIGFOX_ERR_MCU_AES;
	} else {
		memcpy(encrypted_data, enc + TC_AES_BLOCK_SIZE, aes_block_len);
		LOG_HEXDUMP_DBG(enc + TC_AES_BLOCK_SIZE,
				aes_block_len, "aes out");

		ret = SIGFOX_ERR_NONE;
	}

	(void)memset(iv, 0, TC_AES_BLOCK_SIZE);
	(void)memset(&aes, 0, sizeof(struct tc_aes_key_sched_struct));

	return ret;
#else
	LOG_HEXDUMP_DBG(data_to_encrypt, aes_block_len, "aes in");

	enc_utils_encrypt(encrypted_data, data_to_encrypt,
			  aes_block_len, key, use_key);

	LOG_HEXDUMP_DBG(encrypted_data, aes_block_len, "aes out");

	return SIGFOX_ERR_NONE;
#endif
}

sfx_u8 MCU_API_get_device_id_and_payload_encryption_flag(
	sfx_u8 dev_id[ID_LENGTH],
	sfx_bool *payload_encryption_enabled)
{
#if defined(CONFIG_SIGFOX_ST_RETRIEVER)
	enc_utils_retrieve_data(&_id, _pac, &_rcz);
	LOG_DBG("ID: %08x", _id);
	memcpy(dev_id, &_id, ID_LENGTH);
#else
	dev_id[0] = 0xFE;
	dev_id[1] = 0xDC;
	dev_id[2] = 0xBA;
	dev_id[3] = 0x98;
#endif
	(*payload_encryption_enabled) = 0;

	LOG_DBG("ID: %02x-%02x-%02x-%02x, Encrypt: %s",
		dev_id[0], dev_id[1], dev_id[2], dev_id[3],
		*payload_encryption_enabled ? "True" : "False");

	return SIGFOX_ERR_NONE;
}

sfx_u8 MCU_API_get_initial_pac(sfx_u8 initial_pac[PAC_LENGTH])
{
#if defined(CONFIG_SIGFOX_ST_RETRIEVER)
	memcpy(initial_pac, _pac, PAC_LENGTH);
#else
	initial_pac[0] = 0x11;
	initial_pac[1] = 0x22;
	initial_pac[2] = 0x33;
	initial_pac[3] = 0x44;

	initial_pac[4] = 0x55;
	initial_pac[5] = 0x66;
	initial_pac[6] = 0x77;
	initial_pac[7] = 0x88;
#endif
	LOG_DBG("ID: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x",
		initial_pac[3], initial_pac[2],
		initial_pac[1], initial_pac[0],
		initial_pac[7], initial_pac[6],
		initial_pac[5], initial_pac[4]);

	return SIGFOX_ERR_NONE;
}

/**
 * Sigfox dependency
 */
void __aeabi_memcpy(void *dest, const void *src, size_t n)
{
	memcpy(dest, src, n);
}

void __aeabi_memcpy4(void *dest, const void *src, size_t n)
{
	memcpy(dest, src, n);
}

void __aeabi_memclr(void *dest, size_t n)
{
	memset(dest, 0, n);
}

void __aeabi_memclr4(void *dest, size_t n)
{
	memset(dest, 0, n);
}

void __aeabi_memset(void *dest, char c, size_t n)
{
	memset(dest, c, n);
}

/**
 * ST, deve ser removido
 */

#if defined(CONFIG_SIGFOX_ST_RETRIEVER)
uint32_t GetNVMBoardDataAddress(void)
{
	uint32_t address = 0x600;

	LOG_DBG("Address: 0x%08x", address);

	return address;
}

NVM_RW_RESULTS NVM_Read(uint32_t nAddress, uint8_t cNbBytes, uint8_t *pcBuffer)
{
	LOG_DBG("Address: 0x%08x, len: %d", nAddress, cNbBytes);

	return 0;
}

uint8_t EepromRead(uint16_t nAddress, uint8_t cNbBytes, uint8_t *pcBuffer)
{
	LOG_DBG("Address: 0x%08x, len: %d", nAddress, cNbBytes);

	switch (nAddress) {
	case 0x1f0:
		memcpy(pcBuffer, sfx_data_vector, cNbBytes);
		break;
	case 0x200:
		memcpy(pcBuffer, sfx_data_vector + 16, cNbBytes);
		break;
	case 0x6:
		memcpy(pcBuffer, sfx_data_vector + 6, cNbBytes);
		break;
	}

	LOG_HEXDUMP_DBG(pcBuffer, cNbBytes, "dump");

	return 0;
}
#endif
