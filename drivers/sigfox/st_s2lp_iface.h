/*
 * Copyright (c) 2020 Gerson Fernando Budke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SIGFOX_ST_S2LP_IFACE_H_
#define ZEPHYR_DRIVERS_SIGFOX_ST_S2LP_IFACE_H_

#define S2LP_IFACE_WRITE                        (0 << 0)
#define S2LP_IFACE_READ                         (1 << 0)
#define S2LP_IFACE_CMD                          (1 << 7)

/**
 * @brief Wake-Up from shutdown mode
 *
 * @param[in] dev   Transceiver device instance
 *
 * @return =0  on Success
 *         <0  otherwise
 */
int s2lp_iface_wakeup(const struct device *dev);

/**
 * @brief Put transceiver in shutdown mode
 *
 * @param[in] dev   Transceiver device instance
 */
void s2lp_iface_shutdown(const struct device *dev);

/**
 * @brief Send a command to transceiver
 *
 * @param[in] dev   Transceiver device instance
 * @param[in] cmd   A s2-lp valid command code
 */
void s2lp_iface_cmd(const struct device *dev,
		    enum s2lp_trx_cmd_t cmd);

/**
 * @brief Reads current value from a transceiver register
 *
 * This function reads the current value from a transceiver register.
 *
 * @param[in] dev   Transceiver device instance
 * @param[in] addr  Specifies the address of the trx register
 * from which the data shall be read
 *
 * @return value of the register read
 */
uint8_t s2lp_iface_reg_read(const struct device *dev,
			    uint8_t addr);

/**
 * @brief Writes data into a transceiver register
 *
 * This function writes a value into transceiver register.
 *
 * @param[in] dev   Transceiver device instance
 * @param[in] addr  Address of the trx register
 * @param[in] data  Data to be written to trx register
 *
 */
void s2lp_iface_reg_write(const struct device *dev,
			  uint8_t addr,
			  uint8_t data);

/**
 * @brief Subregister read
 *
 * @param[in] dev   Transceiver device instance
 * @param[in] addr  offset of the register
 * @param[in] mask  bit mask of the subregister
 * @param[in] pos   bit position of the subregister
 *
 * @return  value of the read bit(s)
 */
uint8_t s2lp_iface_bit_read(const struct device *dev,
			    uint8_t addr,
			    uint8_t mask,
			    uint8_t pos);

/**
 * @brief Subregister write
 *
 * @param[in]   dev       Transceiver device instance
 * @param[in]   reg_addr  Offset of the register
 * @param[in]   mask      Bit mask of the subregister
 * @param[in]   pos       Bit position of the subregister
 * @param[out]  new_value Data, which is muxed into the register
 */
void s2lp_iface_bit_write(const struct device *dev,
			  uint8_t reg_addr,
			  uint8_t mask,
			  uint8_t pos,
			  uint8_t new_value);

/**
 * @brief Reads data from the transceiver
 *
 * This function reads data from the transceiver sram
 *
 * @param[in]   dev     Transceiver device instance
 * @param[in]   addr    Offset of the register
 * @param[out]  data    Pointer to the location to store read data
 * @param[in]   length  Number of bytes to be read from the address
 */
void s2lp_iface_burst_read(const struct device *dev,
			   uint8_t addr,
			   uint8_t *data,
			   uint8_t length);

/**
 * @brief Writes data into the transceiver sram
 *
 * This function writes data into the transceiver sram
 *
 * @param[in] dev    Transceiver device instance
 * @param[in] addr   Offset of the register
 * @param[in] data   Pointer to data to be written into sram
 * @param[in] length Number of bytes to be written into sram
 */
void s2lp_iface_burst_write(const struct device *dev,
			    uint8_t addr,
			    uint8_t *data,
			    uint8_t length);

#endif /* ZEPHYR_DRIVERS_SIGFOX_ST_S2LP_IFACE_H_ */
