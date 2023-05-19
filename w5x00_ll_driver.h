/**
 * Copyright (c) 2022 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _W5X00_LL_DRIVER_H_
#define _W5X00_LL_DRIVER_H_

#include "driver.h"

#if ETHERNET_ENABLE && (_WIZCHIP_ == 5105 || _WIZCHIP_ == 5500)

#include "wizchip_conf.h"

typedef enum {
    WizChipInit_OK = 0,
    WizChipInit_MemErr = -1,
    WizChipInit_WrongChip = -2,
    WizChipInit_UnknownLinkStatus = -3
} wizchip_init_err_t;

/* GPIO */
/*! \brief Initialize w5x00 gpio interrupt callback function
 *  \ingroup w5x00_gpio_irq
 *
 *  Add a w5x00 interrupt callback.
 *
 *  \param socket socket number
 *  \param callback the gpio interrupt callback function
 */
bool wizchip_gpio_interrupt_initialize(uint8_t socket, void (*callback)(void));

/*! \brief W5x00 chip reset
 *  \ingroup w5x00_spi
 *
 *  Set a reset pin and reset.
 *
 *  \param none
 */
void wizchip_reset(void);

/*! \brief Initialize WIZchip
 *  \ingroup w5x00_spi
 *
 *  Set callback function to read/write byte using SPI.
 *  Set callback function for WIZchip select/deselect.
 *  Set memory size of W5x00 chip and monitor PHY link status.
 *
 *  \param none
 */
wizchip_init_err_t wizchip_initialize (void);

/* Network */
/*! \brief Initialize network
 *  \ingroup w5x00_spi
 *
 *  Set network information.
 *
 *  \param net_info network information.
 */
void network_initialize(wiz_NetInfo net_info);

#endif /* _W5X00_LL_DRIVER_H_s */
#endif
