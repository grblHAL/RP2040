/*
  i2c.c - I2C support for EEPROM, keypad and Trinamic plugins

  Part of grblHAL driver for RP2040

  Copyright (c) 2021-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include "i2c.h"
#include "grbl/hal.h"

#define MAX_PAGE_SIZE 64
#define I2C_MAX_TSIZE 256

#define I2CN_PORT(port) I2Cn(port)
#define I2Cn(port) i2c##port
#define I2CN_IRQ(port) I2Cirq(port)
#define I2Cirq(port) I2C##port##_IRQ

#if I2C_ENABLE

#define QI2C_PORT I2CN_PORT(I2C_PORT)
#define QI2C_IRQ I2CN_IRQ(I2C_PORT)

typedef union
{
    uint16_t value;
    struct
    {
        uint8_t data;
        uint8_t cmd;
    };
} i2c_data_t;

typedef struct
{
    size_t len;
    int channel;
    bool busy;
    i2c_hw_t *port;
    i2c_data_t payload[I2C_MAX_TSIZE];
} i2c_trans_t;

static i2c_trans_t tx = {0};
static uint8_t keycode = 0;
static keycode_callback_ptr keypad_callback = NULL;

void i2c_irq_handler(void)
{
    const uint32_t event = tx.port->intr_stat;

    if(event & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        tx.port->clr_tx_abrt;

        if(tx.busy) {
            dma_channel_abort(tx.channel);
            tx.busy = false;
        }
    }

    if(event & I2C_IC_INTR_STAT_R_STOP_DET_BITS) {
        tx.port->clr_stop_det;
        tx.busy = false;
    }

    if(!tx.busy)
        irq_set_enabled(QI2C_IRQ, false);
}

static void i2c_cfg_pin (uint8_t gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_I2C);
    gpio_set_pulls(gpio, true, false);
    gpio_set_input_hysteresis_enabled(gpio, true);
    gpio_set_slew_rate(gpio, GPIO_SLEW_RATE_SLOW);
}

i2c_cap_t i2c_start (void)
{
    static i2c_cap_t cap = {};

    if(cap.started)
        return cap;

    i2c_cfg_pin(I2C_SDA);
    i2c_cfg_pin(I2C_SCL);
    i2c_init(QI2C_PORT, 100000UL);

    static const periph_pin_t scl = {
        .function = Output_SCK,
        .group = PinGroup_I2C,
        .pin = I2C_SCL,
        .mode = { .mask = PINMODE_OD }
    };

    static const periph_pin_t sda = {
        .function = Bidirectional_SDA,
        .group = PinGroup_I2C,
        .pin = I2C_SDA,
        .mode = { .mask = PINMODE_OD }
    };

    hal.periph_port.register_pin(&scl);
    hal.periph_port.register_pin(&sda);

    tx.port = i2c_get_hw(QI2C_PORT);   
    tx.port->intr_mask = I2C_IC_INTR_MASK_M_STOP_DET_BITS|I2C_IC_INTR_MASK_M_TX_ABRT_BITS;
    tx.channel = dma_claim_unused_channel(false);

    irq_set_exclusive_handler(QI2C_IRQ, i2c_irq_handler);

    cap.started = On;
    cap.tx_non_blocking = cap.tx_dma = tx.channel != -1;

    return cap;
}

bool i2c_probe (uint_fast16_t i2cAddr)
{
    char buf = '\0';

    return i2c_read_blocking(QI2C_PORT, i2cAddr, &buf, 1, false) != PICO_ERROR_GENERIC;
}

// TODO: add timeout handling
bool i2c_receive (uint_fast16_t i2cAddr, uint8_t *buf, size_t size, bool block)
{
    return i2c_read_blocking(QI2C_PORT, i2cAddr, buf, size, false) != PICO_ERROR_GENERIC;
}

// TODO: add timeout handling
bool i2c_send (uint_fast16_t i2cAddr, uint8_t *buf, size_t bytes, bool block)
{
    bool ok;

    if((ok = block || tx.channel == -1)) {
 
        while(tx.busy) {
            if(!hal.stream_blocking_callback())
                return false;
        }

        ok = i2c_write_blocking(QI2C_PORT, i2cAddr, buf, bytes, false) == bytes;

    } else {

        if(tx.busy)
            return false;

        uint16_t *data = &tx.payload[0].value;

        tx.len = bytes;

        while(bytes--)
            *data++ = *buf++;

        tx.payload[0].value |= I2C_IC_DATA_CMD_RESTART_BITS;
        tx.payload[tx.len - 1].value |= I2C_IC_DATA_CMD_STOP_BITS;

        tx.port->enable = 0;
        tx.port->tar = i2cAddr;
        tx.port->enable = 1;

        dma_channel_config cfg = dma_channel_get_default_config(tx.channel);
        channel_config_set_read_increment(&cfg, true);
        channel_config_set_write_increment(&cfg, false);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
        channel_config_set_dreq(&cfg, i2c_get_dreq(QI2C_PORT, true));

        irq_set_enabled(QI2C_IRQ, true);
        dma_channel_configure(tx.channel, &cfg, &tx.port->data_cmd, tx.payload, tx.len, true);

        ok = true;
    }

    return ok;
}

bool i2c_get_keycode (uint_fast16_t i2cAddr, keycode_callback_ptr callback)
{
    uint8_t c;
    keycode = 0;
    keypad_callback = callback;

    dma_channel_wait_for_finish_blocking(tx.channel);

    if(i2c_read_blocking(QI2C_PORT, i2cAddr, &c, 1, false) == 1)
        keypad_callback(c);

    return true;
}

bool i2c_transfer (i2c_transfer_t *i2c, bool read)
{
    static uint8_t txbuf[NVS_SIZE + 2];

    bool ok;

    if(i2c->word_addr_bytes == 2) {
        txbuf[0] = i2c->word_addr >> 8;
        txbuf[1] = i2c->word_addr & 0xFF;
    } else
        txbuf[0] = i2c->word_addr;

    if(read) {
        i2c_write_blocking(QI2C_PORT, i2c->address, txbuf, i2c->word_addr_bytes, true);
        ok = i2c_read_blocking(QI2C_PORT, i2c->address, i2c->data, i2c->count, false) != PICO_ERROR_GENERIC;
    } else if((ok = i2c->count + i2c->word_addr_bytes) <= sizeof(txbuf)) {
        memcpy(&txbuf[i2c->word_addr_bytes], i2c->data, i2c->count);
        if(i2c->no_block)
            ok = i2c_send(i2c->address, txbuf, i2c->count + i2c->word_addr_bytes, false) != PICO_ERROR_GENERIC;
        else
            ok = i2c_write_blocking(QI2C_PORT, i2c->address, txbuf, i2c->count + i2c->word_addr_bytes, false) != PICO_ERROR_GENERIC;
    }

    return ok;
}

/*
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(keypad_callback && keycode != 0) {
        keypad_callback(keycode);
        keypad_callback = NULL;
    }
}
*/

#if TRINAMIC_ENABLE && TRINAMIC_I2C

static const uint8_t tmc_addr = I2C_ADR_I2CBRIDGE << 1;

uint8_t *I2C_ReadRegister (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes, bool block)
{
    i2c_write_blocking(QI2C_PORT, i2cAddr, buf, 1, true);
    i2c_read_blocking(QI2C_PORT, i2cAddr, buf, bytes, false);

    return buf;
}

static TMC2130_status_t TMC_I2C_ReadRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    uint8_t tmc_reg, buffer[5] = {0};
    TMC2130_status_t status = {0};

    if((tmc_reg = TMCI2C_GetMapAddress((uint8_t)(driver ? (uint32_t)driver->cs_pin : 0), reg->addr).value) == 0xFF)
        return status; // unsupported register


    HAL_I2C_Mem_Read(&i2c_port, tmc_addr, tmc_reg, I2C_MEMADD_SIZE_8BIT, buffer, 5, 100);

    status.value = buffer[0];
    reg->payload.value = buffer[4];
    reg->payload.value |= buffer[3] << 8;
    reg->payload.value |= buffer[2] << 16;
    reg->payload.value |= buffer[1] << 24;

    return status;
}

static TMC2130_status_t TMC_I2C_WriteRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    uint8_t tmc_reg, buffer[4];
    TMC2130_status_t status = {0};

    reg->addr.write = 1;
    tmc_reg = TMCI2C_GetMapAddress((uint8_t)(driver ? (uint32_t)driver->cs_pin : 0), reg->addr).value;
    reg->addr.write = 0;

    if(tmc_reg != 0xFF) {

        buffer[0] = (reg->payload.value >> 24) & 0xFF;
        buffer[1] = (reg->payload.value >> 16) & 0xFF;
        buffer[2] = (reg->payload.value >> 8) & 0xFF;
        buffer[3] = reg->payload.value & 0xFF;

        HAL_I2C_Mem_Write(&i2c_port, tmc_addr, tmc_reg, I2C_MEMADD_SIZE_8BIT, buffer, 4, 100);
    }

    return status;
}

#endif
#endif // I2C_ENABLE
