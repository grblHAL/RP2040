/*
  spi.c - SPI support for SD card, Trinamic & networking (WizNet) plugins

  Part of grblHAL driver for RP2040

  Copyright (c) 2020-2026 Terje Io

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

#include "driver.h"

#if SPI_ENABLE

#include "hardware/dma.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#define SPIport(p) SPIportI(p)
#define SPIportI(p) spi ## p
#define SPIdreqRX(p) SPIdreqrxI(p)
#define SPIdreqrxI(p) DREQ_SPI ## p ## _RX
#define SPIdreqTX(p) SPIdreqtxI(p)
#define SPIdreqtxI(p) DREQ_SPI ## p ## _TX

#define SPIPORT SPIport(SPI_PORT)
#define SPIDMARX SPIdreqRX(SPI_PORT)
#define SPIDMATX SPIdreqTX(SPI_PORT)

#define SPI_CLK 8000000

typedef struct
{
    int channel;
    dma_channel_config config;
    io_rw_32 *port;
    uint8_t dummy;
} spi_dma_t;

static spi_dma_t dma_tx;
static spi_dma_t dma_rx;

spi_cap_t spi_start (spi_slave_t *device)
{
    static const periph_pin_t sck = {
        .function = Output_SPICLK,
        .group = PinGroup_SPI,
        .pin = SPI_SCK_PIN,
        .mode = { .mask = PINMODE_OUTPUT }
    };

    static const periph_pin_t sdo = {
        .function = Input_MISO,
        .group = PinGroup_SPI,
        .pin = SPI_MISO_PIN,
        .mode = { .mask = PINMODE_NONE }
    };

    static const periph_pin_t sdi = {
        .function = Output_MOSI,
        .group = PinGroup_SPI,
        .pin = SPI_MOSI_PIN,
        .mode = { .mask = PINMODE_NONE }
    };
    
    static bool init = false;

    if(!init) {

        spi_init(SPIPORT, SPI_CLK);
        gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
        gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
        gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);

        hal.periph_port.register_pin(&sck);
        hal.periph_port.register_pin(&sdi);
        hal.periph_port.register_pin(&sdo);

        dma_rx.port = &spi_get_hw(SPIPORT)->dr;
        dma_rx.channel = dma_claim_unused_channel(true);
        dma_rx.config = dma_channel_get_default_config(dma_rx.channel);
        channel_config_set_transfer_data_size(&dma_rx.config, DMA_SIZE_8);
        channel_config_set_dreq(&dma_rx.config, SPIDMARX);
        channel_config_set_read_increment(&dma_rx.config, false);

        dma_tx.dummy = 0xFF;
        dma_tx.port = &spi_get_hw(SPIPORT)->dr;
        dma_tx.channel = dma_claim_unused_channel(true);
        dma_tx.config = dma_channel_get_default_config(dma_tx.channel);
        channel_config_set_transfer_data_size(&dma_tx.config, DMA_SIZE_8);
        channel_config_set_dreq(&dma_tx.config, SPIDMATX);
        channel_config_set_write_increment(&dma_tx.config, false);

        init = true;
    }

    return (spi_cap_t){ .started = On };
}

uint8_t spi_get_byte (void)
{
	uint8_t byte;

	spi_read_blocking(SPIPORT, 0xFF, &byte, 1);

    return byte;
}

uint8_t spi_put_byte (uint8_t byte)
{
	spi_write_blocking(SPIPORT, &byte, 1);

    return 0;
}

bool spi_write (uint8_t *data, uint16_t len)
{
    if(len <= 2)
    	spi_write_blocking(SPIPORT, data, len);
    else {
        channel_config_set_read_increment(&dma_tx.config, true);
        dma_channel_configure(dma_tx.channel, &dma_tx.config, dma_tx.port, data, len, false);

        channel_config_set_write_increment(&dma_rx.config, false);
        dma_channel_configure(dma_rx.channel, &dma_rx.config, &dma_rx.dummy, dma_rx.port, len, false);

        dma_start_channel_mask((1 << dma_tx.channel) | (1 << dma_rx.channel));
        dma_channel_wait_for_finish_blocking(dma_rx.channel);
    }

    return true;
}

bool spi_read (uint8_t *data, uint16_t len)
{
    if(len <= 2) {
    	spi_read_blocking(SPIPORT, 0xFF, data, len);
    } else {
        channel_config_set_read_increment(&dma_tx.config, false);
        dma_channel_configure(dma_tx.channel, &dma_tx.config, dma_tx.port, &dma_tx.dummy, len, false);

        channel_config_set_write_increment(&dma_rx.config, true);
        dma_channel_configure(dma_rx.channel, &dma_rx.config, data, dma_rx.port, len, false);

        dma_start_channel_mask((1 << dma_tx.channel) | (1 << dma_rx.channel));
        dma_channel_wait_for_finish_blocking(dma_rx.channel);
    }

    return true;
}

bool spi_select (spi_slave_t *device)
{
    static uint32_t spi_freq = SPI_CLK;

    if(device->f_clock && device->f_clock != spi_freq)
  	    spi_set_baudrate(SPIPORT, (spi_freq = device->f_clock));

    DIGITAL_OUT(device->cs_pin, 0);

    return true;
}

bool spi_deselect (spi_slave_t *device)
{
    DIGITAL_OUT(device->cs_pin, 1);

    return true;
}

#endif // SPI_ENABLE
