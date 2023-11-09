//
// Created by Max Larsson on 11/6/23.
//

/*
This file should be tailored to match the hardware design.

See
https://github.com/carlk3/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/tree/main#customizing-for-the-hardware-configuration
*/

#include "hw_config.h"

/* Configuration of RP2040 hardware SPI object */
static spi_t spi = {
        .hw_inst = spi1,  // RP2040 SPI component
        .sck_gpio = 10,    // GPIO number (not Pico pin number)
        .mosi_gpio = 11,
        .miso_gpio = 12,
        .baud_rate = 12 * 1000 * 1000   // Actual frequency: 10416666.
};

/* SPI Interface */
static sd_spi_if_t spi_if = {
        .spi = &spi,  // Pointer to the SPI driving this card
        .ss_gpio = 13      // The SPI slave select GPIO for this SD card
};

/* Configuration of the SD Card socket object */
static sd_card_t sd_card = {
        /* "pcName" is the FatFs "logical drive" identifier.
        (See http://elm-chan.org/fsw/ff/doc/filename.html#vol) */
        .pcName = "0:",
        .type = SD_IF_SPI,
        .spi_if_p = &spi_if  // Pointer to the SPI interface driving this card
};

/* ********************************************************************** */

size_t sd_get_num() { return 1; }

sd_card_t *sd_get_by_num(size_t num) {
    if (0 == num) {
        return &sd_card;
    } else {
        return NULL;
    }
}

/* [] END OF FILE */