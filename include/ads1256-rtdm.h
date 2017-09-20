/**
 * Copyright (C) 2017 Piotr Piórkowski <qba100@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef ADS1256_RTDM_ADS1256_RTDM_H
#define ADS1256_RTDM_ADS1256_RTDM_H

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include "spi-bcm283x-rtdm.h"

typedef enum ads1256_commands {
    ADS1256_COMMAND_WAKEUP  = 0x00, // Completes SYNC and Exits Standby Mode
    ADS1256_COMMAND_RDATA   = 0x01, // Read Data
    ADS1256_COMMAND_RDATAC  = 0x03, // Read Data Continuously
    ADS1256_COMMAND_SDATAC  = 0x0F, // Stop Read Data Continuously
    ADS1256_COMMAND_RREG    = 0x10, // Read from REG rrrr 0001 rrrr 0000 nnnn (r - register adress, n numers register)
    ADS1256_COMMAND_WREG    = 0x50, // Write to REG rrrr
    ADS1256_COMMAND_SELFCAL = 0xF0, // Offset and Gain Self-Calibration
    ADS1256_COMMAND_SELFOCAL= 0xF1, // Offset Self-Calibration
    ADS1256_COMMAND_SELFGCAL= 0xF2, // Gain Self-Calibration
    ADS1256_COMMAND_SYSOCAL = 0xF3, // System Offset Calibration
    ADS1256_COMMAND_SYSGCAL = 0xF4, // System Gain Calibration
    ADS1256_COMMAND_SYNC    = 0xFC, // Synchronize the A/D Conversion
    ADS1256_COMMAND_STNDBY  = 0xFD, // Begin Standby Mode
    ADS1256_COMMAND_RESET   = 0xFE, // Reset to Power-Up Values
} ads1256_commands_e;

typedef enum ads1256_registers {
    ADS1256_REGISTER_STATUS = 0x00,
    ADS1256_REGISTER_MUX    = 0x01,
    ADS1256_REGISTER_ADCON  = 0x02,
    ADS1256_REGISTER_DRATE  = 0x03,
    ADS1256_REGISTER_IO     = 0x04,
    ADS1256_REGISTER_OFC0   = 0x05,
    ADS1256_REGISTER_OFC1   = 0x06,
    ADS1256_REGISTER_OFC2   = 0x07,
    ADS1256_REGISTER_FSC0   = 0x08,
    ADS1256_REGISTER_FSC1   = 0x09,
    ADS1256_REGISTER_FSC2   = 0x0A,
    ADS1256_REGISTER_NONE   = 0xFF,
} ads1256_registers_e;

typedef enum ads1256_analog_inputs{
    AIN0   = 0x00,
    AIN1   = 0x01,
    AIN2   = 0x02,
    AIN3   = 0x03,
    AIN4   = 0x04,
    AIN5   = 0x05,
    AIN6   = 0x06,
    AIN7   = 0x07,
    AINCOM = 0x08,

}ads1256_analog_inputs_e;

typedef enum ads1256_data_rate{
    ADS1256_DATA_RATE_30K_SPS     = 0xF0,
    ADS1256_DATA_RATE_15K_SPS     = 0xE0,
    ADS1256_DATA_RATE_7K5_SPS     = 0xD0,
    ADS1256_DATA_RATE_3K75_SPS    = 0xC0,
    ADS1256_DATA_RATE_2K_SPS      = 0xB0,
    ADS1256_DATA_RATE_1K_SPS      = 0xA1,
    ADS1256_DATA_RATE_0K5_SPS     = 0x92,
    ADS1256_DATA_RATE_0K1_SPS     = 0x82,
    ADS1256_DATA_RATE_0K06_SPS    = 0x72,
    ADS1256_DATA_RATE_0K05_SPS    = 0x63,
    ADS1256_DATA_RATE_0K03_SPS    = 0x53,
    ADS1256_DATA_RATE_0K025_SPS   = 0x43,
    ADS1256_DATA_RATE_0K015_SPS   = 0x33,
    ADS1256_DATA_RATE_0K01_SPS    = 0x23,
    ADS1256_DATA_RATE_0K005_SPS   = 0x13,
    ADS1256_DATA_RATE_0K002_5_SPS = 0x03,

}ads1256_data_rate_e;

struct ads1256_register_status_s{
    uint8_t id:4;
    uint8_t order:1;
    uint8_t acal:1;
    uint8_t bufen:1;
    uint8_t drdy:1;
};

struct ads1256_register_mux_s{
    uint8_t reserved0:1;
    uint8_t clk:2;
    uint8_t sdcs:2;
    uint8_t pga:3;
};

struct ads1256_register_adcon_s{
    uint8_t psel:4;
    uint8_t nsel:4;
};

struct ads1256_register_io_s{
    uint8_t dir3:1;
    uint8_t dir2:1;
    uint8_t dir1:1;
    uint8_t dir0:1;
    uint8_t dio:4;
};

typedef union ads1256_register_status_u{
    struct ads1256_register_status_s elements;
    uint8_t value;
} ads1256_register_status_t;

typedef union ads1256_register_mux_u{
    struct ads1256_register_mux_s elements;
    uint8_t value;
} ads1256_register_mux_t;

typedef union ads1256_register_adcon_u{
    struct ads1256_register_status_s elements;
    uint8_t value;
} ads1256_register_adcon_t;

typedef union ads1256_register_drate_u{
    ads1256_data_rate_e value_enum;
    uint8_t value;
} ads1256_register_drate_t;

typedef union ads1256_register_io_u{
    struct ads1256_register_io_s elements;
    uint8_t value;
} ads1256_register_io_t;




extern int bcm283x_spi_rtdm_set_default_config(spi_bcm283x_context_t *context, int chip_select);
extern ssize_t bcm283x_spi_rtdm_read_rt_using_context(spi_bcm283x_context_t *context, buffer_t *buf, size_t size);
extern ssize_t bcm283x_spi_rtdm_write_rt_using_context(spi_bcm283x_context_t *context, buffer_t *buf, size_t size);
extern int bcm283x_spi_change_bit_order(spi_bcm283x_context_t *context, const int value);
extern int bcm283x_spi_change_data_mode(spi_bcm283x_context_t *context, const int value);
extern int bcm283x_spi_change_clock_divider(spi_bcm283x_context_t *context, const int value);
extern int bcm283x_spi_change_cs_polarity(spi_bcm283x_context_t *context, const int value);

#endif //ADS1256_RTDM_ADS1256_RTDM_H
