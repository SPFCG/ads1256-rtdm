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
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/types.h>
#include "ring_buffer.h"
#include <rtdm/rtdm.h>
#include <rtdm/driver.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spi.h>
#include <rtdm/uapi/spi.h>
#include <rtdm/fd.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/stat.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <asm/thread_info.h>
#include <cobalt/kernel/thread.h>
#include <cobalt/kernel/sched.h>
#include <linux/sched.h>
#include <linux/cpumask.h>
#include "spi-device.h"
#include "spi-master.h"

#ifdef __CDT_PARSER__


#define __init
#define __exit
#define MODULE_VERSION(x)
#define MODULE_DESCRIPTION(x)
#define MODULE_AUTHOR(x)
#define MODULE_LICENSE(x)
#define module_init(x)
#define module_exit(x)

#include <asm-generic/atomic.h>

#endif /* __CDT_PARSER__ */

#define ADS1256_ADC_DEVICES_NUMBER 8
#define ADS1256_DEVICE_ID 0x03
#define ADS1256_BUFFER_SIZE 2000000

#define  ADS1256_DEFAULT_DRDY_GPIO_PIN 17
#define  ADS1256_DEFAULT_RESET_GPIO_PIN 18

#define ADS1256_EMPTY_CHANNEL 0xFF

#define ADS1256_DEFAULT_SAMPLE_RATE ADS1256_DATA_RATE_30K_SPS
#define ADS1256_DEFAULT_PGA ADS1256_PGA1

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
    ADS1256_AIN0   = 0x00,
    ADS1256_AIN1   = 0x01,
    ADS1256_AIN2   = 0x02,
    ADS1256_AIN3   = 0x03,
    ADS1256_AIN4   = 0x04,
    ADS1256_AIN5   = 0x05,
    ADS1256_AIN6   = 0x06,
    ADS1256_AIN7   = 0x07,
    ADS1256_AINCOM = 0x08,

}ads1256_analog_inputs_e;

typedef enum ads1256_pga{
    ADS1256_PGA1   = 0x00,
    ADS1256_PGA2   = 0x01,
    ADS1256_PGA4   = 0x02,
    ADS1256_PGA8   = 0x03,
    ADS1256_PGA16   = 0x04,
    ADS1256_PGA32  = 0x05,
    ADS1256_PGA64   = 0x06,


}ads1256_pga_e;

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

typedef enum ads1256_ioctl {
    ADS1256_SET_SAMPLERATE,
    ADS1256_SET_PGA,
    ADS1256_SET_CHANNEL_CYCLING_SIZE,
    ADS1256_SET_BLOCKING_READ

}ads1256_ioctl_e;

struct ads1256_register_status_s{
    u8 order:1;
    u8 acal:1;
    u8 bufen:1;
    u8 drdy:1;
    u8 id:4;




};

struct ads1256_register_mux_s{
    u8 reserved0:1;
    u8 clk:2;
    u8 sdcs:2;
    u8 pga:3;
};

struct ads1256_register_adcon_s{
    u8 nsel:4;
    u8 psel:4;
};

struct ads1256_register_io_s{
    u8 dio:4;
    u8 dir3:1;
    u8 dir2:1;
    u8 dir1:1;
    u8 dir0:1;
};

typedef union ads1256_register_status_u{
    struct ads1256_register_status_s elements;
    u8 value;
} ads1256_register_status_t;

typedef union ads1256_register_mux_u{
    struct ads1256_register_mux_s elements;
    u8 value;
} ads1256_register_mux_t;

typedef union ads1256_register_adcon_u{
    struct ads1256_register_status_s elements;
    u8 value;
} ads1256_register_adcon_t;

typedef union ads1256_register_drate_u{
    ads1256_data_rate_e value_enum;
    u8 value;
} ads1256_register_drate_t;

typedef union ads1256_register_io_u{
    struct ads1256_register_io_s elements;
    u8 value;
} ads1256_register_io_t;


static struct ads1256_dev_context_s {
    ads1256_analog_inputs_e analog_input;
    struct ring_buffer buffer;
    rtdm_lock_t lock;
    int device_used;
};

static struct buffer_s {
    u8 data[100];
    ssize_t size;
};


typedef struct ads1256_dev_context_s ads1256_dev_context_t;
typedef struct buffer_s buffer_t;


#endif //ADS1256_RTDM_ADS1256_RTDM_H
