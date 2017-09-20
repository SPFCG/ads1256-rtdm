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
#include "../include/ads1256-rtdm.h"


/* Bypass CLion syntax checker */
#ifdef __CDT_PARSER__
#define __init
#define __exit
#define MODULE_VERSION(x)
#define MODULE_DESCRIPTION(x)
#define MODULE_AUTHOR(x)
#define MODULE_LICENSE(x)
#define module_init(x)
#define module_exit(x)
#endif /* __CDT_PARSER__ */

spi_bcm283x_context_t *context;

static int ads1256_read_data(buffer_t * data){
    bcm283x_spi_rtdm_read_rt_using_context(context, data, BCM283X_SPI_BUFFER_SIZE_MAX );
    return 0;
}

static int ads1256_rtdm_send_cmd(ads1256_commands_e cmd, ads1256_registers_e register_address, uint8_t value){
    ssize_t write_size;
    buffer_t buffer;
    switch(cmd) {
        case ADS1256_COMMAND_RDATA:
        case ADS1256_COMMAND_RDATAC:
        case ADS1256_COMMAND_RESET:
        case ADS1256_COMMAND_SDATAC:
        case ADS1256_COMMAND_SELFCAL:
        case ADS1256_COMMAND_SELFGCAL:
        case ADS1256_COMMAND_SELFOCAL:
        case ADS1256_COMMAND_STNDBY:
        case ADS1256_COMMAND_SYNC:
        case ADS1256_COMMAND_SYSGCAL:
        case ADS1256_COMMAND_SYSOCAL:
        case ADS1256_COMMAND_WAKEUP:
            buffer.data[0] = (char)cmd;
            buffer.size = 1;
            break;

        case ADS1256_COMMAND_RREG:
            switch(register_address){
                case ADS1256_REGISTER_STATUS:
                case ADS1256_REGISTER_MUX:
                case ADS1256_REGISTER_ADCON:
                case ADS1256_REGISTER_DRATE:
                case ADS1256_REGISTER_IO:
                case ADS1256_REGISTER_OFC0:
                case ADS1256_REGISTER_OFC1:
                case ADS1256_REGISTER_OFC2:
                case ADS1256_REGISTER_FSC0:
                case ADS1256_REGISTER_FSC1:
                case ADS1256_REGISTER_FSC2:
                    buffer.data[0] = (char)((uint8_t) cmd + (uint8_t) register_address);
                    buffer.data[1] = 0;
                    buffer.size = 2;
                    break;
                default:
                    printk(KERN_ERR "Invalid register_address used in ads1256_rtdm_send_cmd  while using ADS1256_COMMAND_RREG or ADS1256_COMMAND_WREG command.\r\n");
                    return EINVAL;
            }
        case ADS1256_COMMAND_WREG:
            switch(register_address){
                case ADS1256_REGISTER_STATUS:
                case ADS1256_REGISTER_MUX:
                case ADS1256_REGISTER_ADCON:
                case ADS1256_REGISTER_DRATE:
                case ADS1256_REGISTER_IO:
                case ADS1256_REGISTER_OFC0:
                case ADS1256_REGISTER_OFC1:
                case ADS1256_REGISTER_OFC2:
                case ADS1256_REGISTER_FSC0:
                case ADS1256_REGISTER_FSC1:
                case ADS1256_REGISTER_FSC2:
                    buffer.data[0] = (char)((uint8_t) cmd + (uint8_t) register_address);
                    buffer.data[1] = 0;
                    buffer.data[2] = value;
                    buffer.size = 3;
                    break;
                default:
                    printk(KERN_ERR "Invalid register_address used in ads1256_rtdm_send_cmd  while using ADS1256_COMMAND_RREG or ADS1256_COMMAND_WREG command.\r\n");
                    return EINVAL;
            }

            break;
        default:
            printk(KERN_ERR "Invalid command in function ads1256_rtdm_send_cmd.\r\n");
            return EINVAL;
    }

    write_size = bcm283x_spi_rtdm_write_rt_using_context(context, &buffer,(size_t) buffer.size);
    if(write_size > 0)
        return 0;
    else {
        printk(KERN_ERR "Problem communication with ads1256 using SPI.\r\n");
        return EIO;
    }
}

static int ads1256_rtdm_read_register(ads1256_registers_e register_address, uint8_t *return_value) {
    switch (register_address) {
        case ADS1256_REGISTER_STATUS:
        case ADS1256_REGISTER_MUX:
        case ADS1256_REGISTER_ADCON:
        case ADS1256_REGISTER_DRATE:
        case ADS1256_REGISTER_IO:
        case ADS1256_REGISTER_OFC0:
        case ADS1256_REGISTER_OFC1:
        case ADS1256_REGISTER_OFC2:
        case ADS1256_REGISTER_FSC0:
        case ADS1256_REGISTER_FSC1:
        case ADS1256_REGISTER_FSC2:
            ads1256_rtdm_send_cmd(ADS1256_COMMAND_RREG, register_address, 0);
            buffer_t data;
            ads1256_read_data(&data);
            if (data.size > 0) {
                *return_value = data.data[0];
                return 0;
            } else {
                printk(KERN_ERR "Problem communication with ads1256 using SPI.\r\n");
                return EIO;
            }

            break;
        default:
            printk(KERN_ERR "Invalid register address used in ads1256_rtdm_read_register while using.\r\n");
            return EINVAL;

    }
}

static int ads1256_rtdm_read_register_status(ads1256_register_status_t *value){

    return  ads1256_rtdm_read_register(ADS1256_REGISTER_STATUS, &(value->value));
}

static int ads1256_rtdm_read_register_mux(ads1256_register_mux_t *value){

    return  ads1256_rtdm_read_register(ADS1256_REGISTER_MUX, &(value->value));
}

static int ads1256_rtdm_read_register_adcon(ads1256_register_adcon_t *value){

    return  ads1256_rtdm_read_register(ADS1256_REGISTER_ADCON, &(value->value));
}

static int ads1256_rtdm_read_register_io(ads1256_register_io_t *value){

    return  ads1256_rtdm_read_register(ADS1256_REGISTER_IO, &(value->value));
}

static int ads1256_rtdm_read_register_drate(ads1256_register_drate_t *value){

   return  ads1256_rtdm_read_register(ADS1256_REGISTER_DRATE, &(value->value));
}

static int ads1256_rtdm_read_registers_ofc(uint32_t  * value){
    buffer_t buffer;
    ssize_t  write_size;
    value = 0x0;
    buffer.data[0] = (uint8_t) ADS1256_COMMAND_RREG + (uint8_t) ADS1256_REGISTER_OFC0;
    buffer.data[1] = 0x2;
    buffer.size = 2;

    write_size = bcm283x_spi_rtdm_write_rt_using_context(context, &buffer, buffer.size);
    if(write_size <= 0)
    {
        printk(KERN_ERR "Problem communication with ads1256 using SPI.\r\n");
        return EIO;
    }
    if(ads1256_read_data(&buffer) == 0){
        *value = buffer.data[2];
        *value << 16;
        *value += buffer.data[1];
        *value << 8;
        *value += buffer.data[0];
        return 0;
    }else{
        printk(KERN_ERR "Problem communication with ads1256 using SPI.\r\n");
        return EIO;
    }
}

static int ads1256_rtdm_read_registers_fsc(uint32_t  * value){
    buffer_t buffer;
    ssize_t  write_size;
    value = 0x0;
    buffer.data[0] = (uint8_t) ADS1256_COMMAND_RREG + (uint8_t) ADS1256_REGISTER_FSC0;
    buffer.data[1] = 0x2;
    buffer.size = 2;

    write_size = bcm283x_spi_rtdm_write_rt_using_context(context, &buffer, buffer.size);
    if(write_size <= 0)
    {
        printk(KERN_ERR "Problem communication with ads1256 using SPI.\r\n");
        return EIO;
    }
    if(ads1256_read_data(&buffer) == 0){
        *value = buffer.data[2];
        *value << 16;
        *value += buffer.data[1];
        *value << 8;
        *value += buffer.data[0];
        return 0;
    }else{
        printk(KERN_ERR "Problem communication with ads1256 using SPI.\r\n");
        return EIO;
    }
}

static int __init ads1256_rtdm_init(void)
{
    printk(KERN_INFO "Real-Time A/C driver for the ADS1256 init!\n");
    context = (struct spi_bcm283x_context_s *)vmalloc(sizeof(struct spi_bcm283x_context_s));
    bcm283x_spi_rtdm_set_default_config(context, 0);
    return 0;
}

static void __exit  ads1256_rtdm_exit(void)
{
    vfree(context);
    printk(KERN_INFO "Cleaning up module.\n");
}

module_init(ads1256_rtdm_init);
module_exit(ads1256_rtdm_exit);


/*
 * Register module values
 */
#ifndef GIT_VERSION
#define GIT_VERSION "0.1-untracked";
#endif /* ! GIT_VERSION */
MODULE_VERSION(GIT_VERSION);
MODULE_DESCRIPTION("Real-Time A/C driver for the ads1256 using the RTDM API");
MODULE_AUTHOR("Piotr Piórkowski <qba100@gmail.com>");
MODULE_LICENSE("GPL v2");
