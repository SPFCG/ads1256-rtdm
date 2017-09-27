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



static ads1256_dev_context_t *context;



static struct rtdm_fd * file_desctiptor_device;
static int device_handle;
static rtdm_task_t  ads1256_task_agent;
static struct rtdm_device ads1256_adc_devices[ADS1256_ADC_DEVICES_NUMBER];

static int ads1256_read_data(buffer_t * data){
    bcm283x_spi_rtdm_read_rt_using_context(context, data, BCM283X_SPI_BUFFER_SIZE_MAX );
    return 0;
}

static uint8_t ads1256_read_byte(void){
   return bcm283x_spi_rtdm_read_byte_rt(context);
}

static ssize_t ads1256_write_byte(uint8_t byte){
    return bcm283x_spi_rtdm_write_byte_rt(context, byte);
}

static int ads1256_rtdm_send_cmd(ads1256_commands_e cmd, ads1256_registers_e register_address, uint8_t value){
    ssize_t write_size;
    buffer_t buffer;
    int i;
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
            buffer.data[0] = cmd;
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
                    buffer.data[0] = ((uint8_t) cmd + (uint8_t) register_address);
                    buffer.data[1] = 0x00;
                    buffer.size = 2;
                    break;
                default:
                    printk(KERN_ERR "Invalid register_address used in ads1256_rtdm_send_cmd  while using ADS1256_COMMAND_RREG or ADS1256_COMMAND_WREG command.\r\n");
                    return EINVAL;
            }
            break;
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
                    buffer.data[0] = (uint8_t) cmd + (uint8_t) register_address;
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

    //write_size = bcm283x_spi_rtdm_write_rt_using_context(context, &buffer,(size_t) buffer.size);
    for(i = 0; i < buffer.size; i++)
        ads1256_write_byte(buffer.data[i]);

//    if(write_size > 0)
 //       return 0;
  //  else {
  //  else {
   //     printk(KERN_ERR "Problem communication with ads1256 using SPI.write_size: %u\r\n", write_size);
    //    return EIO;
    //}
    return 0;
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
                *return_value = ads1256_read_byte();
                return 0;


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
        *value = buffer.data[0];
        *value << 16;
        *value += buffer.data[1];
        *value << 8;
        *value += buffer.data[2];
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

static void ads1256_rtdm_print_register_status(void){
    ads1256_register_status_t register_address;
    ads1256_rtdm_read_register_status(&register_address) ;

    printk(KERN_INFO "[ads1256-rtdm] ADS1256 device status:\n");
    printk(KERN_INFO "[ads1256-rtdm] ID: %x", register_address.elements.id);
    printk(KERN_INFO "[ads1256-rtdm] ACAL: %x", register_address.elements.acal);
    printk(KERN_INFO "[ads1256-rtdm] BUFEN: %x", register_address.elements.bufen);
    printk(KERN_INFO "[ads1256-rtdm] ORDER: %x", register_address.elements.order);
    printk(KERN_INFO "[ads1256-rtdm] DRDY: %x", register_address.elements.drdy);
}

static void ads1256_rtdm_print_register_data_rate(void){
    ads1256_register_drate_t register_address;
    ads1256_rtdm_read_register_drate(&register_address) ;

    printk(KERN_INFO "[ads1256-rtdm] ADS1256 device status:\n");
    printk(KERN_INFO "[ads1256-rtdm] DATA RATE: %x\n", register_address.value);

}

static int ads1256_rtdm_open(struct rtdm_fd *fd, int oflags) {
    ads1256_dev_context_t * context = (ads1256_dev_context_t *) rtdm_fd_to_private(fd);
    context->analog_input = fd->minor;
 return 0;
}

static void ads1256_rtdm_close(struct rtdm_fd *fd) {
    return;

}

static ssize_t ads1256_rtdm_read_rt(struct rtdm_fd *fd, void __user *buf, size_t size) {

    ads1256_dev_context_t * context = (ads1256_dev_context_t *) rtdm_fd_to_private(fd);

    ads1256_rtdm_send_cmd(ADS1256_COMMAND_WREG, ADS1256_REGISTER_MUX, (context->analog_input << 4) + AINCOM);
    ads1256_rtdm_send_cmd(ADS1256_COMMAND_SYNC, ADS1256_REGISTER_NONE, 0);
    ads1256_rtdm_send_cmd(ADS1256_COMMAND_WAKEUP, ADS1256_REGISTER_NONE, 0);
    ads1256_rtdm_send_cmd(ADS1256_COMMAND_RDATA, ADS1256_REGISTER_NONE, 0);
    buffer_t data;

    data.data[0] = ads1256_read_byte();
    data.data[1] = ads1256_read_byte();
    data.data[2] = ads1256_read_byte();
    rtdm_safe_copy_to_user(fd, buf, (const void *) data.data, 3);
    return 3;

}

static ssize_t ads1256_rtdm_write_rt(struct rtdm_fd *fd, const void __user *buf, size_t size) {
    return 0;
}

static int ads1256_rtdm_ioctl_rt(struct rtdm_fd *fd, unsigned int request, void __user *arg) {
    return 0;
}

/**
 * This structure describes the RTDM driver.
 */
static struct rtdm_driver ads1256_driver = {
        .profile_info = RTDM_PROFILE_INFO(foo, RTDM_CLASS_EXPERIMENTAL, RTDM_SUBCLASS_GENERIC, 42),
        .device_flags = RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE | RTDM_FIXED_MINOR,
        .device_count = ADS1256_ADC_DEVICES_NUMBER,
        .context_size = sizeof(struct ads1256_dev_context_s),

        .ops = {
                .open = ads1256_rtdm_open,
                .read_rt = ads1256_rtdm_read_rt,
                .write_rt = ads1256_rtdm_write_rt,
                .ioctl_rt = ads1256_rtdm_ioctl_rt,
                .close = ads1256_rtdm_close
        }
};

int ads1256_rtdm_init_ac_devices(void){

    int device_id, res;
    for(device_id = 0; device_id < ADS1256_ADC_DEVICES_NUMBER; device_id++){

        /* Set device parameters */
        ads1256_adc_devices[device_id].driver = &ads1256_driver;
        ads1256_adc_devices[device_id].label = "acd0.%d";
        ads1256_adc_devices[device_id].minor = device_id;

        /* Try to register the device */
        res = rtdm_dev_register(&ads1256_adc_devices[device_id]);
        if (res == 0) {
            printk(KERN_INFO "%s: Device acd0.%d registered without errors.\r\n", __FUNCTION__, device_id);
        } else {
            printk(KERN_ERR "%s: Device acd0.%d registration failed : ", __FUNCTION__, device_id);
            switch (res) {
                case -EINVAL:
                    printk(KERN_ERR "The descriptor contains invalid entries.\r\n");
                    break;

                case -EEXIST:
                    printk(KERN_ERR "The specified device name of protocol ID is already in use.\r\n");
                    break;

                case -ENOMEM:
                    printk(KERN_ERR "A memory allocation failed in the process of registering the device.\r\n");
                    break;

                default:
                    printk(KERN_ERR "Unknown error code returned.\r\n");
                    break;
            }
            return res;
        }
    }
    return 0;
}

static int ads1256_rtdm_search_device_on_spi(void) {
    int device_id, res ;

    for(device_id = 0 ; device_id < SPI_BCM283X_RTDM_DEVICES_NUMBER; device_id++){

        res = rtdm_open(spi_rtdm_device[device_id], O_RDWR);

        if(res > 0){

            printk(KERN_INFO "Search device on %s", spi_rtdm_device[device_id]);

            struct rtdm_fd * fd = rtdm_fd_get(res, RTDM_FD_MAGIC);
            context = (ads1256_dev_context_t *)rtdm_fd_to_private(fd);

            ads1256_register_status_t register_address;
            ads1256_rtdm_read_register_status(&register_address);

            if(register_address.elements.id == ADS126_DEVICE_ID){

                printk(KERN_INFO "Found ADS1256 device on %s", spi_rtdm_device[device_id]);
                device_handle = res;
                rtdm_fd_put(fd);
                return 0;
            }else{
                rtdm_fd_put(fd);
                context = NULL;
                rtdm_close(res);
                continue;
            }
        }else{
            context = NULL;
            continue;
        }
    }
    printk(KERN_ERR "Device ADS1256 not connected.\r\n");
    return -ENOENT;
}
/*
static int ads1256_rtdm_task_init(rtdm_task_t *task, const char *name,
                   rtdm_task_proc_t task_proc, void *arg,
                   int priority, nanosecs_rel_t period)
{
    union xnsched_policy_param param;
    struct xnthread_start_attr sattr;
    struct xnthread_init_attr iattr;
    int err;

    iattr.name = name;
    iattr.flags = 0;
    iattr.personality = &xenomai_personality;
    iattr.affinity = CPU_MASK_CPU0;
    param.rt.prio = priority;

    err = xnthread_init(task, &iattr, &xnsched_class_rt, &param);
    if (err)
        return err;

    /* We need an anonymous registry entry to obtain a handle for fast
       mutex locking. */
    err = xnthread_register(task, "");
    if (err)
        goto cleanup_out;

    if (period > 0) {
        err = xnthread_set_periodic(task, XN_INFINITE,
                                    XN_RELATIVE, period);
        if (err)
            goto cleanup_out;
    }

    sattr.mode = 0;
    sattr.entry = task_proc;
    sattr.cookie = arg;
    err = xnthread_start(task, &sattr);
    if (err)
        goto cleanup_out;

    return 0;

    cleanup_out:
    xnthread_cancel(task);
    return err;
}

*/


static int __init ads1256_rtdm_init(void)
{
    int res;
    printk(KERN_INFO "Real-Time A/C driver for the ADS1256 init!\n");
    ads1256_rtdm_init_ac_devices();

    res = ads1256_rtdm_search_device_on_spi();
    if(res == 0) {

        ads1256_rtdm_print_register_status();
        ads1256_rtdm_print_register_data_rate();




        return 0;
    }else{
        return -ENOENT;
    }


}

static void __exit  ads1256_rtdm_exit(void)
{
    int device_id,res ;
    if(context != NULL) {
        res = rtdm_close(device_handle);


    }

    //if(ads1256_task_agent != NULL)
        //rtdm_task_destroy(&ads1256_task_agent);

    for (device_id = 0; device_id < ADS1256_ADC_DEVICES_NUMBER; device_id++) {
        //if(bcm283x_spi_rtdm_open_devices[device_id] != NULL){
        //    bcm283x_spi_rtdm_open_devices[device_id]->device_used = 0;
        //}
        printk(KERN_INFO "%s: Unregistering device acd0.%d  ...\r\n", __FUNCTION__, device_id);
        rtdm_dev_unregister(&ads1256_adc_devices[device_id]);
        printk(KERN_INFO "%s: Device acd0.%d unregistered  ...\r\n", __FUNCTION__, device_id);
    }

    //vfree(context);
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
