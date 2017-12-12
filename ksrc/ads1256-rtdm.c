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


#ifdef __CDT_PARSER__
#define __init
#define __exit
#define MODULE_VERSION(x)
#define MODULE_DESCRIPTION(x)
#define MODULE_AUTHOR(x)
#define MODULE_LICENSE(x)
#define module_init(x)
#define module_exit(x)
#define module_param(x)
#define MODULE_PARM_DESC(x)



#endif

bool high_edge = true;
bool device_init = false;

static short drdy_gpio_pin= ADS1256_DEFAULT_DRDY_GPIO_PIN;
static short reset_gpio_pin = ADS1256_DEFAULT_RESET_GPIO_PIN;

static rtdm_task_t  ads1256_task_agent;
static rtdm_task_t  ads1256_search_ac_task;
static struct rtdm_device ads1256_adc_devices[ADS1256_ADC_DEVICES_NUMBER];
static ads1256_dev_context_t * ads1256_ads_open_devices[ADS1256_ADC_DEVICES_NUMBER];
static rtdm_lock_t global_lock;

static struct rtdm_spi_remote_slave * active_cs;
static struct rtdm_spi_master *rtdm_master;
rtdm_irq_t irqh;
static u8 active_channel = ADS1256_EMPTY_CHANNEL;
static struct ring_buffer global_buffer;

static buffer_t buffer_cmd;
static u8 last_byte_no = 0;
static atomic_t drdy_is_low_status = ATOMIC_INIT(0);


module_param(drdy_gpio_pin, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(drdy_gpio_pin, "DRDY GPIO pin");
module_param(reset_gpio_pin, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(reset_gpio_pin, "RESET GPIO pin");


static void ads1256_wait_drdy(void)
{
    u32 i;

    for (i = 0; i < 100; i++)
    {
        if (gpio_get_value(drdy_gpio_pin) == 0 && high_edge == true)
        {
            high_edge = false;
            break;
        }else{
            high_edge = true;
        }
        udelay(3);
    }
    if (i >= 100)
    {
        printk(KERN_WARNING"ADS1256_WaitDRDY() Time Out ...");
    }
}

static void ads1256_spi_do_chip_select(void)
{
    gpio_set_value(active_cs->cs_gpio, 0);
}

static void ads1256_spi_do_chip_deselect(void)
{
    gpio_set_value(active_cs->cs_gpio, 1);
}


static int ads1256_read_n_bytes(u8 *bytes, u32 n){
    int ret;
    //rtdm_mutex_lock(&rtdm_master->bus_lock);
   // ads1256_spi_do_chip_select();
    ret = rtdm_master->ops->read(active_cs, bytes, n);
   // ads1256_spi_do_chip_deselect();

   // rtdm_mutex_unlock(&rtdm_master->bus_lock);
    return ret;
}

static uint8_t ads1256_read_byte(void){
    u8 byte;
    ssize_t size;
    size = ads1256_read_n_bytes(&byte, 1);
   // printk(KERN_ERR"Read %u bytes %u", size, byte);

    return byte;
}

static ssize_t ads1256_write_n_bytes(u8 *bytes, u32 n){
    int ret;
    ssize_t  size;
   // rtdm_mutex_lock(&rtdm_master->bus_lock);

    size = rtdm_master->ops->write(active_cs, bytes, n);
    //ads1256_spi_do_chip_deselect();

   // rtdm_mutex_unlock(&rtdm_master->bus_lock);
    return size;
}

static ssize_t ads1256_write_byte(uint8_t byte){
    return ads1256_write_n_bytes(&byte, 1);
}





static int ads1256_rtdm_send_cmd(void){
    ssize_t write_size;


    write_size = ads1256_write_n_bytes(&(buffer_cmd.data), buffer_cmd.size);
    buffer_cmd.size = 0;
    last_byte_no = 0;
    if(write_size > 0)
       return 0;
    else {

        printk(KERN_ERR "Problem communication with ads1256 using SPI.write_size: %u\r\n", write_size);
        return EIO;
    }

}


static int ads1256_rtdm_prepare_cmd(ads1256_commands_e cmd, ads1256_registers_e register_address, uint8_t value){
;
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
            buffer_cmd.data[last_byte_no++] = cmd;
            buffer_cmd.size++;
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
                    buffer_cmd.data[last_byte_no++] = ((uint8_t) cmd + (uint8_t) register_address);
                    buffer_cmd.data[last_byte_no++] = 0x00;
                    buffer_cmd.size += 2;
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
                    buffer_cmd.data[last_byte_no++] = (uint8_t) cmd + (uint8_t) register_address;
                    buffer_cmd.data[last_byte_no++] = 0;
                    buffer_cmd.data[last_byte_no++] = value;
                    buffer_cmd.size += 3;
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
            ads1256_spi_do_chip_select();
            //ads1256_wait_drdy();
            ads1256_rtdm_prepare_cmd(ADS1256_COMMAND_RREG, register_address, 0);
            ads1256_rtdm_send_cmd();
            udelay(10);
            *return_value = ads1256_read_byte();
            ads1256_spi_do_chip_deselect();
            return 0;
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
/*
static int ads1256_rtdm_read_registers_ofc(uint32_t  * value){
    buffer_t buffer;
    ssize_t  write_size;
    value = 0x0;
    buffer.data[0] = (uint8_t) ADS1256_COMMAND_RREG + (uint8_t) ADS1256_REGISTER_OFC0;
    buffer.data[1] = 0x2;
    buffer.size = 2;

    write_size = bcm283x_spi_rtdm_write_rt_using_context(spi_context, &buffer, buffer.size);
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
}*/

/*static int ads1256_rtdm_read_registers_fsc(uint32_t  * value){
    buffer_t buffer;
    ssize_t  write_size;
    value = 0x0;
    buffer.data[0] = (uint8_t) ADS1256_COMMAND_RREG + (uint8_t) ADS1256_REGISTER_FSC0;
    buffer.data[1] = 0x2;
    buffer.size = 2;

    write_size = bcm283x_spi_rtdm_write_rt_using_context(spi_context, &buffer, buffer.size);
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
}*/

static void ads1256_rtdm_print_register_status(void){
    ads1256_register_status_t register_address;
    register_address.value = 0;
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
    if(active_channel == ADS1256_EMPTY_CHANNEL) {
        ads1256_dev_context_t *context = (ads1256_dev_context_t *) rtdm_fd_to_private(fd);
        int device_id = fd->minor;
        rtdm_lock_get(&global_lock);
        context->analog_input = device_id;
        context->device_used = 1;
        active_channel = device_id;
        ads1256_ads_open_devices[device_id] = context;
        ring_buffer_init(&(context->buffer), ADS1256_BUFFER_SIZE);
        rtdm_lock_put(&global_lock);
        return 0;
    }else{
        return -EBUSY;
    }
}

static void ads1256_rtdm_close(struct rtdm_fd *fd) {
    rtdm_lock_get(&global_lock);
    ads1256_dev_context_t * context = (ads1256_dev_context_t *) rtdm_fd_to_private(fd);
    context->analog_input = fd->minor;
    context->device_used = 0;
    ads1256_ads_open_devices[fd->minor] = NULL;
    active_channel = ADS1256_EMPTY_CHANNEL;
    rtdm_lock_put(&global_lock);

    return;

}

static ssize_t ads1256_rtdm_read_rt(struct rtdm_fd *fd, void __user *buf, size_t size) {

    /* TO DO uwzglednic size */
    ads1256_dev_context_t * context = (ads1256_dev_context_t *) rtdm_fd_to_private(fd);

    u8 *buffer;
    int curr_size;
    rtdm_lock_get(&(context->lock));
    curr_size = context->buffer.cur_size;
    buffer = rtdm_malloc(curr_size);
    ring_buffer_get_n(&(context->buffer), buffer, curr_size );

    rtdm_lock_put(&(context->lock));
    rtdm_safe_copy_to_user(fd, buf, (const void *) buffer, curr_size);
    rtdm_free(buffer);
    return curr_size;

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
        ads1256_ads_open_devices[device_id] = NULL;

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

static int ads1256_drdy_is_low(rtdm_irq_t *irqh)
{

    atomic_set(&drdy_is_low_status, 1);
    //rtdm_task_unblock(&ads1256_task_agent);
    //xnthread_resume(&ads1256_task_agent, 0);
    return RTDM_IRQ_HANDLED;
}

static void ads1256_dma_ready(unsigned long data){
    printk(KERN_ERR "To chyba działa");
}



static void ads1256_dma_exampel(void){

    struct rtdm_spi_iobufs iobufs;
    int ret;
    static unsigned char *i_area, *o_area;
#define TRANSFER_SIZE 30000 * 3  * 1;

    struct rtdm_spi_config *config = &active_cs->config;
    config->bits_per_word = 8;
    config->mode = SPI_MODE_1;
    config->speed_hz = 3920000;
    rtdm_master->ops->configure(active_cs);


    iobufs.io_len = TRANSFER_SIZE;
    ret = rtdm_master->ops->set_iobufs(active_cs, &iobufs);
    rtdm_master->ops->dma_done_handler(active_cs, ads1256_dma_ready);
    rtdm_master->ops->transfer_iobufs(active_cs);
    i_area = iobufs.io_virt + iobufs.i_offset;
    o_area = iobufs.io_virt + iobufs.o_offset;


}

static void ads1256_rtdm_search_device_on_spi(void * arg) {
    int device_id, res ,ret, speed_hz = 60000000, * return_value = (int)arg, j;
    struct spi_ioc_transfer ;
    struct rtdm_spi_config config;
    struct rtdm_spi_iobufs iobufs;
    struct _rtdm_mmap_request mmap_request;
    static unsigned char *i_area, *o_area;
    void *u_addr = NULL;
    int  irq_trigger = 0;
    unsigned int irq;
    ;


    struct spi_device *spi_device;
#define MY_BUS_NUM 0
    struct spi_master *master;
    struct device *dev;
    struct rtdm_spi_iobufs io_bufs;

    //Register information about your slave device:
    struct spi_board_info spi_device_info = {
            .modalias = "rtdm_spi_device",
            .max_speed_hz = 1, //speed your device (slave) can handle
            .bus_num = MY_BUS_NUM,
            .chip_select = 0,
            .mode = SPI_MODE_0,
    };



    /*To send data we have to know what spi port/pins should be used. This information
      can be found in the device-tree. */
    master = spi_busnum_to_master( spi_device_info.bus_num );
    rtdm_master = dev_get_drvdata(&master->dev);
    if( rtdm_master == NULL){
        printk("MASTER not found.\n");
        return;// -ENODEV;
    }else {
        printk("MASTER found.\n");

    }

    struct list_head *i;

    list_for_each(i, &rtdm_master->slaves){
        active_cs = list_entry(i, struct rtdm_spi_remote_slave, next);
        ads1256_spi_do_chip_select();

        struct rtdm_spi_config *config = &active_cs->config;
        config->bits_per_word = 8;
        config->mode = SPI_MODE_1;
        config->speed_hz = 3920000;
        rtdm_master->ops->configure(active_cs);


        printk(KERN_INFO "Search ADS1256 device on %s",active_cs->dev.name );


        ads1256_register_status_t register_address;
        register_address.value = 0;
        ads1256_rtdm_read_register_status(&register_address);
        printk(KERN_INFO "Elemet id: %u", register_address.elements.id);
        if (register_address.elements.id == ADS1256_DEVICE_ID) {
            u8 chanell = 0;
            ads1256_spi_do_chip_select();
            ads1256_rtdm_prepare_cmd(ADS1256_COMMAND_WREG, ADS1256_REGISTER_MUX, (chanell << 4) + AINCOM);
            ads1256_rtdm_send_cmd();
            udelay(10);

            ads1256_rtdm_prepare_cmd(ADS1256_COMMAND_SYNC, ADS1256_REGISTER_NONE, 0);
            ads1256_rtdm_prepare_cmd(ADS1256_COMMAND_WAKEUP, ADS1256_REGISTER_NONE, 0);
            ads1256_rtdm_send_cmd();
            udelay(10);

            ads1256_rtdm_prepare_cmd(ADS1256_COMMAND_RDATAC, ADS1256_REGISTER_NONE, 0);
            ads1256_rtdm_send_cmd();

            irq = gpio_to_irq(drdy_gpio_pin);
            irq_trigger |= IRQ_TYPE_EDGE_FALLING;

            if (irq_trigger)
                irq_set_irq_type(irq, irq_trigger);

            ads1256_dma_exampel();
           // ret = rtdm_irq_request(&irqh, irq, ads1256_drdy_is_low,
           //                        0 , "DRDY", NULL);

           // cpumask_t mask;
           // cpumask_set_cpu(3, &mask);

          //  xnintr_affinity(&irqh, mask);

           // rtdm_irq_enable(&irqh);

            printk(KERN_INFO "Found ADS1256 device on %s", active_cs->dev.name);
            *return_value = 0;

            return;
       }
        ads1256_spi_do_chip_deselect();
    }




    printk(KERN_ERR "Device ADS1256 not connected.\r\n");
    * return_value = -ENOENT;
    return;
}

int ads1256_rtdm_task_init(rtdm_task_t *task, const char *name,
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
    cpumask_t mask;
    cpumask_set_cpu(3, &mask);

    iattr.affinity = mask;
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






static void task_fn(void * arg){
    ktime_t start;

    int i = 0;
    uint8_t data[3];
    atomic_set(&drdy_is_low_status, 1);
    start = ktime_get( );
    while(!rtdm_task_should_stop()) {
        if((ktime_get().tv64- start.tv64) > 1000000000){
            start = ktime_get();
            rtdm_printk(KERN_ERR"Sample rate: %d", i);
            i = 0;
        }
        if (atomic_read(&drdy_is_low_status) == 1) {
            atomic_set(&drdy_is_low_status, 0);
            ads1256_read_n_bytes(data, 3);

            ring_buffer_npush_back(&(global_buffer), data, 3);
            i++;


        }
        rtdm_task_sleep(40);
    }
}

static void ads1256_rtdm_unregister_devices(void){
    int device_id ;
    for (device_id = 0; device_id < ADS1256_ADC_DEVICES_NUMBER; device_id++) {
        if(ads1256_ads_open_devices[device_id] != NULL){
            ads1256_ads_open_devices[device_id]->device_used = 0;
            ring_buffer_destroy(&(ads1256_ads_open_devices[device_id]->buffer));
            ads1256_ads_open_devices[device_id] = NULL;
        }
        printk(KERN_INFO "%s: Unregistering device acd0.%d  ...\r\n", __FUNCTION__, device_id);
        rtdm_dev_unregister(&ads1256_adc_devices[device_id]);
        printk(KERN_INFO "%s: Device acd0.%d unregistered  ...\r\n", __FUNCTION__, device_id);
    }
}

static int ads1256_prepare_gpio(void){
    gpio_request_one(drdy_gpio_pin, GPIOF_IN, "DRDY");
    gpio_request_one(reset_gpio_pin,GPIOF_OUT_INIT_HIGH, "RESET");
}

static int ads1256_reset(void){
    gpio_set_value(reset_gpio_pin,0);
    udelay(50);
    gpio_set_value(reset_gpio_pin, 1);
}



static int __init ads1256_rtdm_init(void)
{
    int res = -1;
    buffer_cmd.size = 0;
    ring_buffer_init(&(global_buffer), ADS1256_BUFFER_SIZE);

    printk(KERN_INFO "Real-Time A/C driver for the ADS1256 init!\n");
    ads1256_rtdm_init_ac_devices();
    ads1256_prepare_gpio();
    rtdm_task_init(&ads1256_search_ac_task, "search_device_on_spi", ads1256_rtdm_search_device_on_spi, &res, 99, 20000);
    rtdm_task_join(&ads1256_search_ac_task);
    printk(KERN_ERR "Result search %d" , res);
    if(res == 0) {
       // rtdm_lock_init(&global_lock);
        ads1256_rtdm_task_init(&ads1256_task_agent, "rtdm_agent", task_fn, NULL, 99, 0);


        device_init = true;
        return 0;
    }else{
        ads1256_rtdm_unregister_devices();
        return 0; /**To do fix value**/
    }


}

static void ads1256_free_gpios(void){
    gpio_free(drdy_gpio_pin);
    gpio_free(reset_gpio_pin);
}

static void __exit  ads1256_rtdm_exit(void)
{

    rtdm_master->ops->mmap_release(active_cs);
    ads1256_reset();
    ads1256_free_gpios();
    rtdm_irq_free(&irqh);
    ads1256_spi_do_chip_deselect();
    u32 result;


    const char* dump_filename = "/log.bin"; //Set to the file you are targeting
    struct file *file;
    int i;
    void* data;  //Needs to be a kernel pointer, not userspace pointer
    int block_count; //Set me to something
    int block_size; //Set me to something
    loff_t pos = 0;
    mm_segment_t old_fs;

    old_fs = get_fs();  //Save the current FS segment
    set_fs(get_ds());

    file = filp_open(dump_filename, O_WRONLY|O_CREAT, 0644);

    if(file){


        data= global_buffer.buf; //Wherever your data is

        vfs_write(file, data, global_buffer.cur_size, &pos);
        pos = pos+block_size;


        filp_close(file,NULL);
    }
    set_fs(old_fs);
    ring_buffer_destroy(&global_buffer);

    if(device_init == true){
        rtdm_task_destroy(&ads1256_task_agent);
        ads1256_rtdm_unregister_devices();
    }

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
