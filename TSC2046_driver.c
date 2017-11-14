 /* @file TSC2046_driver.c
 * @author Alexander Hirschfeld
 * @date   31 Oct, 2017
 * @brief  A kernel module for controlling the TSC2046 Touch Screen IC. 
 * This uses an 8bit SPI command and receives an 8 or 12bit response.
 * @version v0.1
 * @liscense GPL
 */
 
 /*
 PENIRQ goes low when touch is registered, using this as interrupt on a pin to know when to start reading.
 
 Command bit string:
 
 7  6   5   4   3   2   1   0
 1  A2  A1  A0  M   SER PD1 PD0
 
 7: Start bit, always 1
 A2-0: Mux Select 
 
 M: Mode: 12 bit response when low, 8 bit when high, speed of conversion, and accuracy of touch screen
 
 SER/`DFR Single refrence or diffrential refrence, When high, it is single refrence, low id diffrential, see datasheet
 http://www.ti.com/lit/ds/symlink/tsc2046.pdf
 
 PD1-0 Power down mode, what to do after successful read.
 
 A2-0
 2 1 0: result
 0 0 1: Measure y
 0 1 1: Measure z1
 1 0 0: Measure z2
 1 0 1: Measure x
 
 PD1-0
 1 0: result
 0 0: penirq enabled, power down between conversions
 0 1: penirq disabled, refrence is off, adc is on
 1 0: penirq enabled, refrence is on, adc is off
 1 1: penirq diabled, refrence is on, adc is on.
 
 */
 
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/stat.h>
#include <linux/sysfs.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <linux/delay.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alex Hirschfeld");
MODULE_DESCRIPTION("SPI driver for TSC2046");
MODULE_VERSION("0.1");

#define DRIVER_NAME "TSC2046"

#define GPIO_TO_PIN(bank, pin) ((32 * bank) + pin);
#define DEVICE_NAME "TSC2046"
#define SPI_BUS_SPEED 100000 //100k

#define CREATE_COMMAND(ADC) ((1<<7 | (ADC & 0x7)<<4 | (mode & 0x1)<<3 | (SER & 0x1)<<2 |(device_obj->PD_vals & 0x3)))

static struct spi_device *spi_dev;

static short penirq;
static short spiport;
static short spics;
static short mode;
static short SER;

static unsigned int irqNumber; 

static struct workqueue_struct* wq;
static struct work_struct main_spi_work_struct;

const unsigned char adc_vals[] = {0b001, 0b101, 0b011, 0b100};
struct screenVals {
    int x;
    int y;
    int z1;
    int z2;
    bool touch;
    unsigned char i;
};

struct diffVals {
    int start_x;
    int start_y;
    int start_z1;
    int start_z2;
    long long start_time_ns;
    int diff_x;
    int diff_y;
    int diff_z1;
    int diff_z2;
    long long time_down_ns;
};

struct oneshotVals {
    unsigned char command;
    unsigned char data[2];
};

struct runningVals {
    unsigned char command;
    unsigned char data[2];
};

// Interrupts
static irq_handler_t penirq_interrupt_handler(unsigned int irq, void* dev_id, struct pt_regs *reg);

//helper function
long long get_time_ns (void)
{   struct timespec ts = CURRENT_TIME;
    return timespec_to_ns(&ts); 
}


// Kobject and sysfs
static struct kset *TSC2046_kset;
static struct TSC2046_obj* device_obj;

struct TSC2046_obj {
    struct kobject kobj;
    struct screenVals vals;
    struct diffVals diffs;
    struct oneshotVals oneshot;
    struct runningVals data;
    unsigned char PD_vals;
    bool running;
    struct work_struct get_value; // for workqueue
};
#define to_TSC2046_obj(x) container_of(x, struct TSC2046_obj, kobj)

struct TSC2046_attr {
    struct attribute attr;
    ssize_t (*show)(struct TSC2046_obj* obj, struct TSC2046_attr* attr, char* buf);
    ssize_t (*store)(struct TSC2046_obj* obj, struct TSC2046_attr* attr, const char* buf, size_t count);
};
#define to_TSC2046_attr(x) container_of(x, struct TSC2046_attr, attr);

static ssize_t TSC2046_vals_show(struct TSC2046_obj* obj, struct TSC2046_attr* attr, char* buf) {
    struct screenVals* vals;
    vals = &obj->vals;
    
    return sprintf(buf, "x\t%d\ny\t%d\nz1\t%d\nz2\t%d\nactive\t%d\n",
        vals->x,
        vals->y,
        vals->z1,
        vals->z2,
        vals->touch);
}

static ssize_t TSC2046_diffs_show(struct TSC2046_obj* obj, struct TSC2046_attr* attr, char* buf) {
    struct diffVals* vals;
    vals = &obj->diffs;
    
    return sprintf(buf, "x\t%d\ny\t%d\nz1\t%d\nz2\t%d\ntime\t%lld\n",
        vals->diff_x,
        vals->diff_y,
        vals->diff_z1,
        vals->diff_z2,
        (vals->time_down_ns) ? vals->time_down_ns : (get_time_ns() - vals->start_time_ns));
}

static ssize_t TSC2046_batt_show(struct TSC2046_obj* obj, struct TSC2046_attr* attr, char* buf) {
    char command;
    char data[2];
    
    command = CREATE_COMMAND(0b010);
    spi_write_then_read(spi_dev, (void*)&command, 1, (void*)&data, 2);
    return sprintf(buf, "0x%x%x\n", data[1], data[0]);
    
}

static ssize_t TSC2046_temp_show(struct TSC2046_obj* obj, struct TSC2046_attr* attr, char* buf) {
    char command;
    char data[2];
    
    command = CREATE_COMMAND(0b000);
    spi_write_then_read(spi_dev, (void*)&command, 1, (void*)&data, 2);
    return sprintf(buf, "0x%x%x\n", data[1], data[0]);
}

static ssize_t TSC2046_PD_show(struct TSC2046_obj* obj, struct TSC2046_attr* attr, char* buf) {
    return sprintf("%hhu\n", &obj->PD_vals);
}

static ssize_t TSC2046_PD_store(struct TSC2046_obj* obj, struct TSC2046_attr* attr, const char* buf, size_t count) {
    unsigned char new_pd;
    sscanf(buf, "%hhu", &new_pd);
    obj->PD_vals = new_pd&0x3;
    return count;
}

static ssize_t TSC2046_oneshot_show(struct TSC2046_obj* obj, struct TSC2046_attr* attr, char* buf) {
    struct oneshotVals* vals;
    vals = &obj->oneshot;
    
    spi_write_then_read(spi_dev, (void*)&vals->command, 1, (void*)&vals->data, 2);
    
    return sprintf(buf, "command\t0x%02x\ndata\t0x%04x\n",
        (unsigned int)vals->command,
        (unsigned int)(vals->data[0]<<8 | vals->data[1]));
}

static ssize_t TSC2046_oneshot_store(struct TSC2046_obj* obj, struct TSC2046_attr* attr, const char* buf, size_t count) {
    struct oneshotVals* vals;
    unsigned int adc_val;
    
    vals = &obj->oneshot;
    if (count != 2) {
        return -1;
    }
    sscanf(buf, "%d", &adc_val);
    vals->command = CREATE_COMMAND(adc_val);
    return count;
}

static void TSC2046_release(struct kobject* kobj) {
    struct TSC2046_obj* obj;
    obj = to_TSC2046_obj(kobj);
    kfree(obj);
}

static struct TSC2046_attr vals_attr = 
    __ATTR(vals, S_IRUSR | S_IRGRP | S_IROTH, TSC2046_vals_show, NULL);
static struct TSC2046_attr diffs_attr = 
    __ATTR(diffs, S_IRUSR | S_IRGRP | S_IROTH, TSC2046_diffs_show, NULL);
static struct TSC2046_attr batt_attr = 
    __ATTR(battery, S_IRUSR | S_IRGRP | S_IROTH, TSC2046_batt_show, NULL);
static struct TSC2046_attr temp_attr = 
    __ATTR(temperature, S_IRUSR | S_IRGRP | S_IROTH, TSC2046_temp_show, NULL);
    // It is a bad idea to make sys files writable to all, using 664 for these, sorry
static struct TSC2046_attr PD_attr = 
    __ATTR(PD_select, S_IWUSR | S_IRUSR | S_IRGRP | S_IWGRP | S_IROTH, TSC2046_PD_show, TSC2046_PD_store);
static struct TSC2046_attr oneshot_attr = 
    __ATTR(oneshot, S_IWUSR | S_IRUSR | S_IRGRP | S_IWGRP | S_IROTH, TSC2046_oneshot_show, TSC2046_oneshot_store);
    
static struct attribute *TSC2046_default_attrs[] = {
    &vals_attr.attr,
    &diffs_attr.attr,
    &batt_attr.attr,
    &temp_attr.attr,
    &PD_attr.attr,
    &oneshot_attr.attr,
    NULL
};

static ssize_t TSC2046_attr_show(struct kobject* kobj, struct attribute *attr, char* buf) {
    struct TSC2046_attr* attribute;
    struct TSC2046_obj* obj;
    
    attribute = to_TSC2046_attr(attr);
    obj = to_TSC2046_obj(kobj);
    
    if (!attribute->show)
    
        return -EIO;
    return attribute->show(obj, attribute, buf);
}

static ssize_t TSC2046_attr_store(struct kobject* kobj, struct attribute* attr, const char* buf, size_t count) {
    struct TSC2046_attr* attribute;
    struct TSC2046_obj* obj;
    
    attribute = to_TSC2046_attr(attr);
    obj = to_TSC2046_obj(kobj);
    
    if (!attribute->store)
        return -EIO;
    return attribute->store(obj, attribute, buf, count);
}

static struct sysfs_ops TSC2046_sysfs_ops = {
    .show = TSC2046_attr_show,
    .store = TSC2046_attr_store
};

static struct kobj_type TSC2046_ktype = {
    .sysfs_ops = &TSC2046_sysfs_ops,
    .release = &TSC2046_release,
    .default_attrs = TSC2046_default_attrs
};


static struct TSC2046_obj* init_TSC2046_obj(const char* name) {
    struct TSC2046_obj* obj;
    int retval;
    
    obj = kzalloc(sizeof(*obj), GFP_KERNEL);
    if (!obj) {
        return NULL;
    }
    
    obj->kobj.kset = TSC2046_kset;
    retval = kobject_init_and_add(&obj->kobj, &TSC2046_ktype, NULL, "%s", name);
    if (retval) {
        kobject_put(&obj->kobj);
        return NULL;
    }
    kobject_uevent(&obj->kobj, KOBJ_ADD);
    return obj;
}

static void destroy_TSC2046_obj(struct TSC2046_obj* obj) {
    kobject_put(&obj->kobj);
}


// Module Params
module_param(spics, short, S_IRUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(spics, "Chip Select");
module_param(spiport, short, S_IRUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(spiport, "SPI port");
module_param(penirq, short, S_IRUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(penirq, "Pen Interrupt Port");
module_param(mode, short, S_IRUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(mode, "8 or 12 bit mode");
module_param(SER, short, S_IRUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(SER, "single or dual channel mode");


// Work Queue Stuff
static void main_spi_work_handler(struct work_struct* ws) {
    struct screenVals* vals;
    struct diffVals* diffs;
    unsigned char command;
    unsigned char data[2];
    //printk(KERN_INFO "TSC2046: Main Work Handler Called\n");
    vals = &device_obj->vals;
    diffs = &device_obj->diffs;
    while (vals->touch) {
        command = CREATE_COMMAND(adc_vals[vals->i]);
        spi_write_then_read(spi_dev, (void*)&command, 1, (void*)&data, 2);
        //printk(KERN_DEBUG "%x :: %x %x", command, data[0], data[1]);
        switch(vals->i) {
            case 0:
                vals->y = (int)(data[1]<<4 | data[0]>>4);
                if(!diffs->start_y)
                    diffs->start_y = vals->y;
                diffs->diff_y = vals->y - diffs->start_y;
                break;
            case 1:
                vals->x = (int)(data[1]<<4 | data[0]>>4);
                if(!diffs->start_x)
                    diffs->start_x = vals->x;
                diffs->diff_x = vals->x - diffs->start_x;
                break;
            case 2:
                vals->z1 = (int)(data[1]<<4 | data[0]>>4);
                if(!diffs->start_z1)
                    diffs->start_z1 = vals->z1;
                diffs->diff_z1 = vals->z1 - diffs->start_z1;
                break;
            case 3:
                vals->z2 = (int)(data[1]<<4 | data[0]>>4);
                if(!diffs->start_z2)
                    diffs->start_z2 = vals->z2;
                diffs->diff_z2 = vals->z2 - diffs->start_z2;
                break;
            default:
                vals->i = -1;
        }
        vals->i = (vals->i + 1)%4;
        msleep(10);
    }
}


// Initializations
static int __init initialization(void){
    int status;
    
    struct spi_master* master;
    struct device* temp;
    char buff[20];
    
    printk(KERN_INFO "TSC2046: Initializing\n");
    
    printk(KERN_INFO "TSC2046: Params :: spics: %hd :: spiport:%hd :: penirq: %hd :: mode: %hd :: SER: %hd\n", spics, spiport, penirq, mode, SER);
    
    if (spics == 0 && spiport == 0 && penirq == 0) {
        printk(KERN_INFO "TSC2046: Invalid params, please values for spics, spiport, and penirq\n");
        return -EINVAL;
    }
    
    wq = create_singlethread_workqueue("TSC2046 Work Queue");
    if (!wq) {
        destroy_workqueue(wq);
        printk(KERN_INFO "TSC2046: Unable to create workqueue");
        return -EINVAL;
    }
    
    TSC2046_kset = kset_create_and_add(DRIVER_NAME, NULL, firmware_kobj);
    if (!TSC2046_kset) {
        kset_unregister(TSC2046_kset);
        printk(KERN_INFO "TSC2046: Unable to register kset in firmware\n");
        destroy_workqueue(wq);
        return -EINVAL;
    }
    
    device_obj = init_TSC2046_obj("device");
    if (!device_obj) {
        destroy_TSC2046_obj(device_obj);
        kset_unregister(TSC2046_kset);
        printk(KERN_INFO "TSC2046: Driver init failed with init_TSC2046_obj\n");
        destroy_workqueue(wq);
        return -EINVAL;
    }
    
    if (penirq == 0) {
        destroy_TSC2046_obj(device_obj);
        kset_unregister(TSC2046_kset);
        printk(KERN_INFO "TSC2046: Please pass a valid pin for penirq\n");
        destroy_workqueue(wq);
        return -EINVAL;
    }
    
    //printk(KERN_INFO "TS2046:passed penirq check\n");
    master = spi_busnum_to_master(spiport);
    if (!master) {
        destroy_TSC2046_obj(device_obj);
        kset_unregister(TSC2046_kset);
        printk(KERN_INFO "TS2046: Init Failed, Bad Master port, %dh", spiport);
        destroy_workqueue(wq);
        return -EINVAL;
    }
    //printk(KERN_INFO "TS2046:pass busnum to master\n");
    
    spi_dev = spi_alloc_device(master);
    if (!spi_dev) {
        destroy_TSC2046_obj(device_obj);
        kset_unregister(TSC2046_kset);
        put_device(&master->dev);
        printk(KERN_INFO "TSC2046: Init Failed, Bad Device\n");
        destroy_workqueue(wq);
        return -EINVAL;
    }
    //printk(KERN_INFO "TS2046:passed spi_alloc\n");
    spi_dev->chip_select = spics;
    snprintf(buff, sizeof(buff), "%s.%u", dev_name(&spi_dev->master->dev), spi_dev->chip_select);
    
    // Attempt to find the device, and if found hijack it.
    temp = bus_find_device_by_name(spi_dev->dev.bus, NULL, buff);
    if (temp) {
        spi_unregister_device(to_spi_device(temp));
        spi_dev_put(to_spi_device(temp));
    }
    //printk(KERN_INFO "TS2046:passed device busy\n");
    spi_dev->max_speed_hz = SPI_BUS_SPEED;
    spi_dev->mode = SPI_MODE_0;
    spi_dev->bits_per_word = 8;
    spi_dev->irq = -1;
    spi_dev->controller_data = NULL;
    spi_dev->controller_state = NULL;
    strcpy(spi_dev->modalias, DRIVER_NAME);
    
    status = spi_add_device(spi_dev);
    
    if (status < 0) {
        destroy_TSC2046_obj(device_obj);
        kset_unregister(TSC2046_kset);
        spi_dev_put(spi_dev);
        put_device(&master->dev);
        printk(KERN_INFO "TS2046: Init Failed: Failed to add device\n");
        destroy_workqueue(wq);
        return status;
    }
    //printk(KERN_INFO "TS2046:pass spi add\n");
    
    // GPIO and INterrupt
    gpio_request(penirq, "sysfs");
    gpio_direction_input(penirq);
    gpio_set_debounce(penirq, 200);
    gpio_export(penirq, false);
    
    irqNumber = gpio_to_irq(penirq);
    
    status = request_irq(irqNumber,             // The interrupt number requested
                        (irq_handler_t) penirq_interrupt_handler, // The pointer to the handler function below
                        IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,   // Interrupt on rising edge (button press, not release)
                        "penirq_interrupt_handler",    // Used in /proc/interrupts to identify the owner
                        NULL);
                        
                        
    // Work Queue initialization
    
    
    printk(KERN_INFO "TSC2046: Initializations Done\n");
    return status;
}

static void __exit exiting(void) {
    spi_dev_put(spi_dev);
    free_irq(irqNumber, NULL);
    destroy_TSC2046_obj(device_obj);
    kset_unregister(TSC2046_kset);
    gpio_free(penirq);
    destroy_workqueue(wq);
    printk(KERN_INFO "TSC2046: Goodbye\n");
    
}

static irq_handler_t penirq_interrupt_handler(unsigned int irq, void *dev_id, struct pt_regs *regs) {
    struct screenVals* vals;
    struct diffVals* diffs;
    
    printk(KERN_INFO "TSC2046: Trigger for IRQ, state: %d\n", gpio_get_value(penirq));
    
    vals = &device_obj->vals;
    diffs = &device_obj->diffs;
    if (gpio_get_value(penirq)) { // if it is a 1, it is released;
        vals->touch = false;
        diffs->time_down_ns = get_time_ns() - diffs->start_time_ns;
    } else { // if it is a 0, it is pressed
        vals->touch = true;
        diffs->time_down_ns = 0;
        diffs->start_time_ns = get_time_ns();
    }
    INIT_WORK(&main_spi_work_struct, main_spi_work_handler);
    schedule_work(&main_spi_work_struct);
    return (irq_handler_t) IRQ_HANDLED;
}

module_init(initialization);
module_exit(exiting);