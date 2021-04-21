/* Copyright (C) 2021  Andreas Olofsson
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/timekeeping.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/termios.h>

#include "ring_buffer.c"
#include "uart.c"

#define MODULE_LOG_NAME "gpio_uart: "
#define DEVICE_NAME "gpio_uart"
#define MAX_DEVS 4

// Declarations

struct gpio_uart_dev
{
    uint pin;
    struct cdev cdev;
    ring_buffer ring;
    uart uart;
};

int init_uart_dev(uint index, uint pin);
void destroy_uart_dev(uint index);

static irqreturn_t irq_handler(int irq, void *arg);

// --- Params ---

static int get_pins(char *buffer, const struct kernel_param *kp);
static int set_pins(const char *val, const struct kernel_param *kp);

static struct kernel_param_ops pins_ops = {
    .get = &get_pins,
    .set = &set_pins,
};
module_param_cb(pins, &pins_ops, NULL, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);

// --- Devices ---

static dev_t first;
static struct class *gpio_uart_class;

static struct gpio_uart_dev devs[MAX_DEVS];
static uint dev_count = 0;

static ssize_t read_gpio_uart(struct file *filp, char __user *buf, size_t len, loff_t *off);
static long uart_gpio_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

static struct file_operations gpio_uart_fops = {
    .owner = THIS_MODULE,
    .read = read_gpio_uart,
    .unlocked_ioctl = uart_gpio_ioctl,
};

// Definitions
// --- Params ---

// Read the pins parameter.
// Which pin each gpio-uart device is bound to.
static int get_pins(char *buffer, const struct kernel_param *kp)
{
    int i = 0, j;
    int char_count;
    uint val;
    int dev_i;
    for (dev_i = 0; dev_i < dev_count; dev_i++)
    {
        if (dev_i != 0)
        {
            buffer[i++] = ',';
        }

        if (devs[dev_i].pin == 0)
        {
            buffer[i++] = '0';
        }
        else
        {
            char_count = 0;
            val = devs[dev_i].pin;
            while (val > 0)
            {
                char_count++;
                val /= 10;
            }

            val = devs[dev_i].pin;
            for (j = char_count - 1; val > 0; j--)
            {
                buffer[i + j] = '0' + val % 10;
                val /= 10;
            }
            i += char_count;
        }
    }
    buffer[i++] = 0;
    return i;
}

// Write the pins parameter.
// Remaps the pin each gpio-uart device is bound to.
static int set_pins(const char *val, const struct kernel_param *kp)
{
    int i;
    int ret;

    uint new_pins[MAX_DEVS];
    int pin_n = 0;

    int pin_val = 0;
    int pin_chars = 0;

    const char *c;
    for (c = val; *c != 0; c++)
    {
        if (*c == ',')
        {
            if (pin_chars && pin_n < MAX_DEVS - 1)
            {
                new_pins[pin_n] = pin_val;
                pin_n++;
                pin_val = 0;
                pin_chars = 0;
            }
            else
            {
                return -EINVAL;
            }
        }
        else if ('0' <= *c && *c <= '9')
        {
            pin_val *= 10;
            pin_val += *c - '0';
            pin_chars++;
        }
        else if (*c != ' ' && *c != '\t' && *c != '\r' && *c != '\n')
        {
            return -EINVAL;
        }
    }

    if (pin_chars)
    {
        new_pins[pin_n] = pin_val;
        pin_n++;
    }
    else if (pin_n != 0)
    {
        return -EINVAL;
    }

    for (i = 0; i < dev_count; i++)
    {
        if (i >= pin_n || devs[i].pin != new_pins[i])
        {
            printk(KERN_INFO MODULE_LOG_NAME "Unbinding device %d from pin %d\n", i, devs[i].pin);
            destroy_uart_dev(i);
        }
    }
    for (i = 0; i < pin_n; i++)
    {
        if (i >= dev_count || devs[i].pin != new_pins[i])
        {
            printk(KERN_INFO MODULE_LOG_NAME "Binding device %d to pin %d\n", i, new_pins[i]);
            if ((ret = init_uart_dev(i, new_pins[i])) < 0)
            {
                return ret;
            }
        }
    }
    dev_count = pin_n;

    return 0;
}

// --- Devices ---

/// Initialize a gpio-uart cdev and setup interrupts
int init_uart_dev(uint index, uint pin)
{
    int ret;
    struct device *dev;
    int irq;

    if (IS_ERR_VALUE(dev = device_create(gpio_uart_class, NULL, first + index, NULL, DEVICE_NAME "%d", index)))
    {
        printk(KERN_ERR MODULE_LOG_NAME "Failed to create device " DEVICE_NAME "%d with error %ld\n ", index, PTR_ERR(dev));
        return PTR_ERR(dev);
    }

    // initialize gpio-uart dev
    devs[index].pin = pin;
    devs[index].ring.read = devs[index].ring.write = 0;
    uart_init(&devs[index].uart, 9600);

    // initialize cdev
    cdev_init(&devs[index].cdev, &gpio_uart_fops);

    if ((ret = cdev_add(&devs[index].cdev, first + index, 1)) < 0)
    {
        printk(KERN_ERR MODULE_LOG_NAME "Failed to add device " DEVICE_NAME "%d\n", index);
        device_destroy(gpio_uart_class, first + index);
        return ret;
    }

    // setup interrupts
    irq = gpio_to_irq(pin);
    if ((ret = request_irq(irq, irq_handler, IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "gpio interrupt", &devs[index])) != 0)
    {
        printk(KERN_ERR MODULE_LOG_NAME "Failed to register interrupts for " DEVICE_NAME "%d\n", index);
        cdev_del(&devs[index].cdev);
        device_destroy(gpio_uart_class, first + index);
        return ret;
    }

    printk(KERN_INFO MODULE_LOG_NAME "Created device " DEVICE_NAME "%d\n ", index);
    return 0;
}

/// Destroy a gpio-uart cdev
void destroy_uart_dev(uint index)
{
    int irq = gpio_to_irq(devs[index].pin);
    free_irq(irq, &devs[index]);
    cdev_del(&devs[index].cdev);
    device_destroy(gpio_uart_class, first + index);
    printk(KERN_INFO MODULE_LOG_NAME "Removed device " DEVICE_NAME "%d\n ", index);
}

/// Handle interrupts
static irqreturn_t irq_handler(int irq, void *arg)
{
    struct gpio_uart_dev *dev = (struct gpio_uart_dev *)arg;
    int high = gpio_get_value(dev->pin);
    int b;

    if (high)
    {
        b = uart_update_state(&dev->uart, 1);
    }
    else
    {
        b = uart_update_state(&dev->uart, 0);
    }

    if (b != -1)
    {
        ring_buffer_write_byte(&dev->ring, b);
    }

    return IRQ_HANDLED;
}

/// Read from a gpio-uart cdev
static ssize_t read_gpio_uart(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
    int index = filp->f_inode->i_rdev - first;
    struct gpio_uart_dev *dev = &devs[index];

    int b = uart_finish_byte(&dev->uart);
    if (b != -1)
    {
        ring_buffer_write_byte(&dev->ring, b);
    }

    return ring_buffer_copy_to_user(&dev->ring, buf, len);
}

static long uart_gpio_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int index = filp->f_inode->i_rdev - first;
    struct gpio_uart_dev *dev = &devs[index];
    int i;

    if (cmd == TCGETS)
    {
        struct ktermios termios;
        int baud;
        switch (dev->uart.baud_rate)
        {
            case 50: baud = B50; break;
            case 75: baud = B75; break;
            case 110: baud = B110; break;
            case 134: baud = B134; break;
            case 150: baud = B150; break;
            case 200: baud = B200; break;
            case 300: baud = B300; break;
            case 600: baud = B600; break;
            case 1200: baud = B1200; break;
            case 1800: baud = B1800; break;
            case 2400: baud = B2400; break;
            case 4800: baud = B4800; break;
            case 9600: baud = B9600; break;
            case 19200: baud = B19200; break;
            case 38400: baud = B38400; break;
            case 57600: baud = B57600; break;
            case 115200: baud = B115200; break;
            case 230400: baud = B230400; break;
            case 460800: baud = B460800; break;
            case 500000: baud = B500000; break;
            case 576000: baud = B576000; break;
            case 921600: baud = B921600; break;
            case 1000000: baud = B1000000; break;
            case 1152000: baud = B1152000; break;
            case 1500000: baud = B1500000; break;
            case 2000000: baud = B2000000; break;
            case 2500000: baud = B2500000; break;
            case 3000000: baud = B3000000; break;
            case 3500000: baud = B3500000; break;
            case 4000000: baud = B4000000; break;
            default: baud = BOTHER; break;
        }
        termios.c_iflag = IGNBRK | IGNPAR;
        termios.c_oflag = 0;
        termios.c_cflag = baud | CS8 | CREAD | CLOCAL;
        termios.c_lflag = 0;
        termios.c_line = 0;
        for (i = 0; i < sizeof(termios.c_cc) / sizeof(termios.c_cc[0]); i++)
        {
            termios.c_cc[i] = '\0';
        }
        termios.c_ispeed = dev->uart.baud_rate;
        termios.c_ospeed = dev->uart.baud_rate;
        kernel_termios_to_user_termios_1((struct termios *)arg, &termios);

        return 0;
    }
    else if (cmd == TCSETS || cmd == TCSETSW)
    {
        struct ktermios termios;
        int speed;
        user_termios_to_kernel_termios_1(&termios, (struct termios *)arg);
        switch (termios.c_cflag & (CBAUD | CBAUDEX))
        {
            case B50: speed = 50; break;
            case B75: speed = 75; break;
            case B110: speed = 110; break;
            case B134: speed = 134; break;
            case B150: speed = 150; break;
            case B200: speed = 200; break;
            case B300: speed = 300; break;
            case B600: speed = 600; break;
            case B1200: speed = 1200; break;
            case B1800: speed = 1800; break;
            case B2400: speed = 2400; break;
            case B4800: speed = 4800; break;
            case B9600: speed = 9600; break;
            case B19200: speed = 19200; break;
            case B38400: speed = 38400; break;
            case BOTHER: speed = termios.c_ispeed; break;
            case B57600: speed = 57600; break;
            case B115200: speed = 115200; break;
            case B230400: speed = 230400; break;
            case B460800: speed = 460800; break;
            case B500000: speed = 500000; break;
            case B576000: speed = 576000; break;
            case B921600: speed = 921600; break;
            case B1000000: speed = 1000000; break;
            case B1152000: speed = 1152000; break;
            case B1500000: speed = 1500000; break;
            case B2000000: speed = 2000000; break;
            case B2500000: speed = 2500000; break;
            case B3000000: speed = 3000000; break;
            case B3500000: speed = 3500000; break;
            case B4000000: speed = 4000000; break;
            default: speed = termios.c_ispeed; break;
        }
        printk(KERN_INFO MODULE_LOG_NAME "Setting baud rate for " DEVICE_NAME "%d to %d\n", index, speed);
        uart_init(&dev->uart, speed);
        return 0;
    }

    return -1;
}

// Setup

int __init init_module(void)
{
    printk(KERN_INFO MODULE_LOG_NAME "Loading gpio-uart module.\n");
    if (alloc_chrdev_region(&first, 0, MAX_DEVS, DEVICE_NAME) < 0)
    {
        printk(KERN_ERR MODULE_LOG_NAME "Failed to allocate devices\n");
        return -1;
    }
    printk(KERN_INFO MODULE_LOG_NAME "Allocated %d:%d-%d for gpio-uart.\n", MAJOR(first), MINOR(first), MINOR(first) + MAX_DEVS - 1);
    if ((gpio_uart_class = class_create(THIS_MODULE, DEVICE_NAME)) == NULL)
    {
        printk(KERN_ERR MODULE_LOG_NAME "Failed to create class %s\n", DEVICE_NAME);
        return -EINVAL;
    }
    return 0;
}

void __exit exit_module(void)
{
    int i;
    printk(KERN_INFO MODULE_LOG_NAME "Unloading gpio-uart module.\n");
    for (i = 0; i < dev_count; i++)
    {
        destroy_uart_dev(i);
    }
    class_destroy(gpio_uart_class);
    unregister_chrdev_region(first, MAX_DEVS);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andreas Olofsson <ao@blazar.se>");
MODULE_DESCRIPTION("UART driver using GPIO");
MODULE_VERSION("0.1.0");
