# GPIO-UART

A linux kernel module driver that turns up to four GPIO pins into UART devices. It's currently highly experimental, input only and doesn't support most common serial features. It's only tested using a single device using a baud rate of 9600.

This project was written to allow more than one serial input for raspberry pis.

## Building

`make ARCH=<ARCH> CROSS_COMPILE=<TOOLCHAIN_PREFIX> KDIR=<PATH_TO_LINUX_SOURCE>`

Building for a raspberry pi zero/1/2/3, this means using:  
`make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- KDIR=<PATH>`

## Configuring

The number of devices exposed and the pins they are bound to is configured using the module parameter `pins`, which can be set while loading the module or by writing to `/sys/module/gpio_uart/parameters/pins`.

Setting `pins` to `"1,2,3"` creates the devices `gpio_uart0`, `gpio_uart1`, and `gpio_uart2` and binds them to pins 1, 2, and 3, respectively.

The baud rate can be set using ioctl (or using `stty`) like any other serial device.
