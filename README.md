## RP2040 Driver

A grblHAL driver for the Raspberry Pi Pico RP2040 processor on a [Pi Pico or Pi Pico W board](https://www.raspberrypi.org/products/raspberry-pi-pico/).

This driver can be built with the [Web Builder](https://svn.io-engineering.com:8443/?driver=RP2040&board=PicoCNC).

__Important__ download information can be found [here](https://github.com/grblHAL/core/wiki/Compiling-grblHAL).  
The project has been updated to/now uses [SDK version 2.1.1](https://github.com/raspberrypi/pico-sdk/releases).

The default build environment is Visual Studio Code, it is surprisingly easy to set up on Raspberry Pi - see the [Getting started](https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf) documentation for how to for this and other platforms.

> [!NOTE]
> The RP2350B_5X board uses a RP2350B processor, to build the firmware with Visual Studio Code choose the _pimoroni_pga2350_ board.
For other boards choose the _pico_, _pico\_w_, _pico2_ or _pico2\_w_ board that matches the processor or Pico board used.  
The board is selected in the lower right corner of the UI.

---
2025-12-31
