## RP2040 Driver

A grblHAL driver for the Raspberry Pi Pico RP2040 processor on a [Pi Pico or Pi Pico W board](https://www.raspberrypi.org/products/raspberry-pi-pico/).

__Important__ download information can be found [here](https://github.com/grblHAL/core/wiki/Compiling-grblHAL).

The default build environment is Visual Studio Code, it is surprisingly easy to set up on Raspberry Pi - see the [Getting started](https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf) documentation for how to for this and other platforms.

__Note:__ Before enabling mDNS and/or SSDP protocols for Pico W buggy code in the SDK has to be removed.  
The broken code is in [cyw43_lwip.c](https://github.com/georgerobotics/cyw43-driver/blob/195dfcc10bb6f379e3dea45147590db2203d3c7b/src/cyw43_lwip.c#L176-L184), either remove this part of the code or inactivate it by adding a `x` in front of `LWIP_MDNS_RESPONDER` in line 176.  
There are two issues with this code: first is that it is for a newer version of lwIP than is supplied with the SDK, second is that mDNS is enabled with a default hostname - not the run-time configurable hostname in from grblHAL.

---
2022-10-28
