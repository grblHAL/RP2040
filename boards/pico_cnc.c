/*
  pico_cnc.c - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io
  
  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#if defined(BOARD_PICO_CNC)

static bool find_port (xbar_t *properties, uint8_t port, void *data)
{
    bool found;

    if((found = !properties->mode.claimed && properties->pin == (properties->cap.external ? 3 : 2)))
        *(uint8_t *)data = port;

    return found;
}

void board_init (void)
{
    uint8_t port;
#if defined(AUXOUTPUT0_PIN) && !(MODBUS_ENABLE & MODBUS_RTU_DIR_ENABLED)
    if(ioports_enumerate(Port_Digital, Port_Output, (pin_cap_t){}, find_port, &port))
        ioport_claim(Port_Digital, Port_Output, &port, "N/A");
#endif
#if N_AXIS == 3
    if(ioports_enumerate(Port_Digital, Port_Output, (pin_cap_t){ .external = On }, find_port, &port))
        ioport_claim(Port_Digital, Port_Output, &port, "N/A");
#endif
}

#endif // defined(BOARD_PICO_CNC)
