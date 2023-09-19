/*
  MCP3221.c - analog input from a MCP3221 I2C ADC

  Driver code for RP2040 processor

  Part of grblHAL

  Copyright (c) 2021-2023 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "i2c.h"
#include "MCP3221.h"

uint16_t MCP3221_read (void)
{
    uint8_t value[2];

    i2c_receive(MCP3221_ADDRESS, value, 2, true);

    return (value[0] << 8) | value[1];
}

bool MCP3221_init (void)
{
    i2c_init();

    return i2c_send(MCP3221_ADDRESS, NULL, 0, true);
}
