/**********************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
**********************************************************************/

#include "Masters/DS248x/DS2484/DS2484.h"

using OneWire::OneWireMaster;
using OneWire::DS2484;

//*********************************************************************
DS2484::DS2484(mbed::I2C & i2c_bus, uint8_t adrs) : DS248x(i2c_bus, adrs)
{
}

//*********************************************************************
OneWireMaster::CmdResult DS2484::adjustOwPort(OwAdjustParam param, uint8_t val)
{
    OneWireMaster::CmdResult result;

    uint8_t read_port_config;
    uint8_t control_byte;

    control_byte = (((param & 0x0F) << 4) | (val & 0x0F));

    result = sendCommand(AdjustOwPortCmd, control_byte);
    if (result != Success)
    {
        return result;
    }

    result = readRegister(PortConfigReg, read_port_config, true);
    if (result != Success)
    {
        return result;
    }

    if ((control_byte & 0x0F) != read_port_config)
    {
        result = OneWireMaster::OperationFailure;
    }

    return result;
}