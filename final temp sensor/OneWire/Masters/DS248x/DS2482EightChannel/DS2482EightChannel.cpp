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


#include "Masters/DS248x/DS2482EightChannel/DS2482EightChannel.h"

using OneWire::OneWireMaster;
using OneWire::DS2482EightChannel;


//*********************************************************************
DS2482EightChannel::DS2482EightChannel(mbed::I2C & i2c_bus, uint8_t adrs) : DS248x(i2c_bus, adrs)
{
}


//*********************************************************************
OneWireMaster::CmdResult DS2482EightChannel::selectChannel(uint8_t channel)
{
    OneWireMaster::CmdResult result;
    uint8_t ch, ch_read;

    // Channel Select (Case A)
    //   S AD,0 [A] CHSL [A] CC [A] Sr AD,1 [A] [RR] A\ P
    //  [] indicates from slave
    //  CC channel value
    //  RR channel read back

    switch (channel)
    {
    default:
    case 0:
        ch = 0xF0;
        ch_read = 0xB8;
        break;
    case 1:
        ch = 0xE1;
        ch_read = 0xB1;
        break;
    case 2:
        ch = 0xD2;
        ch_read = 0xAA;
        break;
    case 3:
        ch = 0xC3;
        ch_read = 0xA3;
        break;
    case 4:
        ch = 0xB4;
        ch_read = 0x9C;
        break;
    case 5:
        ch = 0xA5;
        ch_read = 0x95;
        break;
    case 6:
        ch = 0x96;
        ch_read = 0x8E;
        break;
    case 7:
        ch = 0x87;
        ch_read = 0x87;
        break;
    };

    result = sendCommand(ChannelSelectCmd, ch);
    if (result == OneWireMaster::Success)
    {
        result = readRegister(ChannelSelectReg, ch, true);
        if (result == OneWireMaster::Success)
        {
            // check for failure due to incorrect read back of channel
            if (ch != ch_read)
            {
                result = OneWireMaster::OperationFailure;
            }
        }
    }

    return result;
}
