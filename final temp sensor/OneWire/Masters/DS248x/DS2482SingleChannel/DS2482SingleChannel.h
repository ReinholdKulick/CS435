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


#ifndef OneWire_Masters_DS2482_Single_Channel
#define OneWire_Masters_DS2482_Single_Channel


#include "Masters/DS248x/DS248x.h"


namespace OneWire
{
    ///DS2482_100 I2C to 1-wire Master
    class DS2482SingleChannel : public DS248x
    {
    public:
        ///Valid slave addresses for DS2482-100
        enum I2CAddress
        {
            I2C_ADRS0 = 0x30,
            I2C_ADRS1,
            I2C_ADRS2,
            I2C_ADRS3
        };
        
        /// Construct to use an existing I2C interface.
        /// @param i2c_bus Configured I2C communication interface for DS248x.
        /// @param adrs I2C bus address of the DS248x in mbed format.
        DS2482SingleChannel(mbed::I2C & i2c_bus, uint8_t adrs) : DS248x(i2c_bus, adrs) { }
    };
}


#endif /* OneWire_Masters_DS2482_Single_Channel */