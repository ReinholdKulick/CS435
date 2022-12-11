/******************************************************************//**
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

#ifndef OneWire_Masters_DS2480B
#define OneWire_Masters_DS2480B

#include "Serial.h"
#include "Masters/OneWireMaster.h"

namespace OneWire
{
    /// Serial to 1-Wire Line Driver
    class DS2480B : public OneWireMaster
    {
    public:
        /// Baud rates for DS2480B
        enum BaudRate
        {
            Baud9600bps = 0, ///< 9600 bps
            Baud19200bps = 2, ///< 19200 bps
            Baud57600bps = 4, ///< 57600 bps
            Baud115200bps = 6 ///< 115200 bps
        };

        DS2480B(PinName tx, PinName rx);

        virtual OneWireMaster::CmdResult OWInitMaster();
        virtual OneWireMaster::CmdResult OWReset();
        virtual OneWireMaster::CmdResult OWTouchBitSetLevel(uint8_t & sendRecvBit, OWLevel afterLevel);
        virtual OneWireMaster::CmdResult OWWriteByteSetLevel(uint8_t sendByte, OWLevel afterLevel);
        virtual OneWireMaster::CmdResult OWReadByteSetLevel(uint8_t & recvByte, OWLevel afterLevel);
        virtual OneWireMaster::CmdResult OWSetSpeed(OWSpeed newSpeed);
        virtual OneWireMaster::CmdResult OWSetLevel(OWLevel newLevel);
        
        OneWireMaster::CmdResult detect();
        OneWireMaster::CmdResult changeBaud(BaudRate newBaud);

    private:
        OneWireMaster::CmdResult writeCom(size_t outlen, uint8_t *outbuf);
        OneWireMaster::CmdResult readCom(size_t inlen, uint8_t *inbuf);
        void breakCom();
        void flushCom();
        void setComBaud(BaudRate new_baud);

        // COM interface
        mbed::Serial serial;

        // DS2480B state
        OWLevel level; // 1-Wire level
        BaudRate baud;  // baud rate
        uint8_t mode;  // command or data mode state
        uint8_t speed; // 1-Wire communication speed
    };
}

#endif

