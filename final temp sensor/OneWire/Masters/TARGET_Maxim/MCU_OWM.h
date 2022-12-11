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

#ifndef OneWire_Masters_OWM
#define OneWire_Masters_OWM

#if defined(TARGET_MAX32620) || defined(TARGET_MAX32625) || defined(TARGET_MAX32630)

#include <owm.h>
#include "Masters/OneWireMaster.h"

namespace OneWire
{
    /// MCU peripheral 1-Wire master (OWM)
    class MCU_OWM : public OneWireMaster
    {
    public:
        MCU_OWM(bool extWeakPup = false, bool extStrongPup = false);

        virtual OneWireMaster::CmdResult OWInitMaster();
        virtual OneWireMaster::CmdResult OWReset();
        virtual OneWireMaster::CmdResult OWTouchBitSetLevel(uint8_t & sendRecvBit, OWLevel afterLevel);
        virtual OneWireMaster::CmdResult OWWriteByteSetLevel(uint8_t sendByte, OWLevel afterLevel);
        virtual OneWireMaster::CmdResult OWReadByteSetLevel(uint8_t & recvByte, OWLevel afterLevel);
        virtual OneWireMaster::CmdResult OWSetSpeed(OWSpeed newSpeed);
        virtual OneWireMaster::CmdResult OWSetLevel(OWLevel newLevel);

    private:
        bool extStrongPup() const;
    
        owm_cfg_t m_owmCfg;
        sys_cfg_owm_t m_sysCfgOwm;
    };
}

#endif /* TARGET_MAX326xx */

#endif /* OneWire_Masters_OWM */

