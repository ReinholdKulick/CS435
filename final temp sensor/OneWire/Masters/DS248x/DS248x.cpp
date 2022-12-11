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

#include "Masters/DS248x/DS248x.h"
#include "I2C.h"

using OneWire::OneWireMaster;
using OneWire::DS248x;

/// DS248x Status Bits
enum StatusBit
{
    Status_1WB = 0x01,
    Status_PPD = 0x02,
    Status_SD = 0x04,
    Status_LL = 0x08,
    Status_RST = 0x10,
    Status_SBR = 0x20,
    Status_TSB = 0x40,
    Status_DIR = 0x80
};

static const int I2C_WRITE_OK = 0;
static const int I2C_READ_OK = 0;

uint8_t DS248x::Config::readByte() const
{
    uint8_t config = 0;
    if (get1WS())
    {
        config |= 0x08;
    }
    if (getSPU())
    {
        config |= 0x04;
    }
    if (getPDN())
    {
        config |= 0x02;
    }
    if (getAPU())
    {
        config |= 0x01;
    }
    return config;
}

uint8_t DS248x::Config::writeByte() const
{
    uint8_t config = readByte();
    return ((~config << 4) | config);
}

void DS248x::Config::reset()
{
    set1WS(false);
    setSPU(false);
    setPDN(false);
    setAPU(true);
}


DS248x::DS248x(mbed::I2C & i2c_bus, uint8_t adrs):
m_i2c_bus(i2c_bus), m_adrs(adrs)
{
}


OneWireMaster::CmdResult DS248x::OWInitMaster(void)
{
    OneWireMaster::CmdResult result;

    // reset DS2465 
    result = reset();
    if (result != OneWireMaster::Success)
    {
        return result;
    }

    // write the default configuration setup
    Config defaultConfig;
    result = writeConfig(defaultConfig, true);
    return result;
}

OneWireMaster::CmdResult DS248x::reset(void)
{
    // Device Reset
    //   S AD,0 [A] DRST [A] Sr AD,1 [A] [SS] A\ P
    //  [] indicates from slave
    //  SS status byte to read to verify state

    OneWireMaster::CmdResult result;
    uint8_t buf;

    result = sendCommand(DeviceResetCmd);

    if (result == OneWireMaster::Success)
    {
        result = readRegister(StatusReg, buf, true);
    }

    if (result == OneWireMaster::Success)
    {
        if ((buf & 0xF7) != 0x10)
        {
            result = OneWireMaster::OperationFailure;
        }
    }

    if (result == OneWireMaster::Success)
    {
        OWReset(); // do a command to get 1-Wire master reset out of holding state
    }

    return result;
}


OneWireMaster::CmdResult DS248x::OWTriplet(SearchDirection & searchDirection, uint8_t & sbr, uint8_t & tsb)
{
    // 1-Wire Triplet (Case B)
    //   S AD,0 [A] 1WT [A] SS [A] Sr AD,1 [A] [Status] A [Status] A\ P
    //                                         \--------/
    //                           Repeat until 1WB bit has changed to 0
    //  [] indicates from slave
    //  SS indicates byte containing search direction bit value in msbit

    OneWireMaster::CmdResult result;
    result = sendCommand(OwTripletCmd, (uint8_t)((searchDirection == WriteOne) ? 0x80 : 0x00));
    if (result == OneWireMaster::Success)
    {
        uint8_t status;
        result = pollBusy(&status);
        if (result == OneWireMaster::Success)
        {
            // check bit results in status byte
            sbr = ((status & Status_SBR) == Status_SBR);
            tsb = ((status & Status_TSB) == Status_TSB);
            searchDirection = ((status & Status_DIR) == Status_DIR) ? WriteOne : WriteZero;
        }
    }
    return result;
}

OneWireMaster::CmdResult DS248x::OWReset()
{
    // 1-Wire reset (Case B)
    //   S AD,0 [A] 1WRS [A] Sr AD,1 [A] [Status] A [Status] A\ P
    //                                   \--------/
    //                       Repeat until 1WB bit has changed to 0
    //  [] indicates from slave

    OneWireMaster::CmdResult result;
    uint8_t buf;

    result = sendCommand(OwResetCmd);

    if (result == OneWireMaster::Success)
    {
        result = pollBusy(&buf);
    }

    if (result == OneWireMaster::Success)
    {
        // check for presence detect
        if ((buf & Status_PPD) != Status_PPD)
        {
            result = OneWireMaster::OperationFailure;
        }
    }

    return result;
}

OneWireMaster::CmdResult DS248x::OWTouchBitSetLevel(uint8_t & sendRecvBit, OWLevel afterLevel)
{
    // 1-Wire bit (Case B)
    //   S AD,0 [A] 1WSB [A] BB [A] Sr AD,1 [A] [Status] A [Status] A\ P
    //                                          \--------/
    //                           Repeat until 1WB bit has changed to 0
    //  [] indicates from slave
    //  BB indicates byte containing bit value in msbit

    OneWireMaster::CmdResult result;

    result = configureLevel(afterLevel);
    if (result != OneWireMaster::Success)
    {
        return result;
    }

    uint8_t status;

    result = sendCommand(OwSingleBitCmd, (uint8_t)(sendRecvBit ? 0x80 : 0x00));

    if (result == OneWireMaster::Success)
    {
        result = pollBusy(&status);
    }

    if (result == OneWireMaster::Success)
    {
        sendRecvBit = (status & Status_SBR);
    }

    return result;
}

OneWireMaster::CmdResult DS248x::OWWriteByteSetLevel(uint8_t sendByte, OWLevel afterLevel)
{
    // 1-Wire Write Byte (Case B)
    //   S AD,0 [A] 1WWB [A] DD [A] Sr AD,1 [A] [Status] A [Status] A\ P
    //                                          \--------/
    //                             Repeat until 1WB bit has changed to 0
    //  [] indicates from slave
    //  DD data to write

    OneWireMaster::CmdResult result;

    result = configureLevel(afterLevel);
    if (result != OneWireMaster::Success)
    {
        return result;
    }

    result = sendCommand(OwWriteByteCmd, sendByte);
    if (result == OneWireMaster::Success)
    {
        result = pollBusy();
    }

    return result;
}

OneWireMaster::CmdResult DS248x::OWReadByteSetLevel(uint8_t & recvByte, OWLevel afterLevel)
{
    // 1-Wire Read Bytes (Case C)
    //   S AD,0 [A] 1WRB [A] Sr AD,1 [A] [Status] A [Status] A
    //                                   \--------/
    //                     Repeat until 1WB bit has changed to 0
    //   Sr AD,0 [A] SRP [A] E1 [A] Sr AD,1 [A] DD A\ P
    //
    //  [] indicates from slave
    //  DD data read

    OneWireMaster::CmdResult result;
    uint8_t buf;

    result = configureLevel(afterLevel);
    if (result != OneWireMaster::Success)
    {
        return result;
    }

    result = sendCommand(OwReadByteCmd);

    if (result == OneWireMaster::Success)
    {
        result = pollBusy();
    }

    if (result == OneWireMaster::Success)
    {
        result = readRegister(ReadDataReg, buf);
    }

    if (result == OneWireMaster::Success)
    {
        recvByte = buf;
    }

    return result;
}

OneWireMaster::CmdResult DS248x::OWSetSpeed(OWSpeed newSpeed)
{
    // Requested speed is already set
    if (m_curConfig.get1WS() == (newSpeed == OverdriveSpeed))
    {
        return OneWireMaster::Success;
    }

    // set the speed
    Config newConfig = m_curConfig;
    newConfig.set1WS(newSpeed == OverdriveSpeed);

    // write the new config
    return writeConfig(newConfig, true);
}

OneWireMaster::CmdResult DS248x::OWSetLevel(OWLevel newLevel)
{
    if (newLevel == StrongLevel)
    {
        return OneWireMaster::OperationFailure;
    }

    return configureLevel(newLevel);
}

OneWireMaster::CmdResult DS248x::writeConfig(const Config & config, bool verify)
{
    uint8_t configBuf;
    OneWireMaster::CmdResult result;

    configBuf = config.writeByte();
    result = sendCommand(WriteDeviceConfigCmd, configBuf);
    if (verify)
    {
        if (result == OneWireMaster::Success)
        {
            result = readRegister(ConfigReg, configBuf);
        }
        if (result == OneWireMaster::Success)
        {
            if (configBuf != config.readByte())
            {
                result = OneWireMaster::OperationFailure;
            }
        }
    }

    if (result == OneWireMaster::Success)
    {
        m_curConfig = config;
    }

    return result;
}

OneWireMaster::CmdResult DS248x::readRegister(Register reg, uint8_t & buf, bool skipSetPointer) const
{
    CmdResult result = Success;
    if (!skipSetPointer)
    {
        result = sendCommand(SetReadPointerCmd, reg);
    }
    if (result == Success)
    {
        if (m_i2c_bus.read(m_adrs, reinterpret_cast<char *>(&buf), 1) != I2C_READ_OK)
        {
            result = CommunicationReadError;
        }
    }
    return result;
}

OneWireMaster::CmdResult DS248x::pollBusy(uint8_t * pStatus)
{
    const unsigned int pollLimit = 200;

    OneWireMaster::CmdResult result;
    uint8_t status;
    unsigned int pollCount = 0;

    do
    {
        result = readRegister(StatusReg, status, true);
        if (result != OneWireMaster::Success)
        {
            return result;
        }
        if (pStatus != NULL)
        {
            *pStatus = status;
        }
        if (pollCount++ >= pollLimit)
        {
            return OneWireMaster::TimeoutError;
        }
    } while (status & Status_1WB);

    return OneWireMaster::Success;
}

OneWireMaster::CmdResult DS248x::configureLevel(OWLevel level)
{
    OneWireMaster::CmdResult result;
    if (m_curConfig.getSPU() != (level == StrongLevel))
    {
        Config newConfig = m_curConfig;
        newConfig.setSPU(level == StrongLevel);
        result = writeConfig(newConfig, true);
    }
    else
    {
        result = OneWireMaster::Success;
    }
    return result;
}

OneWireMaster::CmdResult DS248x::sendCommand(Command cmd) const
{
    CmdResult result;
    if (m_i2c_bus.write(m_adrs, reinterpret_cast<const char *>(&cmd), 1) == I2C_WRITE_OK)
    {
        result = Success;
    }
    else
    {
        result = CommunicationWriteError;
    }
    return result;
}

OneWireMaster::CmdResult DS248x::sendCommand(Command cmd, uint8_t param) const
{
    CmdResult result;
    uint8_t buf[2] = { cmd, param };
    if (m_i2c_bus.write(m_adrs, reinterpret_cast<const char *>(buf), 2) == I2C_WRITE_OK)
    {
        result = Success;
    }
    else
    {
        result = CommunicationWriteError;
    }
    return result;
}
