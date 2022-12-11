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

#include "Masters/DS2465/DS2465.h"
#include "I2C.h"
#include "wait_api.h"

using namespace OneWire;

#define I2C_WRITE 0
#define I2C_READ 1

/// DS2465 Commands
enum Command
{
    DeviceResetCmd = 0xF0,
    WriteDeviceConfigCmd = 0xD2,
    OwResetCmd = 0xB4,
    OwWriteByteCmd = 0xA5,
    OwReadByteCmd = 0x96,
    OwSingleBitCmd = 0x87,
    OwTripletCmd = 0x78,
    OwTransmitBlockCmd = 0x69,
    OwReceiveBlockCmd = 0xE1,
    CopyScratchpadCmd = 0x5A,
    ComputeSlaveSecretCmd = 0x4B,
    ComputeSlaveAuthMacCmd = 0x3C,
    ComputeSlaveWriteMacCmd = 0x2D,
    ComputeNextMasterSecretCmd = 0x1E,
    SetProtectionCmd = 0x0F
};

/// DS2465 Status Bits
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

static const int I2C_WRITE_OK = 1;
static const uint8_t maxBlockSize = 63;

uint8_t DS2465::Config::readByte() const
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

uint8_t DS2465::Config::writeByte() const
{
    uint8_t config = readByte();
    return ((~config << 4) | config);
}

void DS2465::Config::reset()
{
    set1WS(false);
    setSPU(false);
    setPDN(false);
    setAPU(true);
}

DS2465::DS2465(mbed::I2C & I2C_interface, uint8_t I2C_address)
    : m_I2C_interface(I2C_interface), m_I2C_address(I2C_address)
{

}

OneWireMaster::CmdResult DS2465::OWInitMaster()
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

OneWireMaster::CmdResult DS2465::computeNextMasterSecret(bool swap, unsigned int pageNum, PageRegion region)
{
    uint8_t command[2] = { ComputeNextMasterSecretCmd, (uint8_t)(swap ? (0xC8 | (pageNum << 4) | region) : 0xBF) };
    return writeMemory(CommandReg, command, 2);
}

OneWireMaster::CmdResult DS2465::computeWriteMac(bool regwrite, bool swap, unsigned int pageNum, unsigned int segmentNum) const
{
    uint8_t command[2] = { ComputeSlaveWriteMacCmd, (uint8_t)((regwrite << 7) | (swap << 6) | (pageNum << 4) | segmentNum) };
    return cWriteMemory(CommandReg, command, 2);
}

OneWireMaster::CmdResult DS2465::computeAuthMac(bool swap, unsigned int pageNum, PageRegion region) const
{
    uint8_t command[2] = { ComputeSlaveAuthMacCmd, (uint8_t)(swap ? (0xC8 | (pageNum << 4) | region) : 0xBF) };
    return cWriteMemory(CommandReg, command, 2);
}

OneWireMaster::CmdResult DS2465::computeSlaveSecret(bool swap, unsigned int pageNum, PageRegion region)
{
    uint8_t command[2] = { ComputeSlaveSecretCmd, (uint8_t)(swap ? (0xC8 | (pageNum << 4) | region) : 0xBF) };
    return writeMemory(CommandReg, command, 2);
}

ISha256MacCoproc::CmdResult DS2465::setMasterSecret(const Secret & masterSecret)
{
    OneWireMaster::CmdResult result;
    result = writeMemory(Scratchpad, masterSecret.data(), masterSecret.size());
    if (result == OneWireMaster::Success)
    {
        result = copyScratchpadToSecret();
    }
    if (result == OneWireMaster::Success)
    {
        wait_ms(eepromPageWriteDelayMs);
    }
    return (result == OneWireMaster::Success ? ISha256MacCoproc::Success : ISha256MacCoproc::OperationFailure);
}

ISha256MacCoproc::CmdResult DS2465::computeWriteMac(const WriteMacData & writeMacData, Mac & mac) const
{
    OneWireMaster::CmdResult result;
    // Write input data to scratchpad
    result = writeScratchpad(writeMacData.data(), writeMacData.size());
    // Compute MAC
    if (result == OneWireMaster::Success)
    {
        result = computeWriteMac(false);
    }
    if (result == OneWireMaster::Success)
    {
        wait_ms(shaComputationDelayMs);
        // Read MAC from register
        result = readMemory(MacReadoutReg, mac.data(), mac.size(), true);
    }
    return (result == OneWireMaster::Success ? ISha256MacCoproc::Success : ISha256MacCoproc::OperationFailure);
}

ISha256MacCoproc::CmdResult DS2465::computeAuthMac(const DevicePage & devicePage, const DeviceScratchpad & challenge, const AuthMacData & authMacData, Mac & mac) const
{
    OneWireMaster::CmdResult result;
    int addr = Scratchpad;
    // Write input data to scratchpad
    result = cWriteMemory(addr, devicePage.data(), devicePage.size());
    if (result == OneWireMaster::Success)
    {
        addr += devicePage.size();
        result = cWriteMemory(addr, challenge.data(), challenge.size());
    }
    if (result == OneWireMaster::Success)
    {
        addr += challenge.size();
        result = cWriteMemory(addr, authMacData.data(), authMacData.size());
    }
    // Compute MAC
    if (result == OneWireMaster::Success)
    {
        result = computeAuthMac();
    }
    if (result == OneWireMaster::Success)
    {
        wait_ms(shaComputationDelayMs * 2);
        // Read MAC from register
        result = readMemory(MacReadoutReg, mac.data(), mac.size(), true);
    }
    return (result == OneWireMaster::Success ? ISha256MacCoproc::Success : ISha256MacCoproc::OperationFailure);
}

ISha256MacCoproc::CmdResult DS2465::computeSlaveSecret(const DevicePage & devicePage, const DeviceScratchpad & deviceScratchpad, const SlaveSecretData & slaveSecretData)
{
    OneWireMaster::CmdResult result;
    int addr = Scratchpad;
    // Write input data to scratchpad
    result = writeMemory(addr, devicePage.data(), devicePage.size());
    if (result == OneWireMaster::Success)
    {
        addr += devicePage.size();
        result = writeMemory(addr, deviceScratchpad.data(), deviceScratchpad.size());
    }
    if (result == OneWireMaster::Success)
    {
        addr += deviceScratchpad.size();
        result = writeMemory(addr, slaveSecretData.data(), slaveSecretData.size());
    }
    // Compute secret
    if (result == OneWireMaster::Success)
    {
        result = computeSlaveSecret();
    }
    if (result == OneWireMaster::Success)
    {
        wait_ms(shaComputationDelayMs * 2);
    }
    return (result == OneWireMaster::Success ? ISha256MacCoproc::Success : ISha256MacCoproc::OperationFailure);
}

OneWireMaster::CmdResult DS2465::copyScratchpad(bool destSecret, unsigned int pageNum, bool notFull, unsigned int segmentNum)
{
    uint8_t command[2] = { CopyScratchpadCmd, (uint8_t)(destSecret ? 0 : (0x80 | (pageNum << 4) | (notFull << 3) | segmentNum)) };
    return writeMemory(CommandReg, command, 2);
}

OneWireMaster::CmdResult DS2465::configureLevel(OWLevel level)
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

OneWireMaster::CmdResult DS2465::OWSetLevel(OWLevel newLevel)
{
    if (newLevel == StrongLevel)
    {
        return OneWireMaster::OperationFailure;
    }

    return configureLevel(newLevel);
}

OneWireMaster::CmdResult DS2465::OWSetSpeed(OWSpeed newSpeed)
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

OneWireMaster::CmdResult DS2465::OWTriplet(SearchDirection & searchDirection, uint8_t & sbr, uint8_t & tsb)
{
    // 1-Wire Triplet (Case B)
    //   S AD,0 [A] 1WT [A] SS [A] Sr AD,1 [A] [Status] A [Status] A\ P
    //                                         \--------/        
    //                           Repeat until 1WB bit has changed to 0
    //  [] indicates from slave
    //  SS indicates byte containing search direction bit value in msbit

    OneWireMaster::CmdResult result;
    uint8_t command[2] = { OwTripletCmd, (uint8_t)((searchDirection == WriteOne) ? 0x80 : 0x00) };
    result = writeMemory(CommandReg, command, 2);
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

OneWireMaster::CmdResult DS2465::OWReadBlock(uint8_t *recvBuf, size_t recvLen)
{
    // 1-Wire Receive Block (Case A)
    //   S AD,0 [A] CommandReg [A] 1WRF [A] PR [A] P
    //  [] indicates from slave
    //  PR indicates byte containing parameter

    OneWireMaster::CmdResult result = OneWireMaster::Success;
    for (size_t i = 0; (i < recvLen) && (result == OneWireMaster::Success); i += maxBlockSize)
    {
        uint8_t command[2] = { OwReceiveBlockCmd, std::min(static_cast<uint8_t>(recvLen - i), maxBlockSize) };
        result = writeMemory(CommandReg, command, 2);
        if (result == OneWireMaster::Success)
        {
            result = pollBusy();
        }
        if (result == OneWireMaster::Success)
        {
            result = readMemory(Scratchpad, recvBuf + i, command[1], false);
        }
    }
    return result;
}

OneWireMaster::CmdResult DS2465::OWWriteBlock(const uint8_t *sendBuf, size_t sendLen)
{
    OneWireMaster::CmdResult result = OneWireMaster::Success;
    for (size_t i = 0; (i < sendLen) && (result == OneWireMaster::Success); i += maxBlockSize)
    {
        uint8_t command[2] = { OwTransmitBlockCmd, std::min(static_cast<uint8_t>(sendLen - i), maxBlockSize) };

        // prefill scratchpad with required data
        result = writeMemory(Scratchpad, sendBuf + i, command[1]);

        // 1-Wire Transmit Block (Case A)
        //   S AD,0 [A] CommandReg [A] 1WTB [A] PR [A] P
        //  [] indicates from slave
        //  PR indicates byte containing parameter
        if (result == OneWireMaster::Success)
        {
            result = writeMemory(CommandReg, command, 2);
        }
        if (result == OneWireMaster::Success)
        {
            result = pollBusy();
        }
    }
    return result;
}

OneWireMaster::CmdResult DS2465::OWWriteBlockMac()
{
    // 1-Wire Transmit Block (Case A)
    //   S AD,0 [A] CommandReg [A] 1WTB [A] PR [A] P
    //  [] indicates from slave
    //  PR indicates byte containing parameter

    uint8_t command[2] = { OwTransmitBlockCmd, 0xFF };
    OneWireMaster::CmdResult result = writeMemory(CommandReg, command, 2);
    if (result == OneWireMaster::Success)
    {
        result = pollBusy();
    }
    return result;
}

OneWireMaster::CmdResult DS2465::OWReadByteSetLevel(uint8_t & recvByte, OWLevel afterLevel)
{
    // 1-Wire Read Bytes (Case C)
    //   S AD,0 [A] CommandReg [A] 1WRB [A] Sr AD,1 [A] [Status] A [Status] A 
    //                                                  \--------/        
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

    buf = OwReadByteCmd;
    result = writeMemory(CommandReg, &buf, 1);

    if (result == OneWireMaster::Success)
    {
        result = pollBusy();
    }

    if (result == OneWireMaster::Success)
    {
        result = readMemory(ReadDataReg, &buf, 1);
    }

    if (result == OneWireMaster::Success)
    {
        recvByte = buf;
    }

    return result;
}

OneWireMaster::CmdResult DS2465::OWWriteByteSetLevel(uint8_t sendByte, OWLevel afterLevel)
{
    // 1-Wire Write Byte (Case B)
    //   S AD,0 [A] CommandReg [A] 1WWB [A] DD [A] Sr AD,1 [A] [Status] A [Status] A\ P
    //                                                           \--------/        
    //                             Repeat until 1WB bit has changed to 0
    //  [] indicates from slave
    //  DD data to write

    OneWireMaster::CmdResult result;

    result = configureLevel(afterLevel);
    if (result != OneWireMaster::Success)
    {
        return result;
    }

    uint8_t command[2] = { OwWriteByteCmd, sendByte };

    result = writeMemory(CommandReg, command, 2);
    if (result == OneWireMaster::Success)
    {
        result = pollBusy();
    }

    return result;
}

OneWireMaster::CmdResult DS2465::OWTouchBitSetLevel(uint8_t & sendRecvBit, OWLevel afterLevel)
{
    // 1-Wire bit (Case B)
    //   S AD,0 [A] CommandReg [A] 1WSB [A] BB [A] Sr AD,1 [A] [Status] A [Status] A\ P
    //                                                          \--------/        
    //                           Repeat until 1WB bit has changed to 0
    //  [] indicates from slave
    //  BB indicates byte containing bit value in msbit

    OneWireMaster::CmdResult result;

    result = configureLevel(afterLevel);
    if (result != OneWireMaster::Success)
    {
        return result;
    }

    uint8_t command[2] = { OwSingleBitCmd, (uint8_t)(sendRecvBit ? 0x80 : 0x00) };
    uint8_t status;

    result = writeMemory(CommandReg, command, 2);

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

OneWireMaster::CmdResult DS2465::cWriteMemory(uint8_t addr, const uint8_t * buf, size_t bufLen) const
{
    // Write SRAM (Case A)
    //   S AD,0 [A] VSA [A] DD [A]  P
    //                      \-----/
    //                        Repeat for each data byte
    //  [] indicates from slave
    //  VSA valid SRAM memory address
    //  DD memory data to write

    m_I2C_interface.start();
    if (m_I2C_interface.write((m_I2C_address | I2C_WRITE)) != I2C_WRITE_OK)
    {
        m_I2C_interface.stop();
        return OneWireMaster::CommunicationWriteError;
    }
    if (m_I2C_interface.write(addr) != I2C_WRITE_OK)
    {
        m_I2C_interface.stop();
        return OneWireMaster::CommunicationWriteError;
    }
    // loop to write each byte
    for (size_t i = 0; i < bufLen; i++)
    {
        if (m_I2C_interface.write(buf[i]) != I2C_WRITE_OK)
        {
            m_I2C_interface.stop();
            return OneWireMaster::CommunicationWriteError;
        }
    }
    m_I2C_interface.stop();

    return OneWireMaster::Success;
}

OneWireMaster::CmdResult DS2465::readMemory(uint8_t addr, uint8_t * buf, size_t bufLen, bool skipSetPointer) const
{
    // Read (Case A)
    //   S AD,0 [A] MA [A] Sr AD,1 [A] [DD] A [DD] A\ P
    //                                 \-----/
    //                                   Repeat for each data byte, NAK last byte
    //  [] indicates from slave
    //  MA memory address
    //  DD memory data read

    m_I2C_interface.start();
    if (!skipSetPointer)
    {
        if (m_I2C_interface.write((m_I2C_address | I2C_WRITE)) != I2C_WRITE_OK)
        {
            m_I2C_interface.stop();
            return OneWireMaster::CommunicationWriteError;
        }
        if (m_I2C_interface.write(addr) != I2C_WRITE_OK)
        {
            m_I2C_interface.stop();
            return OneWireMaster::CommunicationWriteError;
        }
        m_I2C_interface.start();
    }

    if (m_I2C_interface.write((m_I2C_address | I2C_READ)) != I2C_WRITE_OK)
    {
        m_I2C_interface.stop();
        return OneWireMaster::CommunicationWriteError;
    }
    // loop to read each byte, NAK last byte
    for (size_t i = 0; i < bufLen; i++)
    {
        buf[i] = m_I2C_interface.read((i == (bufLen - 1)) ? mbed::I2C::NoACK : mbed::I2C::ACK);
    }
    m_I2C_interface.stop();

    return OneWireMaster::Success;
}

OneWireMaster::CmdResult DS2465::writeConfig(const Config & config, bool verify)
{
    uint8_t configBuf;
    OneWireMaster::CmdResult result;

    configBuf = config.writeByte();
    result = writeMemory(ConfigReg, &configBuf, 1);
    if (verify)
    {
        if (result == OneWireMaster::Success)
        {
            result = readMemory(ConfigReg, &configBuf, 1);
        }
        if (result == OneWireMaster::Success)
        {
            if (configBuf != config.readByte())
                result = OneWireMaster::OperationFailure;
        }
    }

    if (result == OneWireMaster::Success)
    {
        m_curConfig = config;
    }

    return result;
}

OneWireMaster::CmdResult DS2465::pollBusy(uint8_t * pStatus)
{
    const unsigned int pollLimit = 200;

    OneWireMaster::CmdResult result;
    uint8_t status;
    unsigned int pollCount = 0;

    do
    {
        result = readMemory(StatusReg, &status, 1, true);
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

OneWireMaster::CmdResult DS2465::OWReset()
{
    // 1-Wire reset (Case B)
    //   S AD,0 [A] CommandReg  [A] 1WRS [A] Sr AD,1 [A] [Status] A [Status] A\ P
    //                                                  \--------/        
    //                       Repeat until 1WB bit has changed to 0
    //  [] indicates from slave

    OneWireMaster::CmdResult result;
    uint8_t buf;

    buf = OwResetCmd;
    result = writeMemory(CommandReg, &buf, 1);

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

OneWireMaster::CmdResult DS2465::reset()
{
    // Device Reset
    //   S AD,0 [A] CommandReg [A] 1WMR [A] Sr AD,1 [A] [SS] A\ P
    //  [] indicates from slave
    //  SS status byte to read to verify state

    OneWireMaster::CmdResult result;
    uint8_t buf;

    buf = DeviceResetCmd;
    result = writeMemory(CommandReg, &buf, 1);

    if (result == OneWireMaster::Success)
    {
        result = readMemory(StatusReg, &buf, 1, true);
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
