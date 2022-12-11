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

#ifndef OneWire_Masters_DS248x
#define OneWire_Masters_DS248x

#include "Masters/OneWireMaster.h"

namespace mbed { class I2C; }

namespace OneWire
{
    /// Interface to the DS2484, DS2482-100, DS2482-101, DS2482-800 1-Wire masters.
    class DS248x : public OneWireMaster
    {
    public:
        /// Device register pointers.
        enum Register
        {
            ConfigReg = 0xC3,
            StatusReg = 0xF0,
            ReadDataReg = 0xE1,
            PortConfigReg = 0xB4,
            ChannelSelectReg = 0xD2 // DS2482-800 only
        };

        /// Represents a DS248x configuration.
        class Config
        {
        public:
            /// @{
            /// 1-Wire Speed
            bool get1WS() const { return m_1WS; }
            void set1WS(bool new1WS) { m_1WS = new1WS; }
            /// @}

            /// @{
            /// Strong Pullup
            bool getSPU() const { return m_SPU; }
            void setSPU(bool newSPU) { m_SPU = newSPU; }
            /// @}

            /// @{
            /// 1-Wire Power Down
            bool getPDN() const { return m_PDN; }
            void setPDN(bool newPDN) { m_PDN = newPDN; }
            /// @}

            /// @{
            /// Active Pullup
            bool getAPU() const { return m_APU; }
            void setAPU(bool newAPU) { m_APU = newAPU; }
            /// @}

            /// Byte representation that is read from the DS248x.
            uint8_t readByte() const;
            /// Byte respresentation that is written to the DS248x.
            uint8_t writeByte() const;

            /// Reset to the power-on default config.
            void reset();
            Config() { reset(); }

        private:
            bool m_1WS, m_SPU, m_PDN, m_APU;
        };
        
        /// Performs a soft reset on the DS248x.
        /// @note This is note a 1-Wire Reset.
        OneWireMaster::CmdResult reset(void);

        /// Write a new configuration to the DS248x.
        /// @param[in] config New configuration to write.
        /// @param verify Verify that the configuration was written successfully.
        OneWireMaster::CmdResult writeConfig(const Config & config, bool verify);

        /// Read the current DS248x configuration.
        /// @returns The cached current configuration.
        Config currentConfig() const { return m_curConfig; }

        /// Reads a register from the DS248x.
        /// @param reg Register to read from.
        /// @param[out] buf Buffer to hold read data.
        /// @param skipSetPointer Assume that the read pointer is already set to the correct register.
        OneWireMaster::CmdResult readRegister(Register reg, uint8_t & buf, bool skipSetPointer = false) const;

        /// @details Performs a device reset followed by writing the configuration byte to default values:
        ///          1-Wire Speed Standard
        ///          Strong Pullup Off
        ///          1-Wire Powerdown Off
        ///          Active Pullup On
        virtual OneWireMaster::CmdResult OWInitMaster();
        
        /// @note Perform a 1-Wire triplet using the DS248x command.
        virtual OneWireMaster::CmdResult OWTriplet(SearchDirection & searchDirection, uint8_t & sbr, uint8_t & tsb);
        
        virtual OneWireMaster::CmdResult OWReset();
        virtual OneWireMaster::CmdResult OWTouchBitSetLevel(uint8_t & sendRecvBit, OWLevel afterLevel);
        virtual OneWireMaster::CmdResult OWReadByteSetLevel(uint8_t & recvByte, OWLevel afterLevel);
        virtual OneWireMaster::CmdResult OWWriteByteSetLevel(uint8_t sendByte, OWLevel afterLevel);
        virtual OneWireMaster::CmdResult OWSetSpeed(OWSpeed newSpeed);
        virtual OneWireMaster::CmdResult OWSetLevel(OWLevel newLevel);

    protected:
        enum Command
        {
            DeviceResetCmd = 0xF0,
            WriteDeviceConfigCmd = 0xD2,
            AdjustOwPortCmd = 0xC3, // DS2484 only
            ChannelSelectCmd = 0xC3, // DS2482-800 only
            SetReadPointerCmd = 0xE1,
            OwResetCmd = 0xB4,
            OwWriteByteCmd = 0xA5,
            OwReadByteCmd = 0x96,
            OwSingleBitCmd = 0x87,
            OwTripletCmd = 0x78
        };
        
        /// Construct to use an existing I2C interface.
        /// @param i2c_bus Configured I2C communication interface for DS248x.
        /// @param adrs I2C bus address of the DS248x in mbed format.
        DS248x(mbed::I2C & i2c_bus, uint8_t adrs);
        
        /// @note Allow marking const since not public.
        OneWireMaster::CmdResult sendCommand(Command cmd) const;

        /// @note Allow marking const since not public.
        OneWireMaster::CmdResult sendCommand(Command cmd, uint8_t param) const;
    
    private:
        /// Polls the DS248x status waiting for the 1-Wire Busy bit (1WB) to be cleared.
        /// @param[out] pStatus Optionally retrieve the status byte when 1WB cleared.
        /// @returns Success or TimeoutError if poll limit reached.
        OneWireMaster::CmdResult pollBusy(uint8_t * pStatus = NULL);

        /// Ensure that the desired 1-Wire level is set in the configuration.
        /// @param level Desired 1-Wire level.
        OneWireMaster::CmdResult configureLevel(OWLevel level);
        
        mbed::I2C & m_i2c_bus;
        uint8_t m_adrs;
        Config m_curConfig;
    };
}

#endif
