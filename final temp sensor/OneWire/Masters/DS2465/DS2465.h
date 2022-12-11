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

#ifndef OneWire_Masters_DS2465
#define OneWire_Masters_DS2465

#include "Masters/OneWireMaster.h"
#include "Slaves/Authenticators/ISha256MacCoproc.h"

namespace mbed { class I2C; }

namespace OneWire
{
    /// Interface to the DS2465 1-Wire master and SHA-256 coprocessor.
    class DS2465 : public OneWireMaster, public ISha256MacCoproc
    {
    public:
        /// Delay required after writing an EEPROM segment.
        static const unsigned int eepromSegmentWriteDelayMs = 10;
        /// Delay required after writing an EEPROM page such as the secret memory.
        static const unsigned int eepromPageWriteDelayMs = 8 * eepromSegmentWriteDelayMs;
        /// Delay required for a SHA computation to complete.
        static const unsigned int shaComputationDelayMs = 2;

        /// Page region to use for swapping.
        enum PageRegion
        {
            FullPage = 0x03,
            FirstHalf = 0x01,
            SecondHalf = 0x02
        };

        /// Starting memory addresses.
        enum MemoryAddress
        {
            Scratchpad = 0x00,
            CommandReg = 0x60,
            StatusReg = 0x61,
            ReadDataReg = 0x62,
            MacReadoutReg = 0x63,
            MemoryProtectionReg = 0x64,
            ConfigReg = 0x67,
            tRSTL_Reg = 0x68,
            tMSP_Reg = 0x69,
            tW0L_Reg = 0x6A,
            tREC0_Reg = 0x6B,
            RWPU_Reg = 0x6C,
            tW1L_Reg = 0x6D,
            UserMemoryPage0 = 0x80,
            UserMemoryPage1 = 0xA0
        };

        /// Represents a DS2465 configuration.
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

            /// Byte representation that is read from the DS2465.
            uint8_t readByte() const;
            /// Byte respresentation that is written to the DS2465.
            uint8_t writeByte() const;

            /// Reset to the power-on default config.
            void reset();
            Config() { reset(); }

        private:
            bool m_1WS, m_SPU, m_PDN, m_APU;
        };

        /// @param I2C_interface Configured I2C communication interface for DS2465.
        /// @param I2C_address I2C bus address of the DS2465 in mbed format.
        DS2465(mbed::I2C & I2C_interface, uint8_t I2C_address = 0x30);

        // Const member functions should not change the settings of the DS2465 or affect the state of the 1-Wire bus.
        // Read pointer, scratchpad, MAC output register, and command register on the DS2465 are considered mutable.

        /// Performs a soft reset on the DS2465.
        /// @note This is not a 1-Wire Reset.
        OneWireMaster::CmdResult reset();

        /// Write a new configuration to the DS2465.
        /// @param[in] config New configuration to write.
        /// @param verify Verify that the configuration was written successfully.
        OneWireMaster::CmdResult writeConfig(const Config & config, bool verify);

        /// Read the current DS2465 configuration.
        /// @returns The cached current configuration.
        Config currentConfig() const { return m_curConfig; }

        // DS2465 Memory Commands

        /// Read memory from the DS2465.
        /// @param addr Address to begin reading from.
        /// @param[out] buf Buffer to hold read data.
        /// @param bufLen Length of buffer, buf, and number of bytes to read.
        /// @param skipSetPointer Assume that the read pointer is already set to the correct address.
        OneWireMaster::CmdResult readMemory(uint8_t addr, uint8_t * buf, size_t bufLen, bool skipSetPointer = false) const;

        /// Write to SRAM memory on the DS2465.
        /// @param addr Address to begin writing to.
        /// @param[in] buf Buffer containing the data to write.
        /// @param bufLen Length of buffer, buf, and number of bytes to write.
        OneWireMaster::CmdResult writeMemory(uint8_t addr, const uint8_t * buf, size_t bufLen) { return cWriteMemory(addr, buf, bufLen); }

        /// Write data to the scratchpad area of the DS2465.
        /// @param[in] buf Buffer containing the data to write.
        /// @param bufLen Length of buffer, buf, and the number of bytes to write.
        OneWireMaster::CmdResult writeScratchpad(const uint8_t * buf, size_t bufLen) const { return cWriteMemory(Scratchpad, buf, bufLen); }

        /// Copy the scratchpad contents to an EEPROM memory page.
        /// @param pageNum Page number to copy to.
        OneWireMaster::CmdResult copyScratchpadToPage(unsigned int pageNum) { return copyScratchpad(false, pageNum, false, 0); }

        /// Copy the scratchpad contents to an EEPROM memory segment.
        /// @param pageNum Page number to copy to.
        /// @param segmentNum Segment number to copy to.
        OneWireMaster::CmdResult copyScratchpadToSegment(unsigned int pageNum, unsigned int segmentNum) { return copyScratchpad(false, pageNum, true, segmentNum); }

        /// Copy the scratchpad contents to the secret EEPROM memory page.
        OneWireMaster::CmdResult copyScratchpadToSecret() { return copyScratchpad(true, 0, false, 0); }

        // 1-Wire Master Commands
        virtual OneWireMaster::CmdResult OWInitMaster();
        virtual OneWireMaster::CmdResult OWReset();
        virtual OneWireMaster::CmdResult OWTouchBitSetLevel(uint8_t & sendRecvBit, OWLevel afterLevel);
        virtual OneWireMaster::CmdResult OWReadByteSetLevel(uint8_t & recvByte, OWLevel afterLevel);
        virtual OneWireMaster::CmdResult OWWriteByteSetLevel(uint8_t sendByte, OWLevel afterLevel);
        virtual OneWireMaster::CmdResult OWReadBlock(uint8_t *recvBuf, size_t recvLen);
        virtual OneWireMaster::CmdResult OWWriteBlock(const uint8_t *sendBuf, size_t sendLen);
        virtual OneWireMaster::CmdResult OWSetSpeed(OWSpeed newSpeed);
        /// @note The DS2465 only supports enabling strong pullup following a 1-Wire read or write operation.
        virtual OneWireMaster::CmdResult OWSetLevel(OWLevel newLevel);
        virtual OneWireMaster::CmdResult OWTriplet(SearchDirection & searchDirection, uint8_t & sbr, uint8_t & tsb);

        /// Write the last computed MAC to the 1-Wire bus
        OneWireMaster::CmdResult OWWriteBlockMac();

        // DS2465 Coprocessor Commands

        /// Compute Next Master Secret with scratchpad data.
        OneWireMaster::CmdResult computeNextMasterSecret() { return computeNextMasterSecret(false, 0, FullPage); }

        /// Compute Next Master Secret with page swapping.
        /// @param pageNum Page number to swap in.
        /// @param region Region of the page to swap in.
        OneWireMaster::CmdResult computeNextMasterSecretSwap(unsigned int pageNum, PageRegion region) { return computeNextMasterSecret(true, pageNum, region); }

        /// Compute Write MAC with scratchpad data.
        /// @param regwrite True if writing to a register or false if regular memory.
        OneWireMaster::CmdResult computeWriteMac(bool regwrite) const { return computeWriteMac(regwrite, false, 0, 0); }

        /// Compute Write MAC with page swapping.
        /// @param regwrite True if writing to a register or false if regular memory.
        /// @param pageNum Page number to swap in.
        /// @param segmentNum Segment number to swap in.
        OneWireMaster::CmdResult computeWriteMacSwap(bool regwrite, unsigned int pageNum, unsigned int segmentNum) const { return computeWriteMac(regwrite, true, pageNum, segmentNum); }

        /// Compute Slave Secret (S-Secret) with scratchpad data.
        OneWireMaster::CmdResult computeSlaveSecret() { return computeSlaveSecret(false, 0, FullPage); }

        /// Compute Slave Secret (S-Secret) with page swapping.
        /// @param pageNum Page number to swap in.
        /// @param region Region of the page to swap in.
        OneWireMaster::CmdResult computeSlaveSecretSwap(unsigned int pageNum, PageRegion region) { return computeSlaveSecret(true, pageNum, region); }

        /// Compute Authentication MAC with scratchpad data.
        OneWireMaster::CmdResult computeAuthMac() const { return computeAuthMac(false, 0, FullPage); }

        /// Compute Authentication MAC with page swapping.
        /// @param pageNum Page number to swap in.
        /// @param region Region of the page to swap in.
        OneWireMaster::CmdResult computeAuthMacSwap(unsigned int pageNum, PageRegion region) const { return computeAuthMac(true, pageNum, region); }

        // ISha256MacCoproc Commands
        virtual ISha256MacCoproc::CmdResult setMasterSecret(const Secret & masterSecret);
        virtual ISha256MacCoproc::CmdResult computeSlaveSecret(const DevicePage & devicePage, const DeviceScratchpad & deviceScratchpad, const SlaveSecretData & slaveSecretData);
        virtual ISha256MacCoproc::CmdResult computeWriteMac(const WriteMacData & writeMacData, Mac & mac) const;
        virtual ISha256MacCoproc::CmdResult computeAuthMac(const DevicePage & devicePage, const DeviceScratchpad & challenge, const AuthMacData & authMacData, Mac & mac) const;

    private:
        mbed::I2C & m_I2C_interface;
        uint8_t m_I2C_address;
        Config m_curConfig;

        /// Polls the DS2465 status waiting for the 1-Wire Busy bit (1WB) to be cleared.
        /// @param[out] pStatus Optionally retrive the status byte when 1WB cleared.
        /// @returns Success or TimeoutError if poll limit reached.
        OneWireMaster::CmdResult pollBusy(uint8_t * pStatus = NULL);

        /// Ensure that the desired 1-Wire level is set in the configuration.
        /// @param level Desired 1-Wire level.
        OneWireMaster::CmdResult configureLevel(OWLevel level);

        /// Const version of writeMemory() for internal use.
        OneWireMaster::CmdResult cWriteMemory(uint8_t addr, const uint8_t * buf, size_t bufLen) const;

        // Legacy implementations
        OneWireMaster::CmdResult copyScratchpad(bool destSecret, unsigned int pageNum, bool notFull, unsigned int segmentNum);
        OneWireMaster::CmdResult computeNextMasterSecret(bool swap, unsigned int pageNum, PageRegion region);
        OneWireMaster::CmdResult computeWriteMac(bool regwrite, bool swap, unsigned int pageNum, unsigned int segmentNum) const;
        OneWireMaster::CmdResult computeSlaveSecret(bool swap, unsigned int pageNum, PageRegion region);
        OneWireMaster::CmdResult computeAuthMac(bool swap, unsigned int pageNum, PageRegion region) const;
    };
}

#endif
