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

#include "Masters/DS2480B/DS2480B.h"
#include "Serial.h"
#include "Timer.h"
#include "wait_api.h"

// Mode Commands
#define MODE_DATA                      0xE1
#define MODE_COMMAND                   0xE3
#define MODE_STOP_PULSE                0xF1

// Return byte value
#define RB_CHIPID_MASK                 0x1C
#define RB_RESET_MASK                  0x03
#define RB_1WIRESHORT                  0x00
#define RB_PRESENCE                    0x01
#define RB_ALARMPRESENCE               0x02
#define RB_NOPRESENCE                  0x03

#define RB_BIT_MASK                    0x03
#define RB_BIT_ONE                     0x03
#define RB_BIT_ZERO                    0x00

// Masks for all bit ranges
#define CMD_MASK                       0x80
#define FUNCTSEL_MASK                  0x60
#define BITPOL_MASK                    0x10
#define SPEEDSEL_MASK                  0x0C
#define MODSEL_MASK                    0x02
#define PARMSEL_MASK                   0x70
#define PARMSET_MASK                   0x0E

// Command or config bit
#define CMD_COMM                       0x81
#define CMD_CONFIG                     0x01

// Function select bits
#define FUNCTSEL_BIT                   0x00
#define FUNCTSEL_SEARCHON              0x30
#define FUNCTSEL_SEARCHOFF             0x20
#define FUNCTSEL_RESET                 0x40
#define FUNCTSEL_CHMOD                 0x60

// Bit polarity/Pulse voltage bits
#define BITPOL_ONE                     0x10
#define BITPOL_ZERO                    0x00
#define BITPOL_5V                      0x00
#define BITPOL_12V                     0x10

// One Wire speed bits
#define SPEEDSEL_STD                   0x00
#define SPEEDSEL_FLEX                  0x04
#define SPEEDSEL_OD                    0x08
#define SPEEDSEL_PULSE                 0x0C

// Data/Command mode select bits
#define MODSEL_DATA                    0x00
#define MODSEL_COMMAND                 0x02

// 5V Follow Pulse select bits 
#define PRIME5V_TRUE                   0x02
#define PRIME5V_FALSE                  0x00

// Parameter select bits
#define PARMSEL_PARMREAD               0x00
#define PARMSEL_SLEW                   0x10
#define PARMSEL_12VPULSE               0x20
#define PARMSEL_5VPULSE                0x30
#define PARMSEL_WRITE1LOW              0x40
#define PARMSEL_SAMPLEOFFSET           0x50
#define PARMSEL_ACTIVEPULLUPTIME       0x60
#define PARMSEL_BAUDRATE               0x70

// Pull down slew rate.
#define PARMSET_Slew15Vus              0x00
#define PARMSET_Slew2p2Vus             0x02
#define PARMSET_Slew1p65Vus            0x04
#define PARMSET_Slew1p37Vus            0x06
#define PARMSET_Slew1p1Vus             0x08
#define PARMSET_Slew0p83Vus            0x0A
#define PARMSET_Slew0p7Vus             0x0C
#define PARMSET_Slew0p55Vus            0x0E

// 12V programming pulse time table
#define PARMSET_32us                   0x00
#define PARMSET_64us                   0x02
#define PARMSET_128us                  0x04
#define PARMSET_256us                  0x06
#define PARMSET_512us                  0x08
#define PARMSET_1024us                 0x0A
#define PARMSET_2048us                 0x0C
#define PARMSET_infinite               0x0E

// 5V strong pull up pulse time table
#define PARMSET_16p4ms                 0x00
#define PARMSET_65p5ms                 0x02
#define PARMSET_131ms                  0x04
#define PARMSET_262ms                  0x06
#define PARMSET_524ms                  0x08
#define PARMSET_1p05s                  0x0A
#define PARMSET_dynamic                0x0C
#define PARMSET_infinite               0x0E

// Write 1 low time
#define PARMSET_Write8us               0x00
#define PARMSET_Write9us               0x02
#define PARMSET_Write10us              0x04
#define PARMSET_Write11us              0x06
#define PARMSET_Write12us              0x08
#define PARMSET_Write13us              0x0A
#define PARMSET_Write14us              0x0C
#define PARMSET_Write15us              0x0E

// Data sample offset and Write 0 recovery time
#define PARMSET_SampOff3us             0x00
#define PARMSET_SampOff4us             0x02
#define PARMSET_SampOff5us             0x04
#define PARMSET_SampOff6us             0x06
#define PARMSET_SampOff7us             0x08
#define PARMSET_SampOff8us             0x0A
#define PARMSET_SampOff9us             0x0C
#define PARMSET_SampOff10us            0x0E

// Active pull up on time
#define PARMSET_PullUp0p0us            0x00
#define PARMSET_PullUp0p5us            0x02
#define PARMSET_PullUp1p0us            0x04
#define PARMSET_PullUp1p5us            0x06
#define PARMSET_PullUp2p0us            0x08
#define PARMSET_PullUp2p5us            0x0A
#define PARMSET_PullUp3p0us            0x0C
#define PARMSET_PullUp3p5us            0x0E

// DS2480B program voltage available
#define DS2480BPROG_MASK                0x20

using OneWire::DS2480B;
using OneWire::OneWireMaster;

static uint32_t calculateBitTimeout(DS2480B::BaudRate baud)
{    
    // Calculate bit timeout in microsecond.
    // 10x the time needed to transmit or receive. 
    // Double for 115200 due to timer inaccuracies.
    
    //*100 for 10 bits/byte and ten times the time needed
    
    uint32_t timeout = 1000000 * 100;
    
    switch (baud)
    {
    case DS2480B::Baud115200bps:
        timeout = (timeout * 2) / 115200;
        break;

    case DS2480B::Baud57600bps:
        timeout /= 57600;
        break;

    case DS2480B::Baud19200bps:
        timeout /= 19200;
        break;

    case DS2480B::Baud9600bps:
    default:
        timeout /= 9600;
        break;
    }
    
    return timeout;
}

DS2480B::DS2480B(PinName tx, PinName rx)
    : serial(tx, rx)
{
    
}

OneWireMaster::CmdResult DS2480B::OWInitMaster()
{
    return detect();
}

OneWireMaster::CmdResult DS2480B::OWReset()
{
    OneWireMaster::CmdResult result;

    uint8_t readbuffer[10], sendpacket[10];
    uint8_t sendlen = 0;

    // make sure normal level
    result = OWSetLevel(OneWireMaster::NormalLevel);
    if (result == OneWireMaster::Success)
    {
        // check for correct mode
        if (mode != MODSEL_COMMAND)
        {
            mode = MODSEL_COMMAND;
            sendpacket[sendlen++] = MODE_COMMAND;
        }

        // construct the command
        sendpacket[sendlen++] = (uint8_t)(CMD_COMM | FUNCTSEL_RESET | speed);

        // flush the buffers
        flushCom();

        // send the packet
        result = writeCom(sendlen, sendpacket);
        if (result == OneWireMaster::Success)
        {
            // read back the 1 byte response
            result = readCom(1, readbuffer);
            if (result == OneWireMaster::Success)
            {
                // make sure this byte looks like a reset byte
                if (((readbuffer[0] & RB_RESET_MASK) == RB_PRESENCE) || ((readbuffer[0] & RB_RESET_MASK) == RB_ALARMPRESENCE))
                {
                    result = OneWireMaster::Success;
                }
                else
                {
                    result = OneWireMaster::OperationFailure;
                }
            }
        }
    }

    return result;
}

OneWireMaster::CmdResult DS2480B::OWTouchBitSetLevel(uint8_t & sendRecvBit, OWLevel afterLevel)
{
    OneWireMaster::CmdResult result = OneWireMaster::OperationFailure;

    uint8_t readbuffer[10], sendpacket[10];
    uint8_t sendlen = 0;

    // make sure normal level
    OWSetLevel(OneWireMaster::NormalLevel);

    // check for correct mode
    if (mode != MODSEL_COMMAND)
    {
        mode = MODSEL_COMMAND;
        sendpacket[sendlen++] = MODE_COMMAND;
    }

    // construct the command
    sendpacket[sendlen] = (sendRecvBit != 0) ? BITPOL_ONE : BITPOL_ZERO;
    sendpacket[sendlen++] |= CMD_COMM | FUNCTSEL_BIT | speed;

    // flush the buffers
    flushCom();

    // send the packet
    result = writeCom(sendlen, sendpacket);
    if (result == OneWireMaster::Success)
    {
        // read back the response
        result = readCom(1, readbuffer);
        if (result == OneWireMaster::Success)
        {
            // interpret the response
            if (((readbuffer[0] & 0xE0) == 0x80) && ((readbuffer[0] & RB_BIT_MASK) == RB_BIT_ONE))
            {
                sendRecvBit = 1;
                result = OneWireMaster::Success;
            }
            else
            {
                sendRecvBit = 0;
                result = OneWireMaster::Success;
            }
        }
        else
        {
            result = OneWireMaster::CommunicationReadError;
        }
    }
    else
    {
        result = OneWireMaster::CommunicationWriteError;
    }

    if (result == OneWireMaster::Success)
    {
        result = OWSetLevel(afterLevel);
    }

    return result;
}

OneWireMaster::CmdResult DS2480B::OWWriteByteSetLevel(uint8_t sendByte, OWLevel afterLevel)
{
    OneWireMaster::CmdResult result = OneWireMaster::OperationFailure;

    uint8_t readbuffer[10], sendpacket[10];
    uint8_t sendlen = 0;

    // make sure normal level
    OWSetLevel(OneWireMaster::NormalLevel);

    // check for correct mode
    if (mode != MODSEL_DATA)
    {
        mode = MODSEL_DATA;
        sendpacket[sendlen++] = MODE_DATA;
    }

    // add the byte to send
    sendpacket[sendlen++] = sendByte;

    // check for duplication of data that looks like COMMAND mode
    if (sendByte == MODE_COMMAND)
    {
        sendpacket[sendlen++] = sendByte;
    }

    // flush the buffers
    flushCom();

    // send the packet
    result = writeCom(sendlen, sendpacket);
    if (result == OneWireMaster::Success)
    {
        // read back the 1 byte response
        result = readCom(1, readbuffer);
        if ((result == OneWireMaster::Success) && (readbuffer[0] == sendByte))
        {
            result = OneWireMaster::Success;
        }
        else
        {
            result = OneWireMaster::CommunicationReadError;
        }
    }
    else
    {
        result = OneWireMaster::CommunicationWriteError;
    }

    if (result == OneWireMaster::Success)
    {
        result = OWSetLevel(afterLevel);
    }

    return result;
}

OneWireMaster::CmdResult DS2480B::OWReadByteSetLevel(uint8_t & recvByte, OWLevel afterLevel)
{
    OneWireMaster::CmdResult result = OneWireMaster::OperationFailure;

    uint8_t readbuffer[10], sendpacket[10];
    uint8_t sendlen = 0;

    // make sure normal level
    OWSetLevel(OneWireMaster::NormalLevel);

    // check for correct mode
    if (mode != MODSEL_DATA)
    {
        mode = MODSEL_DATA;
        sendpacket[sendlen++] = MODE_DATA;
    }

    // add the byte to send
    sendpacket[sendlen++] = 0xFF;

    // flush the buffers
    flushCom();

    // send the packet
    result = writeCom(sendlen, sendpacket);
    if (result == OneWireMaster::Success)
    {
        // read back the 1 byte response
        result = readCom(1, readbuffer);
        if (result == OneWireMaster::Success)
        {
            recvByte = readbuffer[0];
            result = OneWireMaster::Success;
        }
        else
        {
            result = OneWireMaster::CommunicationReadError;
        }
    }
    else
    {
        result = OneWireMaster::CommunicationWriteError;
    }

    // an error occured so re-sync with DS2480B
    if (result == OneWireMaster::Success)
    {
        result = OWSetLevel(afterLevel);
    }

    return result;
}

OneWireMaster::CmdResult DS2480B::OWSetSpeed(OWSpeed newSpeed)
{
    OneWireMaster::CmdResult result = OneWireMaster::OperationFailure;

    uint8_t sendpacket[5];
    uint8_t sendlen = 0;

    // check if change from current mode
    if (((newSpeed == OneWireMaster::OverdriveSpeed) && (speed != SPEEDSEL_OD)) || 
        ((newSpeed == OneWireMaster::StandardSpeed) && (speed != SPEEDSEL_STD))) 
    {
        if (newSpeed == OneWireMaster::OverdriveSpeed) 
        {
            result = changeBaud(Baud115200bps);
            if (result == OneWireMaster::Success) 
            {
                speed = SPEEDSEL_OD;
            }
        } 
        else if (newSpeed == OneWireMaster::StandardSpeed) 
        {
            result = changeBaud(Baud9600bps);
            if (result == OneWireMaster::Success) 
            {
                speed = SPEEDSEL_STD;
            }
        }

        // if baud rate is set correctly then change DS2480 speed
        if (result == OneWireMaster::Success) 
        {
            // check if correct mode
            if (mode != MODSEL_COMMAND) 
            {
                mode = MODSEL_COMMAND;
                sendpacket[sendlen++] = MODE_COMMAND;
            }

            // proceed to set the DS2480 communication speed
            sendpacket[sendlen++] = CMD_COMM | FUNCTSEL_SEARCHOFF | speed;

            // send the packet
            result = writeCom(sendlen,sendpacket);
        }
    }

    return result;
}

OneWireMaster::CmdResult DS2480B::OWSetLevel(OWLevel newLevel)
{
    OneWireMaster::CmdResult result = OneWireMaster::Success;

    uint8_t sendpacket[10], readbuffer[10];
    uint8_t sendlen = 0;

    // check if need to change level
    if (newLevel != level)
    {
        // check for correct mode
        if (mode != MODSEL_COMMAND)
        {
            mode = MODSEL_COMMAND;
            sendpacket[sendlen++] = MODE_COMMAND;
        }

        // check if just putting back to normal
        if (newLevel == OneWireMaster::NormalLevel)
        {
            // stop pulse command
            sendpacket[sendlen++] = MODE_STOP_PULSE;

            // add the command to begin the pulse WITHOUT prime
            sendpacket[sendlen++] = CMD_COMM | FUNCTSEL_CHMOD | SPEEDSEL_PULSE | BITPOL_5V | PRIME5V_FALSE;

            // stop pulse command
            sendpacket[sendlen++] = MODE_STOP_PULSE;

            // flush the buffers
            flushCom();

            // send the packet
            result = writeCom(sendlen, sendpacket);
            if (result == OneWireMaster::Success)
            {
                // read back the 1 byte response
                result = readCom(2, readbuffer);
                if (result == OneWireMaster::Success)
                {
                    // check response byte
                    if (((readbuffer[0] & 0xE0) == 0xE0) && ((readbuffer[1] & 0xE0) == 0xE0))
                    {
                        level = OneWireMaster::NormalLevel;
                    }
                    else
                    {
                        result = OneWireMaster::OperationFailure;
                    }
                }
            }
        }
        // set new level
        else
        {
            // set the SPUD time value
            sendpacket[sendlen++] = CMD_CONFIG | PARMSEL_5VPULSE | PARMSET_infinite;
            // add the command to begin the pulse
            sendpacket[sendlen++] = CMD_COMM | FUNCTSEL_CHMOD | SPEEDSEL_PULSE | BITPOL_5V;

            // flush the buffers
            flushCom();

            // send the packet
            result = writeCom(sendlen, sendpacket);
            if (result == OneWireMaster::Success)
            {
                // read back the 1 byte response from setting time limit
                result = readCom(1, readbuffer);
                if (result == OneWireMaster::Success)
                {
                    // check response byte
                    if ((readbuffer[0] & 0x81) == 0)
                    {
                        level = newLevel;
                    }
                    else
                    {
                        result = OneWireMaster::OperationFailure;
                    }
                }
            }
        }
    }

    return result;
}

OneWireMaster::CmdResult DS2480B::detect()
{
    OneWireMaster::CmdResult result;

    uint8_t sendpacket[10], readbuffer[10];
    uint8_t sendlen = 0;

    // reset modes
    level = OneWireMaster::NormalLevel;
    mode = MODSEL_COMMAND;
    baud = Baud9600bps;
    speed = SPEEDSEL_FLEX;

    // set the baud rate to 9600
    setComBaud(baud);

    // send a break to reset the DS2480B
    breakCom();

    // delay to let line settle
    wait_ms(2);

    // flush the buffers
    flushCom();

    // send the timing byte
    sendpacket[0] = 0xC1;
    result = writeCom(1, sendpacket);
    if (result == OneWireMaster::Success)
    {
        // delay to let line settle
        wait_ms(2);

        // set the FLEX configuration parameters
        // default PDSRC = 1.37Vus
        sendpacket[sendlen++] = CMD_CONFIG | PARMSEL_SLEW | PARMSET_Slew1p37Vus;
        // default W1LT = 10us
        sendpacket[sendlen++] = CMD_CONFIG | PARMSEL_WRITE1LOW | PARMSET_Write10us;
        // default DSO/WORT = 8us
        sendpacket[sendlen++] = CMD_CONFIG | PARMSEL_SAMPLEOFFSET | PARMSET_SampOff8us;

        // construct the command to read the baud rate (to test command block)
        sendpacket[sendlen++] = CMD_CONFIG | PARMSEL_PARMREAD | (PARMSEL_BAUDRATE >> 3);

        // also do 1 bit operation (to test 1-Wire block)
        sendpacket[sendlen++] = CMD_COMM | FUNCTSEL_BIT | baud | BITPOL_ONE;

        // flush the buffers
        flushCom();

        // send the packet
        result = writeCom(sendlen, sendpacket);
        if (result == OneWireMaster::Success)
        {
            // read back the response
            result = readCom(5, readbuffer);
            if (result == OneWireMaster::Success)
            {
                // look at the baud rate and bit operation
                // to see if the response makes sense
                if (((readbuffer[3] & 0xF1) == 0x00) && ((readbuffer[3] & 0x0E) == baud) && ((readbuffer[4] & 0xF0) == 0x90) && ((readbuffer[4] & 0x0C) == baud))
                {
                    result = OneWireMaster::Success;
                }
                else
                {
                    result = OneWireMaster::OperationFailure;
                }
            }
        }
    }

    return result;
}

OneWireMaster::CmdResult DS2480B::changeBaud(BaudRate newBaud)
{
    OneWireMaster::CmdResult result = OneWireMaster::Success;

    uint8_t readbuffer[5], sendpacket[5], sendpacket2[5];
    uint8_t sendlen = 0, sendlen2 = 0;

    //see if diffenent then current baud rate
    if (baud != newBaud)
    {
        // build the command packet
        // check for correct mode
        if (mode != MODSEL_COMMAND)
        {
            mode = MODSEL_COMMAND;
            sendpacket[sendlen++] = MODE_COMMAND;
        }
        // build the command
        sendpacket[sendlen++] = CMD_CONFIG | PARMSEL_BAUDRATE | newBaud;

        // flush the buffers
        flushCom();

        // send the packet
        result = writeCom(sendlen, sendpacket);
        if (result == OneWireMaster::Success)
        {
            // make sure buffer is flushed
            wait_ms(5);

            // change our baud rate
            setComBaud(newBaud);
            baud = newBaud;

            // wait for things to settle
            wait_ms(5);

            // build a command packet to read back baud rate
            sendpacket2[sendlen2++] = CMD_CONFIG | PARMSEL_PARMREAD | (PARMSEL_BAUDRATE >> 3);

            // flush the buffers
            flushCom();

            // send the packet
            result = writeCom(sendlen2, sendpacket2);
            if (result == OneWireMaster::Success)
            {
                // read back the 1 byte response
                result = readCom(1, readbuffer);
                if (result == OneWireMaster::Success)
                {
                    // verify correct baud
                    if ((readbuffer[0] & 0x0E) == (sendpacket[sendlen - 1] & 0x0E))
                    {
                        result = OneWireMaster::Success;
                    }
                    else
                    {
                        result = OneWireMaster::OperationFailure;
                    }
                }
                else
                {
                    result = OneWireMaster::CommunicationReadError;
                }
            }
            else
            {
                result = OneWireMaster::CommunicationWriteError;
            }
        }
        else
        {
            result = OneWireMaster::CommunicationWriteError;
        }
    }

    return result;
}

OneWireMaster::CmdResult DS2480B::writeCom(size_t outlen, uint8_t *outbuf)
{
    OneWireMaster::CmdResult result = OneWireMaster::OperationFailure;

    mbed::Timer timer;
    uint32_t timeout = calculateBitTimeout(baud) * outlen;
    uint8_t idx = 0;

    timer.start();
    do
    {
        while ((idx < outlen) && serial.writeable())
        {
            serial.putc(outbuf[idx++]);
        }

    } while ((idx < outlen) && (static_cast<unsigned int>(timer.read_us()) < timeout));
    
    if (idx == outlen)
    {
        result = OneWireMaster::Success;
    }
    else
    {
        result = OneWireMaster::TimeoutError;
    }

    return result;
}

OneWireMaster::CmdResult DS2480B::readCom(size_t inlen, uint8_t *inbuf)
{
    OneWireMaster::CmdResult result;
    mbed::Timer timer;
    uint32_t num_bytes_read = 0;
    uint32_t timeout = (calculateBitTimeout(baud) * inlen);

    timer.start();
    do
    {
        while ((num_bytes_read < inlen) && serial.readable())
        {
            inbuf[num_bytes_read++] = serial.getc();
        }
    } while ((num_bytes_read < inlen) && (static_cast<unsigned int>(timer.read_us()) < timeout));
    timer.stop();

    if (num_bytes_read == inlen)
    {
        result = OneWireMaster::Success;
    }
    else
    {
        result = OneWireMaster::TimeoutError;
    }

    return result;
}

void DS2480B::breakCom()
{
    // Switch to lower baud rate to ensure break is longer than 2 ms.
    serial.baud(4800);
    serial.send_break();
    setComBaud(baud);
}

void DS2480B::flushCom()
{
    // Clear receive buffer.
    while (serial.readable())
    {
        serial.getc();
    }

    /* No apparent way to clear transmit buffer.
     * From the example in AN192 (http://pdfserv.maximintegrated.com/en/an/AN192.pdf),
     * the data shouldn't be sent, just aborted and the buffer cleaned out.
     * Below is what was used in AN192 example code using windows drivers:
     * PurgeComm(ComID, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR );
     */
}

void DS2480B::setComBaud(BaudRate new_baud)
{
    switch (new_baud)
    {
    case Baud115200bps:
        serial.baud(115200);
        break;

    case Baud57600bps:
        serial.baud(57600);
        break;

    case Baud19200bps:
        serial.baud(19200);
        break;

    case Baud9600bps:
    default:
        serial.baud(9600);
        break;
    }
}
