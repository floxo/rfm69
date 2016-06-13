/*
 * rfm69hcw_registers.h
 * Author: Flox
 * v0.1
 * https://github.com/floxo/rfm69.git
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See LICENSE for more details.
 *
 */

#ifndef RFM69HCW_REGISTERS_H_
#define RFM69HCW_REGISTERS_H_

// Common Registers
#define REG_FIFO                0x00
#define REG_OPMODE              0x01  // 000 sleep, 001 standby, 010 FS, 011 TX, 100 RX
#define REG_DATAMODUL           0x02  // 00 FSK, 01 OOK
#define REG_BITRATEMSB          0x03
#define REG_BITRATELSB          0x04
#define REG_FDEVMSB             0x05
#define REG_FDEVLSB             0x06
#define REG_FRFMSB              0x07
#define REG_FRFMID              0x08
#define REG_FRFLSB              0x09
#define REG_OSC1                0x0A
#define REG_AFCCTRL             0x0B
#define REG_LISTEN1             0x0D
#define REG_LISTEN2             0x0E
#define REG_LISTEN3             0x0F
#define REG_VERSION             0x10

// TX Registers
#define REG_PALEVEL             0x11
#define REG_PARAMP              0x12
#define REG_OCP                 0x13
#define REG_LNA                 0x18
#define REG_RXBW                0x19
#define REG_AFCBW               0x1A
#define REG_OOKPEAK             0x1B
#define REG_OOKAVG              0x1C
#define REG_OOKFIX              0x1D
#define REG_AFCFEI              0x1E
#define REG_AFCMSB              0x1F
#define REG_AFCLSB              0x20
#define REG_FEIMSB              0x21
#define REG_FEILSB              0x22
#define REG_RSSICONFIG          0x23
#define REG_RSSIVALUE           0x24

// IRQ and pin mapping registers
#define REG_DIOMAPPING1         0x25
#define REG_DIOMAPPING2         0x26
#define REG_IRQFLAGS1           0x27
#define REG_IRQFLAGS2           0x28
#define REG_RSSITHRESH          0x29
#define REG_RXTIMEOUT1          0x2A
#define REG_RXTIMEOUT2          0x2B

// Packet engine registers
#define REG_PREAMBLEMSB         0x2C  // preamble size MSB
#define REG_PREAMBLELSB         0x2D  // Preamble size LSB
#define REG_SYNCCONFIG          0x2E  // Bit7 sync word 0 Off, 1 On;
                                      // Bit6 FIFO filling 0 on SyncAddress, 1 as long as FifoFillCondidtion set
                                      // Bit5-3 Size of sync word: (Syncsize+1) bytes;
                                      // Bit2-0 # of tolerated bit errors in Sync word
#define REG_SYNCVALUE1          0x2F  // 1st byte sync word
#define REG_SYNCVALUE2          0x30  // 2nd ...
#define REG_SYNCVALUE3          0x31
#define REG_SYNCVALUE4          0x32
#define REG_SYNCVALUE5          0x33
#define REG_SYNCVALUE6          0x34
#define REG_SYNCVALUE7          0x35
#define REG_SYNCVALUE8          0x36
#define REG_PACKETCONFIG1       0x37  // Bit7 Packet format 0 fixed length, 1 variable length;
                                      // Bit6-5 DcFree encoding 00 None, 01 Manchester, 10 Whitening
                                      // Bit4 CrcOn 0 Off, 1 On
                                      // Bit3 CrcAutoClearOff 0 Clear FIFO, restart RX, no PayloadReady IRQ, 1 keep FIFO, PayloadReady IRQ issued
                                      // Bit2-1 Address filtering 00 None, 01 Address field must match NodeAddress, Address field must match NodeAddress or BroadcastAddress
#define REG_PAYLOADLENGTH       0x38  // payload length if fixed length (-> PacketFormat), max length in RX if variable length
#define REG_NODEADRS            0x39  // NodeAddress
#define REG_BROADCASTADRS       0x3A  // Broadcast Address
#define REG_AUTOMODES           0x3B  // Bit 7-5 EnterCondition: Interrupt condition for entering the intermediate mode
                                      // Bit 4-2 ExitCondition: Interrupt condition for exiting the intermediate mode
                                      // Bit 1-0 IntermediateMode: 00 Sleep mode, 01 Standby, 10 RX, 11 TX
#define REG_FIFOTHRESH          0x3C  // TxStartCondition 0 FIFO bytes > FifoThreshhold, 1 FIFO >= 1 byte
#define REG_PACKETCONFIG2       0x3D  // Bit 7-4 InterPacketRxDelay
                                      // Bit 2 RestartRx forces wait mode in continous mode
                                      // Bit 1 AutoRxRestartOn 0 automatic rx restart off, 1 on
                                      // Bit 0 AesOn 0 off, 1 on (66 byte payload maximum!)
#define REG_AESKEY1             0x3E
#define REG_AESKEY2             0x3F
#define REG_AESKEY3             0x40
#define REG_AESKEY4             0x41
#define REG_AESKEY5             0x42
#define REG_AESKEY6             0x43
#define REG_AESKEY7             0x44
#define REG_AESKEY8             0x45
#define REG_AESKEY9             0x46
#define REG_AESKEY10            0x47
#define REG_AESKEY11            0x48
#define REG_AESKEY12            0x49
#define REG_AESKEY13            0x4A
#define REG_AESKEY14            0x4B
#define REG_AESKEY15            0x4C
#define REG_AESKEY16            0x4D

// Temperature Sensor Registers
#define REG_TEMP1               0x4E  // Bit3 TempMeasStart (on write "1")
                                      // Bit2 TempMeasRunning (while 1)
#define REG_TEMP2               0x4F  // TempValue (-1°C per LSB)

// Test Registers
#define REG_TESTLNA             0x58  // SensitivityBoost        0x1B normal, 0x2D high sensitivity
#define REG_TESTPA1             0x5A  // Pa20dBm1        0x55 normal and RX mode, 0x5D +20dBm mode
#define REG_TESTPA2             0x5C  // Pa20dBm2        0x70 normal and RX mode, 0x7C +20dBm mode
#define REG_TESTDAGC            0x6F  // ContinousDagc        Fading margin improvement, datasheet 3.4.4, 0x00 normal, 0x20 improved margin use if AfcLowBetaOn=1, 0x30 improved use if AfcLowBetaOn=0
#define REG_TESTAFC             0x71  // LowBetaAfcOffset AFC offset for low modulation index systems, used if AfcLowBetaOn=1, Offset = LowBetaAfcOffset x 488Hz


#endif /* RFM69HCW_REGISTERS_H_ */
