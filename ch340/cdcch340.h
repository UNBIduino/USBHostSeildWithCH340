/* Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

  This software may be distributed and modified under the terms of the GNU
  General Public License version 2 (GPL2) as published by the Free Software
  Foundation and appearing in the file GPL2.TXT included in the packaging of
  this file. Please note that GPL2 Section 2[b] requires that all works based
  on this software must also be made publicly available under the terms of
  the GPL2 ("Copyleft").

  Contact information
  -------------------

  Circuits At Home, LTD
  Web      :  http://www.circuitsathome.com
  e-mail   :  support@circuitsathome.com
*/

/**
 * USB Error codes - for reference
 0 x*00  hrSUCCESS Successful Transfer
 0x01  hrBUSY SIE is busy, transfer pending
 0x02  hrBADREQ Bad value in HXFR reg
 0x03  hrUNDEF (reserved)
 0x04  hrNAK Peripheral returned NAK
 0x05  hrSTALL Perpheral returned STALL
 0x06  hrTOGERR  Toggle error/ISO over-underrun
 0x07  hrWRONGPID  Received the wrong PID
 0x08  hrBADBC Bad byte count
 0x09  hrPIDERR  Receive PID is corrupted
 0x0A  hrPKTERR  Packet error (stuff, EOP)
 0x0B  hrCRCERR  CRC error
 0x0C  hrKERR  K-state instead of response
 0x0D  hrJERR  J-state instead of response
 0x0E  hrTIMEOUT  Device did not respond in time
 0x0F  hrBABBLE  Device talked too long
 */


#if !defined(__CDCCH340_H__)
#define __CDCCH340_H__

#include "Usb.h"


#define bmREQ_CH340_IN     USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_VENDOR|USB_SETUP_RECIPIENT_DEVICE
#define bmREQ_CH340_OUT    USB_SETUP_HOST_TO_DEVICE|USB_SETUP_TYPE_VENDOR|USB_SETUP_RECIPIENT_DEVICE


#define DEFAULT_BAUD_RATE 9600
#define DEFAULT_TIMEOUT   1000

/* flags for IO-Bits */
#define CH340_BIT_RTS (1 << 6)
#define CH340_BIT_DTR (1 << 5)

#define CH340_BIT_CTS 0x01
#define CH340_BIT_DSR 0x02
#define CH340_BIT_RI  0x04
#define CH340_BIT_DCD 0x08
#define CH340_BITS_MODEM_STAT 0x0f /* all bits */

/*******************************/
/* baudrate calculation factor */
/*******************************/
#define CH340_BAUDBASE_FACTOR 1532620800
#define CH340_BAUDBASE_DIVMAX 3

/* Break support - not implemented */

#define CH340_REQ_READ_VERSION 0x5F
#define CH340_REQ_WRITE_REG    0x9A
#define CH340_REQ_READ_REG     0x95
#define CH340_REQ_SERIAL_INIT  0xA1
#define CH340_REQ_MODEM_CTRL   0xA4

#define CH340_REG_BREAK        0x05
#define CH340_REG_LCR          0x18
#define CH340_NBREAK_BITS      0x01

#define CH340_LCR_ENABLE_RX    0x80
#define CH340_LCR_ENABLE_TX    0x40
#define CH340_LCR_MARK_SPACE   0x20
#define CH340_LCR_PAR_EVEN     0x10
#define CH340_LCR_ENABLE_PAR   0x08
#define CH340_LCR_STOP_BITS_2  0x04
#define CH340_LCR_CS8          0x03
#define CH340_LCR_CS7          0x02
#define CH340_LCR_CS6          0x01
#define CH340_LCR_CS5          0x00

#define NO_VIDS                3
#define CH340_MAX_ENDPOINTS    4

const struct usb_device_id {
  uint16_t vendor;
  uint16_t product;
} devids[] = {
  0x4348, 0x5523,
  0x1a86, 0x7523,
  0x1a86, 0x5523
};

typedef struct {
  uint32_t baud_rate; /* set baud rate */
  uint8_t mcr;        //RTS, DTR
  uint8_t msr;        //MODEM_STAT
  uint8_t lcr;        //ENABLE_RX, ENABLE_TX, bits 5,6,7,8, parity, stop bits
} linecoding;

class CH340;

class CH340AsyncOper {
  public:

    virtual uint8_t OnInit(CH340 *pftdi __attribute__((unused))) {
      return 0;
    };

    virtual uint8_t OnRelease(CH340 *pftdi __attribute__((unused))) {
      return 0;
    };

};

class CH340 : public USBDeviceConfig, public UsbConfigXtracter {
    static const uint8_t epDataInIndex = 1; // DataIn endpoint index
    static const uint8_t epDataOutIndex = 2; // DataOUT endpoint index
    static const uint8_t epInterruptInIndex = 3; // InterruptIN  endpoint index

    CH340AsyncOper *pAsync;
    USB *pUsb;
    uint8_t bAddress;
    uint8_t bConfNum; // configuration number
    uint8_t bNumIface; // number of interfaces in the configuration
    uint8_t bNumEP; // total number of EP in the configuration
    uint32_t qNextPollTime; // next poll time
    volatile bool bPollEnable; // poll enable flag
    volatile bool ready; //device ready indicator
    uint16_t wCH340Type; // Type of CH340 chip
    uint16_t wIdProduct; //  PID


    EpInfo epInfo[CH340_MAX_ENDPOINTS];
    const struct usb_device_id *pvids = devids;

    void PrintEndpointDescriptor(const USB_ENDPOINT_DESCRIPTOR* ep_ptr);

  public:
    CH340(USB *pusb, CH340AsyncOper *pasync);

    uint8_t SetBaudRate(uint32_t baud, uint8_t lcr);
    uint8_t SetBaudRateOnly(uint32_t baud);
    uint8_t SetModemControl(uint16_t control);
    //uint8_t SetFlowControl(uint8_t on);       //Not working
    uint8_t SetHandShake(uint8_t control);
    uint8_t DtrRts(linecoding *plc, uint8_t on);
    uint8_t Configure(linecoding *plc);
    uint8_t GetCtrlStatus(uint16_t value, uint16_t index, uint16_t nbytes, uint8_t* dataptr);
    uint8_t GetModemStatus(linecoding *plc);
    uint8_t SetCtrlStatus(uint16_t value, uint16_t index);

    // Methods for receiving and sending data
    uint8_t RcvData(uint16_t *bytes_rcvd, uint8_t *dataptr);
    uint8_t SndData(uint16_t nbytes, uint8_t *dataptr);

    // USBDeviceConfig implementation
    uint8_t Init(uint8_t parent, uint8_t port, bool lowspeed);
    uint8_t Release();
    uint8_t Poll();

    virtual uint8_t GetAddress() {
      return bAddress;
    };

    // UsbConfigXtracter implementation
    void EndpointXtract(uint8_t conf, uint8_t iface, uint8_t alt, uint8_t proto, const USB_ENDPOINT_DESCRIPTOR *ep);

    bool vidPidOk(uint16_t vid, uint16_t pid);

    virtual bool isReady() {
      return ready;
    };

};

#endif // __CDCCH340_H__
