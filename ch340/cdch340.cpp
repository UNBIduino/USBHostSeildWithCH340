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
#include "cdcch340.h"

CH340::CH340(USB *p, CH340AsyncOper *pasync) :
  pAsync(pasync),
  pUsb(p),
  bAddress(0),
  bNumEP(1),
  wCH340Type(0),
  wIdProduct(0) {
  for (uint8_t i = 0; i < CH340_MAX_ENDPOINTS; i++) {
    epInfo[i].epAddr = 0;
    epInfo[i].maxPktSize = (i) ? 0 : 8;
    epInfo[i].bmSndToggle = 0;
    epInfo[i].bmRcvToggle = 0;
    epInfo[i].bmNakPower = (i == epDataInIndex) ? USB_NAK_NOWAIT : USB_NAK_MAX_POWER;
  }
  if (pUsb)
    pUsb->RegisterDeviceClass(this);
}

uint8_t CH340::Init(uint8_t parent, uint8_t port, bool lowspeed) {
  const uint8_t constBufSize = sizeof (USB_DEVICE_DESCRIPTOR);

  uint8_t buf[constBufSize];
  USB_DEVICE_DESCRIPTOR * udd = reinterpret_cast<USB_DEVICE_DESCRIPTOR*>(buf);
  uint8_t rcode;
  UsbDevice *p = NULL;
  EpInfo *oldep_ptr = NULL;

  uint8_t num_of_conf; // number of configurations

  AddressPool &addrPool = pUsb->GetAddressPool();

  USBTRACE("CH340 Init\r\n");

  if (bAddress) {
    USBTRACE("CH340 CLASS IN USE??\r\n");
    return USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE;
  }
  // Get pointer to pseudo device with address 0 assigned
  p = addrPool.GetUsbDevicePtr(0);

  if (!p) {
    USBTRACE("CH340 NO ADDRESS??\r\n");
    return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
  }
  if (!p->epinfo) {
    USBTRACE("epinfo\r\n");
    return USB_ERROR_EPINFO_IS_NULL;
  }

  // Save old pointer to EP_RECORD of address 0
  oldep_ptr = p->epinfo;

  // Temporary assign new pointer to epInfo to p->epinfo in order to avoid toggle inconsistence
  p->epinfo = epInfo;

  p->lowspeed = lowspeed;

  // Get device descriptor
  rcode = pUsb->getDevDescr(0, 0, sizeof (USB_DEVICE_DESCRIPTOR), buf);

  // Restore p->epinfo
  p->epinfo = oldep_ptr;

  if (rcode) {
    goto FailGetDevDescr;
  }
  //if(udd->idVendor != CH340_VID || udd->idProduct != udd->idProduct) {
  if (!vidPidOk(udd->idVendor, udd->idProduct )) {
    USBTRACE("CH340 Init: Product not supported\r\n");
    USBTRACE2("Found VID:", udd->idVendor);
    USBTRACE2("Found PID:", udd->idProduct);
    return USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED;
  }

  // Save type of CH340 chip
  wCH340Type = udd->bcdDevice;
  wIdProduct = udd->idProduct;

  // Allocate new address according to device class
  bAddress = addrPool.AllocAddress(parent, false, port);

  if (!bAddress)
    return USB_ERROR_OUT_OF_ADDRESS_SPACE_IN_POOL;

  // Extract Max Packet Size from the device descriptor
  epInfo[0].maxPktSize = udd->bMaxPacketSize0;
  // Some devices set endpoint lengths to zero, which is incorrect.
  // we should check them, and if zero, set them to 64.
  if (epInfo[0].maxPktSize == 0) epInfo[0].maxPktSize = 64;

  // Assign new address to the device
  rcode = pUsb->setAddr(0, 0, bAddress);

  if (rcode) {
    p->lowspeed = false;
    addrPool.FreeAddress(bAddress);
    bAddress = 0;
    USBTRACE2("setAddr:", rcode);
    return rcode;
  }

  USBTRACE2("Addr:", bAddress);

  p->lowspeed = false;

  p = addrPool.GetUsbDevicePtr(bAddress);

  if (!p)
    return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

  p->lowspeed = lowspeed;

  num_of_conf = udd->bNumConfigurations;

  // Assign epInfo to epinfo pointer
  rcode = pUsb->setEpInfoEntry(bAddress, 1, epInfo);

  if (rcode)
    goto FailSetDevTblEntry;

  USBTRACE2("NC:", num_of_conf);

  for (uint8_t i = 0; i < num_of_conf; i++) {
    ConfigDescParser < 0, 0, 0, 0> confDescrParser(this);

    // This interferes with serial output, and should be opt-in for debugging.
    //HexDumper<USBReadParser, uint16_t, uint16_t> HexDump;
    //rcode = pUsb->getConfDescr(bAddress, 0, i, &HexDump);
    //if(rcode)
    //        goto FailGetConfDescr;

    rcode = pUsb->getConfDescr(bAddress, 0, i, &confDescrParser);

    if (rcode)
      goto FailGetConfDescr;

    if (bNumEP > 1)
      break;
  } // for

  if (bNumEP < 2)
    return USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED;

  USBTRACE2("NumEP:", bNumEP);

  // Assign epInfo to epinfo pointer
  rcode = pUsb->setEpInfoEntry(bAddress, bNumEP, epInfo);

  USBTRACE2("Conf:", bConfNum);

  // Set Configuration Value
  rcode = pUsb->setConf(bAddress, 0, bConfNum);

  if (rcode)
    goto FailSetConfDescr;
#if 0
  // default latency is 16ms on-chip, reduce it to 1
  rcode = SetLatency(1);
  if (rcode)
    goto FailOnLatency;

#endif
  rcode = pAsync->OnInit(this);

  if (rcode)
    goto FailOnInit;

  USBTRACE("CH340 configured\r\n");

  ready = true;
  return 0;

FailOnLatency:
#ifdef DEBUG_USB_HOST
  USBTRACE("SetLatency: ");
  goto Fail;
#endif

FailGetDevDescr:
#ifdef DEBUG_USB_HOST
  NotifyFailGetDevDescr();
  goto Fail;
#endif

FailSetDevTblEntry:
#ifdef DEBUG_USB_HOST
  NotifyFailSetDevTblEntry();
  goto Fail;
#endif

FailGetConfDescr:
#ifdef DEBUG_USB_HOST
  NotifyFailGetConfDescr();
  goto Fail;
#endif

FailSetConfDescr:
#ifdef DEBUG_USB_HOST
  NotifyFailSetConfDescr();
  goto Fail;
#endif

FailOnInit:
#ifdef DEBUG_USB_HOST
  USBTRACE("OnInit:");

Fail:
  NotifyFail(rcode);
#endif
  Release();
  return rcode;
}

/**
 * @brief vidPidOk look up vid and pid in table
 *
 * @param[in] vid The retrieved vid from the chip
 * @param[in] pid The retrieved pid from the chip
 *
 * @return true if a valid vid and pid pair found.
 */
bool CH340::vidPidOk(uint16_t vid, uint16_t pid) {
  for (int i = 0; i < NO_VIDS; ++i) {
    if (pvids[i].vendor == vid && pvids[i].product == pid)
      return true;
  }
  return false;


}

void CH340::EndpointXtract(uint8_t conf, uint8_t iface, uint8_t alt, uint8_t proto __attribute__((unused)), const USB_ENDPOINT_DESCRIPTOR *pep) {
  ErrorMessage<uint8_t > (PSTR("Conf.Val"), conf);
  ErrorMessage<uint8_t > (PSTR("Iface Num"), iface);
  ErrorMessage<uint8_t > (PSTR("Alt.Set"), alt);

  bConfNum = conf;

  uint8_t index;

  if ((pep->bmAttributes & bmUSB_TRANSFER_TYPE) == USB_TRANSFER_TYPE_INTERRUPT && (pep->bEndpointAddress & 0x80) == 0x80)
    index = epInterruptInIndex;
  else if ((pep->bmAttributes & bmUSB_TRANSFER_TYPE) == USB_TRANSFER_TYPE_BULK)
    index = ((pep->bEndpointAddress & 0x80) == 0x80) ? epDataInIndex : epDataOutIndex;
  else
    return;

  // Fill in the endpoint info structure
  epInfo[index].epAddr = (pep->bEndpointAddress & 0x0F);
  epInfo[index].maxPktSize = (uint8_t)pep->wMaxPacketSize;
  epInfo[index].bmSndToggle = 0;
  epInfo[index].bmRcvToggle = 0;
  // Some device vendors set endpoint lengths to zero, which is incorrect.
  // Check, and if zero, set to 64.
  if (epInfo[index].maxPktSize == 0) epInfo[index].maxPktSize = 64;

  bNumEP++;

  PrintEndpointDescriptor(pep);
}

uint8_t CH340::Release() {
  pUsb->GetAddressPool().FreeAddress(bAddress);

  bAddress = 0;
  bNumEP = 1;
  qNextPollTime = 0;
  bPollEnable = false;
  ready = false;
  return pAsync->OnRelease(this);
}

uint8_t CH340::Poll() {
  uint8_t rcode = 0;

  //if (!bPollEnable)
  //      return 0;

  //if (qNextPollTime <= (uint32_t)millis())
  //{
  //      USB_HOST_SERIAL.println(bAddress, HEX);

  //      qNextPollTime = (uint32_t)millis() + 100;
  //}
  return rcode;
}

/**
 *  @brief SetBaudRate set baud rate and no. bits, parity etc.
 *
 *  @param[in] baud The baud rate
 *  @param[in] lcr  The no. bits, stop bits, parity, enable rx and tx
 *  @return USB result code or 255 if unacceptable baud rate
 */
uint8_t CH340::SetBaudRate(uint32_t baud, uint8_t lcr) {
  uint16_t a, b;
  uint8_t r;
  unsigned long factor;
  uint16_t divisor;
  const uint16_t cmd1 = 0x1312;
  const uint16_t cmd2 = 0x0f2c;

  if (!baud)
    return 255;
  factor = (CH340_BAUDBASE_FACTOR / baud);
  divisor = CH340_BAUDBASE_DIVMAX;

  while ((factor > 0xfff0) && divisor) {
    factor >>= 3;
    divisor--;
  }

  if (factor > 0xfff0)
    return 255;

  factor = 0x10000 - factor;
  a = (factor & 0xff00) | divisor;

  uint8_t rv = ( pUsb->ctrlReq(bAddress, 0, bmREQ_CH340_OUT, CH340_REQ_WRITE_REG, cmd1 & 0xff, cmd1 >> 8 , a, 0, 0, NULL, NULL));
  if (!rv)
    rv = ( pUsb->ctrlReq(bAddress, 0, bmREQ_CH340_OUT, CH340_REQ_WRITE_REG, cmd2 & 0xff, cmd2 >> 8 , lcr, 0, 0, NULL, NULL));
  if (rv && rv != hrNAK) {
    Release();
  }

  return rv;
}

/**
 *  @brief SetBaudRateOnly sets baud rate only
 *
 *  @param[in] baud The baud rate
 *  @return USB result code or 255 if unacceptable baud rate
 */
uint8_t CH340::SetBaudRateOnly(uint32_t baud) {
  uint16_t a, b;
  uint8_t r;
  unsigned long factor;
  uint16_t divisor;
  const uint16_t cmd1 = 0x1312;
  const uint16_t cmd2 = 0x0f2c;

  if (!baud)
    return 255;
  factor = (CH340_BAUDBASE_FACTOR / baud);
  divisor = CH340_BAUDBASE_DIVMAX;

  while ((factor > 0xfff0) && divisor) {
    factor >>= 3;
    divisor--;
  }

  if (factor > 0xfff0)
    return 255;

  factor = 0x10000 - factor;
  a = (factor & 0xff00) | divisor;
  b = factor & 0xff;

  uint8_t rv = ( pUsb->ctrlReq(bAddress, 0, bmREQ_CH340_OUT, CH340_REQ_WRITE_REG, cmd1 & 0xff, cmd1 >> 8 , a, 0, 0, NULL, NULL));
  if (!rv)
    rv = ( pUsb->ctrlReq(bAddress, 0, bmREQ_CH340_OUT, CH340_REQ_WRITE_REG, cmd2 & 0xff, cmd2 >> 8 , b, 0, 0, NULL, NULL));
  if (rv && rv != hrNAK) {
    Release();
  }
  return rv;
}

/**
 * @brief SetHandShake sends the dtr/rts settings
 *    Reverses the bits ad dtr and rts are active low.
 * @param[in] control DTR/RTS settings
 * @return USB result code
 */
uint8_t CH340::SetHandShake(uint8_t control) {
  uint8_t rv = ( pUsb->ctrlReq(bAddress, 0, bmREQ_CH340_OUT, CH340_REQ_MODEM_CTRL, ~control, 0, 0,  0, 0, 0, NULL));
  return rv;
}

/**
 * @brief SetHandShake decode set/clear the dtr/rts settings
 *
 * @param[in] plc DTR/RTS settings
 * @param[in] on true = set, false clear
 *
 * @return USB result code
 */
uint8_t CH340::DtrRts(linecoding *plc, uint8_t on) {
  if (on)
    plc->msr |= CH340_BIT_RTS | CH340_BIT_DTR;
  else
    plc->msr &= ~(CH340_BIT_RTS | CH340_BIT_DTR);

  uint8_t rv = SetHandShake(plc->msr);
  if (rv && rv != hrNAK) {
    Release();
  }
  return rv;
}

/**
 *  @brief GetCtrlStatus
 *
 *  @param[in] value The status to ask for ?
 *  @param[in] index ?
 *  @param[in] nbytes The size of the buffer
 *  @param[out] dataptr The result of the request returned here
 *  @return USB result code
 */
uint8_t CH340::GetCtrlStatus(uint16_t value, uint16_t index, uint16_t nbytes, uint8_t* dataptr) {
  return ( pUsb->ctrlReq(bAddress, 0, bmREQ_CH340_IN, CH340_REQ_READ_REG, value & 0xff, value >> 8,  index,  nbytes, nbytes, dataptr, NULL));
}


/**
 * @brief GetModemStatus
 *
 *  Get the modem status bits.
 *
 * @param[out] plc The result of the request is returned in the msr member
 *
 * @return USB result code
 */
uint8_t CH340::GetModemStatus(linecoding *plc) {
  const uint16_t size = 8;
  uint8_t buffer[size];

  uint8_t rv = GetCtrlStatus(0x0706, 0, size, buffer);
  if ( rv >= 0 )
    plc->msr = (~(*buffer)) & CH340_BITS_MODEM_STAT;
  return rv;
}

/**
 * @brief SetModemControl
 *
 *   Send the modem control values.
 *
 * @param[in] modemctrl
 *
 * @return USB result code
 */
uint8_t CH340::SetModemControl(uint16_t modemctrl) {
  uint8_t rv = pUsb->ctrlReq(bAddress, 0, bmREQ_CH340_OUT, CH340_REQ_MODEM_CTRL, modemctrl & 0xff, modemctrl >> 8, 0, 0, 0, NULL, NULL);
  if (rv && rv != hrNAK) {
    Release();
  }
  return rv;
}

/**
 *  @brief SetCtrlStatus
 *
 *    Write a 16 bit int to the control register
 *
 *  @param[in] value 16 bit value to set
 *  @param[in] index index value
 *  @return USB result code
 */
uint8_t CH340::SetCtrlStatus(uint16_t value, uint16_t index) {
  uint8_t rv = ( pUsb->ctrlReq(bAddress, 0, bmREQ_CH340_OUT, CH340_REQ_WRITE_REG, value & 0xff, value >> 8,  index,  0, 0, NULL, NULL));
  return rv;
}

#ifdef USE_FLOW_CONTROL
//Setting flow control ON fails with return code 0x0d !
/**
 * @brief SetFlowControl
 *
 *  Send the xon/xoff control.
 * @param[in] on        true = xon/off active, false = xon/xoff inactive
 * @return USB result code
 */
uint8_t CH340::SetFlowControl(uint8_t on) {
  uint8_t rv = 0;
  if (on) {
    rv = SetCtrlStatus( 0x2727, 0x0101);
  } else {
    rv = SetCtrlStatus(0x2727, 0x0000);
  }
  if (rv && rv != hrNAK) {
    Release();
  }
  return rv;
}
#endif

/**
 * @brief RcvData
 *  Receive data packet
 * 
 * @param[out] bytes_rcvd Number of bytes received
 * @param[out] dataptr    Buffer for received data
 * 
 * @return USB result code
 */
uint8_t CH340::RcvData(uint16_t *bytes_rcvd, uint8_t *dataptr) {
  uint8_t rv = pUsb->inTransfer(bAddress, epInfo[epDataInIndex].epAddr, bytes_rcvd, dataptr);
  if (rv && rv != hrNAK) {
    Release();
  }
  return rv;
}

/**
 * @brief SndData
 *  Send data packet
 * 
 * @param[in] bytes_rcvd Number of bytes to send
 * @param[in] dataptr    Buffer for data to be send
 * 
 * @return USB result code
 */
uint8_t CH340::SndData(uint16_t nbytes, uint8_t *dataptr) {
  uint8_t rv = pUsb->outTransfer(bAddress, epInfo[epDataOutIndex].epAddr, nbytes, dataptr);
  if (rv && rv != hrNAK) {
    Release();
  }
  return rv;
}


/**
 * @brief Configure
 *
 *  Initialise the CH340
 *
 * @param[in] plc   The baud rate, line control and modem control values to be used.
 *
 * @return USB result code
 */
uint8_t CH340::Configure(linecoding *plc) {

  const uint16_t nbytes = 2;
  uint8_t buffer[nbytes];

  uint8_t rv = ( pUsb->ctrlReq(bAddress, 0, bmREQ_CH340_IN, CH340_REQ_READ_VERSION, 0, 0, 0,  nbytes, nbytes, buffer, NULL));
  if (!rv ) {
    USBTRACE2("Chip Vers:", buffer[0]);

    rv = ( pUsb->ctrlReq(bAddress, 0, bmREQ_CH340_OUT, CH340_REQ_SERIAL_INIT, 0, 0, 0, 0, 0, NULL, NULL));
    if (!rv) {
      rv = SetBaudRate(plc->baud_rate, plc->lcr);
      if (!rv ) {
        rv = SetHandShake(plc->mcr);
      }
    }
  }
  if (rv && rv != hrNAK) {
    Release();
  }
  return rv;
}

void CH340::PrintEndpointDescriptor(const USB_ENDPOINT_DESCRIPTOR* ep_ptr) {
  Notify(PSTR("Endpoint descriptor:"), 0x80);
  Notify(PSTR("\r\nLength:\t\t"), 0x80);
  D_PrintHex<uint8_t > (ep_ptr->bLength, 0x80);
  Notify(PSTR("\r\nType:\t\t"), 0x80);
  D_PrintHex<uint8_t > (ep_ptr->bDescriptorType, 0x80);
  Notify(PSTR("\r\nAddress:\t"), 0x80);
  D_PrintHex<uint8_t > (ep_ptr->bEndpointAddress, 0x80);
  Notify(PSTR("\r\nAttributes:\t"), 0x80);
  D_PrintHex<uint8_t > (ep_ptr->bmAttributes, 0x80);
  Notify(PSTR("\r\nMaxPktSize:\t"), 0x80);
  D_PrintHex<uint16_t > (ep_ptr->wMaxPacketSize, 0x80);
  Notify(PSTR("\r\nPoll Intrv:\t"), 0x80);
  D_PrintHex<uint8_t > (ep_ptr->bInterval, 0x80);
  Notify(PSTR("\r\n"), 0x80);
}
