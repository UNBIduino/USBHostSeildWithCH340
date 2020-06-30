#include "cdcch340.h"
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

#define TESTING

class CH340Async : public CH340AsyncOper
{
  public:
    uint8_t OnInit(CH340 *pch340);
    uint8_t OnRelease(CH340 *pch340);
};

uint8_t CH340Async::OnInit(CH340 *pch340)
{
  uint8_t rcode = 0;
  linecoding lc;
  lc.msr = 0;
  lc.baud_rate = 115200; //19200;
  lc.lcr = CH340_LCR_ENABLE_RX | CH340_LCR_ENABLE_TX | CH340_LCR_CS8;
  rcode = pch340->Configure(&lc);
  if (rcode) {
    ErrorMessage<uint8_t>(PSTR("Configure failed"), rcode);
    return rcode;
  }

#ifdef TESTING

  rcode = pch340->GetModemStatus(&lc);
  if (rcode)
    ErrorMessage<uint8_t>(PSTR("GetModemStatus"), rcode);

  Serial.print("mcr: ");
  Serial.println(lc.msr, HEX);

  /** Not working
    rcode = pch340->SetFlowControl(true);
    if (rcode)
      ErrorMessage<uint8_t>(PSTR("SetFlowControl true"), rcode);

    rcode = pch340->SetFlowControl(false);
    if (rcode)
      ErrorMessage<uint8_t>(PSTR("SetFlowControl false"), rcode);
  ***/
  rcode = pch340->SetModemControl(lc.lcr);
  if (rcode)
    ErrorMessage<uint8_t>(PSTR("SetModemControl"), rcode);

  rcode = pch340->DtrRts(&lc, 1);
  if (rcode)
    ErrorMessage<uint8_t>(PSTR("DTR/RTS on"), rcode);
  delay(500);
  rcode = pch340->DtrRts(&lc, 0);
  if (rcode)
    ErrorMessage<uint8_t>(PSTR("DTR/RTS off"), rcode);

#endif
  return rcode;
}

/**
   Possible clean op when releasing the connection.
*/
uint8_t CH340Async::OnRelease(CH340 *pch340)
{
  uint8_t rcode = 0;
  return rcode;
}

USB              Usb;
//USBHub         Hub(&Usb);
CH340Async       ch340Async;
CH340            ch340(&Usb, &ch340Async);

void setup()
{
  Serial.begin( 115200 );
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  Serial.println("Start");

  if (Usb.Init() == -1)
    Serial.println("OSC did not start.");

  delay( 200 );
}

void loop()
{
  uint8_t  buf[64];

  Usb.Task();

  //if ( Usb.getUsbTaskState() == USB_STATE_RUNNING ) {
  if (ch340.isReady()) {
    uint8_t  rcode;
    char strbuf[] = "DEADBEEF\n";
    //char strbuf[] = "The quick brown fox jumps over the lazy dog\n";
    //char strbuf[] = "This string contains 61 character to demonstrate CH340 buffers"; //add one symbol to it to see some garbage
    Serial.print(".");

    rcode = ch340.SndData(strlen(strbuf), (uint8_t*)strbuf);
    if (rcode)
      ErrorMessage<uint8_t>(PSTR("SndData"), rcode);

    delay(50);

    for (uint8_t i = 0; i < 64; i++)
      buf[i] = 0;

    uint16_t rcvd = 64;
    rcode = ch340.RcvData(&rcvd, buf);
    if (rcode && rcode != hrNAK)
      ErrorMessage<uint8_t>(PSTR("Ret"), rcode);

    if (rcvd) {
      Serial.print((char*)(buf));
    }
    delay(5000);
  }
}
