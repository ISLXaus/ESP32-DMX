/**
 * DMX Example: transfer DMX from UART2 to UART1 on ESP32
 * UART2 RX: 16 // has to be UART2 default RX pin
 * UART1 TX: 14 // 13 doesn't work
 * 
*/

#include <dmx.h>
#include <Arduino.h>

uint8_t dmxUniverse[512];

void setup() {
  DMX::Initialize(input);
  DMX1::Initialize(output);
}

void loop()
{
  DMX::ReadAll(dmxUniverse,1,512);
  DMX1::WriteAll(dmxUniverse,1,512);
}
