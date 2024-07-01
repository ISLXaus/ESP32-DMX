#include <Arduino.h>
#include <dmx.h>

int readcycle = 0;

#define DMX_SEL_PIN 1
#define DMX_LED_RX_PIN 0  //green LED
#define DMX_LED_TX_PIN 8  //red LED


void setup() {
  Serial.begin(115200);
  DMX::Initialize(DMX_SEL_PIN,DMX_LED_RX_PIN,  input);
}

void loop()
{
  if(millis() - readcycle > 100)
  {
    readcycle = millis();

    Serial.print(readcycle);
      
    if(DMX::IsHealthy())
    {
      Serial.print(": ok - ");
    }
    else
    {
      Serial.print(": fail - ");
    }
    Serial.print(DMX::Read(1));
    Serial.print(" - ");
    Serial.print(DMX::Read(10));
    Serial.print(" - ");
    Serial.println(DMX::Read(256));
  }
}