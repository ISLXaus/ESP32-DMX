#include <dmx.h>

#define DMX_SEL_PIN -1  //no pin selected

int readcycle = 0;

void setup() {
  Serial.begin(115200);
  DMX::Initialize(input, DMX_SEL_PIN);
}

void loop()
{
  if(millis() - readcycle > 1000)
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
    Serial.print(DMX::Read(110));
    Serial.print(" - ");
    Serial.println(DMX::Read(256));
  }
}
