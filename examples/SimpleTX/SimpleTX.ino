#include <dmx.h>

#define DMX_SEL_PIN -1  //no pin selected

int readcycle = 0;
uint8_t send_value = 0;

void setup() {
  Serial.begin(115200);

  DMX::Initialize(output,DMX_SEL_PIN);
  DMX::Write(8, 69);
  
}

void loop()
{
  if(millis() - readcycle > 25)
  {
    readcycle = millis();

    Serial.print(send_value);

    DMX::Write(10, send_value++);
  }
}
