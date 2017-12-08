#include <Arduino.h>
#include "Manchester.h"

int TX_PIN = 2;

uint8_t tag1[5] = {0x30, 0x7F, 0xA2, 0x53, 0x5A};

uint16_t delay_us = 200;
void setup()
{
  Serial.begin(115200);
  pinMode(TX_PIN, OUTPUT);
  man.setupTransmit(TX_PIN, MAN_4800);
  man.SetDelay_us(delay_us);
}

void loop()
{
  for(int i =0; i < 20; i++)
    man.transmitEM4100(tag1);
  man.SetDelay_us(delay_us++);
  if(delay_us > 220)
    delay_us = 200;
}
