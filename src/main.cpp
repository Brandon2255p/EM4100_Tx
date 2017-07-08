#include <Arduino.h>
#include "Manchester.h"

int TX_PIN = 9;

uint8_t tagGarage[5] = {0x30, 0x7E, 0xA4, 0x53, 0x5A};
uint8_t tagWork[5] = {0x30, 0x7E, 0xA1, 0x03, 0x92};
uint8_t tagUCT[5] = {0x30, 0x7E, 0xA4, 0x53, 0x5A};
uint8_t tagUCTJ[5] = {0x4C, 0x00, 0x84, 0xF3, 0xD8};

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
    man.transmitEM4100(tagUCTJ);
  man.SetDelay_us(delay_us++);
  if(delay_us > 220)
    delay_us = 200;
}
