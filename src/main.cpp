#include <Arduino.h>
#include "Manchester.h"

int TX_PIN = 9;

uint8_t tagGarage[5] = {0x30, 0x7E, 0xA4, 0x53, 0x5A};
uint8_t tagWork[5] = {0x30, 0x7E, 0xA1, 0x03, 0x92};
uint8_t tagUCT[5] = {0x30, 0x7E, 0xA4, 0x53, 0x5A};

void setup()
{
  Serial.begin(115200);
  pinMode(TX_PIN, OUTPUT);
  man.setupTransmit(TX_PIN, MAN_4800);
}

void loop()
{
  man.transmitEM4100(tagWork);
  delay(500);
}
