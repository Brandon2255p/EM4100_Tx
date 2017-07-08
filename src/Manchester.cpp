#include "Manchester.h"

Manchester::Manchester() //constructor
{
  applyWorkAround1Mhz = 0;
  reverseManchester = false;
}


void Manchester::setTxPin(uint8_t pin)
{
  TxPin = pin; // user sets the digital pin as output
  pinMode(TxPin, OUTPUT);
}

void Manchester::workAround1MhzTinyCore(uint8_t a)
{
  applyWorkAround1Mhz = a;
}

void Manchester::setupTransmit(uint8_t pin, uint8_t SF)
{
  setTxPin(pin);
  speedFactor = SF;
  //we don't use exact calculation of passed time spent outside of transmitter
  //because of high ovehead associated with it, instead we use this
  //emprirically determined values to compensate for the time loss

  #if F_CPU == 1000000UL
    uint16_t compensationFactor = 88; //must be divisible by 8 for workaround
  #elif F_CPU == 8000000UL
    uint16_t compensationFactor = 12;
  #else //16000000Mhz
    uint16_t compensationFactor = 4;
  #endif

#if (F_CPU == 80000000UL) || (F_CPU == 160000000)   // ESP8266 80MHz or 160 MHz
  delay1 = delay2 = (HALF_BIT_INTERVAL >> speedFactor) - 2;
#else
  delay1 = (HALF_BIT_INTERVAL >> speedFactor) - compensationFactor;
  delay2 = (HALF_BIT_INTERVAL >> speedFactor) - 2;

  #if F_CPU == 1000000UL
    delay2 -= 22; //22+2 = 24 is divisible by 8
    if (applyWorkAround1Mhz) { //definition of micro delay is broken for 1MHz speed in tiny cores as of now (May 2013)
      //this is a workaround that will allow us to transmit on 1Mhz
      //divide the wait time by 8
      delay1 >>= 3;
      delay2 >>= 3;
    }
  #endif
#endif
}

/*
Send data in the EM4100 Protocol

Start with a sync stream of 9 high bits, then 5 data bytes with nibble parity. It ends with a column parity nibble and a stop bit.
All parity is even parity.
PR represents parity in the nibble. PC represents parity of the bits in the columns. S0 represents the stop bit which is always 0.
8 bit version  number 	D00	D01	D02	D03	 PR0
or customer ID.	        D04	D05	D06	D07	 PR1
                        D08	D09	D10	D11	 PR2 	Each group of 4 bits
                        D12	D13	D14	D15	 PR3 	is followed by an Even
32 Data Bits	        D16	D17	D18	D19	 PR4 	parity bit
                        D20	D21	D22	D23	 PR5
                        D24	D25	D26	D27	 PR6
                        D28	D29	D30	D31	 PR7
                        D32	D33	D34	D35	 PR8
                        D36	D37	D38	D39	 PR9
4 column Parity bits	PC0	PC1	PC2	PC3	 S0
*/
void Manchester::transmitEM4100(uint8_t *data)
{
  const int numDataBytes = 5;

  for( int8_t i = 0; i < SYNC_PULSE_DEF; i++) //send capture pulses
  {
    sendOne(); //end of capture pulses
  }
  //Serial.println("");
  uint8_t parityBit0Count = 0;
  uint8_t parityBit1Count = 0;
  uint8_t parityBit2Count = 0;
  uint8_t parityBit3Count = 0;

  // Send the user data
  for (uint8_t i = 0; i < numDataBytes; i++)
  {
    uint8_t d = data[i];

    //for (uint8_t j = 0; j < ; j++)
    {
      uint8_t numHighBits = 0;
      if ((d & 0x80) == 0)
        sendZero();
      else{
        numHighBits++;
        parityBit0Count++;
        sendOne();
      }
      if ((d & 0x40) == 0)
        sendZero();
      else{
        numHighBits++;
        parityBit1Count++;
        sendOne();
      }
      if ((d & 0x20) == 0)
        sendZero();
      else{
        numHighBits++;
        parityBit2Count++;
        sendOne();
      }
      if ((d & 0x10) == 0)
        sendZero();
      else{
        numHighBits++;
        parityBit3Count++;
        sendOne();
      }
      sendEvenParity(numHighBits);
//Serial.println("");
      numHighBits = 0;
      if ((d & 0x08) == 0)
        sendZero();
      else{
        numHighBits++;
        parityBit0Count++;
        sendOne();
      }
      if ((d & 0x04) == 0)
        sendZero();
      else{
        numHighBits++;
        parityBit1Count++;
        sendOne();
      }
      if ((d & 0x02) == 0)
        sendZero();
      else{
        numHighBits++;
        parityBit2Count++;
        sendOne();
      }
      if ((d & 0x01) == 0)
        sendZero();
      else{
        numHighBits++;
        parityBit3Count++;
        sendOne();
      }
      sendEvenParity(numHighBits);
//Serial.println("");
    }//end of byte
  }//end of data
  //Serial.println("PARITY");
    //Parity Nibble
    if(parityBit0Count % 2 == 0)
    sendZero();
    else
    sendOne();
    if(parityBit1Count % 2 == 0)
    sendZero();
    else
    sendOne();
    if(parityBit2Count % 2 == 0)
    sendZero();
    else
    sendOne();
    if(parityBit3Count % 2 == 0)
    sendZero();
    else
    sendOne();
    // Stop bit
    sendZero();
    //Serial.println("");
}//end of send the data

void Manchester::sendEvenParity(int numHighBits){
  bool evenHighBits = numHighBits % 2 == 0;
  if(evenHighBits)
    sendZero();
  else
    sendOne();
}

void Manchester::sendZero(void)
{
    //Serial.print('0');
	if (reverseManchester)
		sendLowHigh();
	else
		sendHighLow();
}//end of send a zero


void Manchester::sendOne(void)
{
	//Serial.print('1');
	if (reverseManchester)
		sendHighLow();
	else
		sendLowHigh();
}

void Manchester::sendLowHigh()
{
	delayMicroseconds(delay1);
	digitalWrite(TxPin, LOW);

	delayMicroseconds(delay2);
	digitalWrite(TxPin, HIGH);
}

void Manchester::sendHighLow()
{
	delayMicroseconds(delay1);
	digitalWrite(TxPin, HIGH);

	delayMicroseconds(delay2);
	digitalWrite(TxPin, LOW);
}

Manchester man;
