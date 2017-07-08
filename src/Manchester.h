#ifndef MANCHESTER_h
#define MANCHESTER_h

//timer scaling factors for different transmission speeds
#define MAN_300 0
#define MAN_600 1
#define MAN_1200 2
#define MAN_2400 3
#define MAN_4800 4
#define MAN_9600 5
#define MAN_19200 6
#define MAN_38400 7

#define     SYNC_PULSE_DEF  9

//setup timing for receiver
#define MinCount        33  //pulse lower count limit on capture
#define MaxCount        65  //pulse higher count limit on capture
#define MinLongCount    66  //pulse lower count on double pulse
#define MaxLongCount    129 //pulse higher count on double pulse

//setup timing for transmitter
#define HALF_BIT_INTERVAL 3072 //(=48 * 1024 * 1000000 / 16000000Hz) microseconds for speed factor 0 (300baud)

#define TimeOutDefault -1 //the timeout in msec default blocks

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include <pins_arduino.h>
#endif

class Manchester
{
  public:
    Manchester(); //the constructor
    void setTxPin(uint8_t pin); //set the arduino digital pin for transmit.

    void workAround1MhzTinyCore(uint8_t a = 1); //apply workaround for defect in tiny Core library for 1Mhz
    void setupTransmit(uint8_t pin, uint8_t SF = MAN_1200); //set up transmission

    void transmitEM4100(uint8_t *data); // transmit array of bytes
    void SetDelay_us(uint16_t delay_us);

    //wrappers for global functions
  private:
    uint8_t TxPin;
    uint8_t speedFactor;
    uint16_t delay1;
    uint16_t delay2;
    void sendZero(void);
    void sendOne(void);
    uint8_t applyWorkAround1Mhz;
    bool reverseManchester;
    void sendEvenParity(int numHighBits);
    void sendLowHigh();
    void sendHighLow();
};//end of class Manchester


extern Manchester man;

#endif
