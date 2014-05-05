#include <OSCBundle.h>
#include <OSCBoards.h>
#include <OSCTiming.h>

#ifdef BOARD_HAS_USB_SERIAL
#include <SLIPEncodedUSBSerial.h>
SLIPEncodedUSBSerial SLIPSerial( thisBoardsSerialUSB );
#else
#include <SLIPEncodedSerial.h>
SLIPEncodedSerial SLIPSerial(Serial);
#endif

void setup()
{                
  //begin SLIPSerial just like Serial
  SLIPSerial.begin(38400);   // set this as high as you can reliably run on your platform
#if ARDUINO >= 100
  while(!Serial)
    ;   // Leonardo bug
#endif

  // Initialization of pins
  pinMode(A0, INPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  // Resolution of the ADC
  analogReadRes(10);
  //analogReadAveraging(4); // For smoothing the input --> 4 by default
  //analogReference(DEFAULT);
}

void loop()                     
{

  OSCBundle bndl;
  uint64_t timetag;
  int32_t analogValue[3];

  // Time 1
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  analogValue[0] = (int32_t)adcRead(0, &timetag);

  // Time 2
  digitalWrite(3, HIGH);
  analogValue[1] = (int32_t)analogRead(0);

  // Time 3
  digitalWrite(2, HIGH);
  analogValue[2] = (int32_t)analogRead(0);

//  bndl.add("/analog/0").add(analogValue[0]);
//  bndl.add("/analog/1").add(analogValue[1]);
//  bndl.add("/analog/2").add(analogValue[2]);
  bndl.add("/analog/time/0").add(timetag);


  // Send the bundle
  SLIPSerial.beginPacket();
  bndl.send(SLIPSerial); // send the bytes to the SLIP stream
  SLIPSerial.endPacket(); // mark the end of the OSC Packet
  bndl.empty(); // empty the bundle to free room for a new one
}


