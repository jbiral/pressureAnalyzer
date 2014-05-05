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

const int DELAYTIME = 1; //in ms
const int NUMSENSORS = 4;
const int SIZEBUF = 5;

void setup()
{                
  //begin SLIPSerial just like Serial
    SLIPSerial.begin(38400);   // set this as high as you can reliably run on your platform
#if ARDUINO >= 100
    while(!Serial)
      ;   // Leonardo bug
#endif
  
  // ADC Configuration
  analogReadRes(10);
  analogReadAveraging(4); // For smoothing the input --> 4 by default
  analogReference(DEFAULT);
  
  // Initialization of pins --> Digital pins 0 & 1 are for the Serial port
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  
  // Reference ground
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
}

void loop()                     
{
  // Initialization of variables
  OSCBundle bndl;
  uint64_t timetag;
  int values[NUMSENSORS];
  
  // Time 1
  // D2 A0 A1 A2 D3 || D4 D5 D6
  // GD ME HI HI HI || PU HI HI
  pinMode(6, INPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  pinMode(3, INPUT);
  
  delay(DELAYTIME);
  values[0] = adcRead(0, &timetag);
  

  
  // Time 2
  // D2 A0 A1 A2 D3 || D4 D5 D6
  // HI GD ME HI HI || HI PU HI
  pinMode(4, INPUT);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  pinMode(A0, OUTPUT);
  digitalWrite(A0, LOW);
  pinMode(2, INPUT);
  
  delay(DELAYTIME);
  values[1] = analogRead(1);
  
  // Time 3
  // D2 A0 A1 A2 D3 || D4 D5 D6
  // HI HI ME GD HI || HI PU HI
  pinMode(A0, INPUT);
  pinMode(A2, OUTPUT);
  digitalWrite(A2, LOW);
  
  delay(DELAYTIME);
  values[2] = analogRead(1);
  
  // Time 4
  // D2 A0 A1 A2 D3 || D4 D5 D6
  // HI HI HI ME GD || HI HI PU
  pinMode(5, INPUT);
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
  pinMode(A2, INPUT);
  
  pinMode(3, OUTPUT); // Ground pin
  digitalWrite(3, LOW); // Ground pin
  
  delay(DELAYTIME);
  values[3] = analogRead(2);
  
  
  bndl.add("/i").add(values[0]).add(values[1]).add(values[2]).add(values[3]);
  bndl.add("/t").add(timetag);
  
  // Send the OSC Bundle through the Serial Port
  SLIPSerial.beginPacket();
        bndl.send(SLIPSerial); // send the bytes to the SLIP stream
    SLIPSerial.endPacket(); // mark the end of the OSC Packet
    bndl.empty(); // empty the bundle to free room for a new one

}


