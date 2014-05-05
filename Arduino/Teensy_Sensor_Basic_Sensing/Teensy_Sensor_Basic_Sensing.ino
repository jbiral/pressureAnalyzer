
/*
    Make an OSC bundle and send it over SLIP serial

    OSCBundles allow OSCMessages to be grouped together to  preserve the order and completeness of related messages.
    They also allow for timetags to be carried to represent the presentation time of the messages.
*/
#include <OSCBundle.h>
#include <OSCBoards.h>

#ifdef BOARD_HAS_USB_SERIAL
#include <SLIPEncodedUSBSerial.h>
SLIPEncodedUSBSerial SLIPSerial( thisBoardsSerialUSB );
#else
#include <SLIPEncodedSerial.h>
 SLIPEncodedSerial SLIPSerial(Serial);
#endif


void setup() {
  //begin SLIPSerial just like Serial
    SLIPSerial.begin(38400);   // set this as high as you can reliably run on your platform
#if ARDUINO >= 100
    while(!Serial)
      ;   // Leonardo bug
#endif

}

void loop(){
    //declare the bundle
    OSCBundle bndl;
    //BOSCBundle's add' returns the OSCMessage so the message's 'add' can be composed together

    
    bndl.add("/analog/0").add((int32_t)analogRead(0));
    bndl.add("/analog/1").add((int32_t)analogRead(1));
    bndl.add("/analog/2").add((int32_t)analogRead(2));
    bndl.add("/analog/3").add((int32_t)analogRead(3));
    bndl.add("/analog/4").add((int32_t)analogRead(4));
    bndl.add("/analog/5").add((int32_t)analogRead(5));
    bndl.add("/analog/6").add((int32_t)analogRead(6));
    bndl.add("/analog/7").add((int32_t)analogRead(7));
    bndl.add("/analog/8").add((int32_t)analogRead(8));
    bndl.add("/analog/9").add((int32_t)analogRead(9));
    bndl.add("/analog/10").add((int32_t)analogRead(10));
    bndl.add("/analog/11").add((int32_t)analogRead(11));

    SLIPSerial.beginPacket();
        bndl.send(SLIPSerial); // send the bytes to the SLIP stream
    SLIPSerial.endPacket(); // mark the end of the OSC Packet
    bndl.empty(); // empty the bundle to free room for a new one

}
