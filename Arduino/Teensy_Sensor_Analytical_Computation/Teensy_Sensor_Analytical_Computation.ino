#include <OSCBundle.h>
#include <OSCTiming.h>
#include <IntervalTimer.h>
#include "FilterFIR.h"
#include "fdacoefs.h"


#ifdef BOARD_HAS_USB_SERIAL
#include <SLIPEncodedUSBSerial.h>
SLIPEncodedUSBSerial SLIPSerial( thisBoardsSerialUSB );
#else
#include <SLIPEncodedSerial.h>
 SLIPEncodedSerial SLIPSerial(Serial);
#endif


// Declaration of constants
const int NUMSENSORS = 4;
const int ADCRESOLUTION = 10;
const int R = 7830;

// Timer variables
IntervalTimer timer;
volatile boolean isADCReady = false;

// Global variables
int dcComponent[NUMSENSORS + 1];

// Interrupt timer function
void timerCallback() {
  isADCReady = true;
}

FIR<BL> fir;

void setup()
{                
  //begin SLIPSerial just like Serial
    SLIPSerial.begin(38400);   // set this as high as you can reliably run on your platform
#if ARDUINO >= 100
    while(!Serial)
      ;   // Leonardo bug
#endif
  
  // ADC Configuration
  analogReadRes(ADCRESOLUTION);
  analogReadAveraging(1); // For smoothing the input --> 4 by default
  analogReference(DEFAULT);
  
  // Calibration
  analogRead(0);
  
  // Get the DC Level
  for(int j = 0; j<NUMSENSORS + 1; j++)
  {
    dcComponent[j] = 0; 
  }
  
  for(int i = 0; i<4000; i++)
  {
    for(int j = 0; j<NUMSENSORS + 1; j++)
    {
      dcComponent[j] += analogRead(j); 
    }
  }
  
  for(int j = 0; j<NUMSENSORS + 1; j = j++)
  {
    dcComponent[j] /= 4000; 
    
    if(j%2==1)
      dcComponent[j] = pow(2, ADCRESOLUTION) - 1 - dcComponent[j];
  }
  
  
  // declare variables for coefficients
  // these should be calculated by hand, or using a tool
  // in case a phase linear filter is required, the coefficients are symmetric
  // for time optimization it seems best to enter symmetric values like below
  fir.setCoefficients(B);

    //declare gain coefficient to scale the output back to normal
  float gain = 1; // set to 1 and input unity to see what this needs to be
  fir.setGain(gain);
  
  // IntervalTimer setup
  timer.begin(timerCallback, 500);
}

void loop()                     
{
  isADCReady = false;
  // Initialization of variables
  OSCBundle bndl;
  uint64_t timetag[NUMSENSORS+1];
  int adcValue[NUMSENSORS+1];
  int32_t pressure[NUMSENSORS];
  int32_t error;
  
  
  if(isADCReady == true)
  {
    for(int i = 0; i<NUMSENSORS+1; i++)
    {
      if(i!=0)
        adcValue[i] = (int32_t) adcRead(i, &timetag[i]); //adcValue[i] = (int32_t) analogRead(i);
      else
        adcValue[0] = (int32_t) adcRead(0, &timetag[i]);
    }
    
    // Calculation of the variable resistor
    pressure[0] = (int32_t) ((float)(R*adcValue[1] - R*adcValue[0]) / (float)abs(adcValue[0] - dcComponent[0]));
    
    if(pow(2, ADCRESOLUTION) - 1  - adcValue[0] - adcValue[1] + dcComponent[0] - dcComponent[1] <= 0)
      pressure[1] = pow(2,32)-1;
    else
      pressure[1] = (int32_t) ((float)(R*adcValue[1] - R*adcValue[2]) / (float)abs(pow(2, ADCRESOLUTION) - 1  - adcValue[0] - adcValue[1] + dcComponent[0] - dcComponent[1]));
    
    if(pow(2, ADCRESOLUTION) - 1  - adcValue[3] - adcValue[4] + dcComponent[4] - dcComponent[3] <= 0)
      pressure[2] = pow(2,32)-1;
    else
       pressure[2] = (int32_t) ((float)(R*adcValue[3] - R*adcValue[2]) / (float)(pow(2, ADCRESOLUTION) - 1  - adcValue[3] - adcValue[4] + dcComponent[4] - dcComponent[3]));
       
    pressure[3] = (int32_t) ((float)(R*adcValue[3] - R*adcValue[4]) / (float)abs(adcValue[4] - dcComponent[4]));
    
    
    // Computer the error
    // The following expression should be equal to zero if the calculation is correct
    error = 2 * (pow(2, ADCRESOLUTION)-1) - adcValue[0] - adcValue[1] - adcValue[2] - adcValue[3] - adcValue[4] + dcComponent[0] + dcComponent[2] + dcComponent[4];
     
    for(int i = 0; i < NUMSENSORS; i++)
    {
      if(i!=0)
      {
        bndl.getOSCMessage("/i")->add(pressure[i]);
      }
      else
      {
        bndl.add("/i").add(pressure[i]);
      }
    }
    
    // bndl.add("/s").add(analogRead(5));
    bndl.add("/e").add(error);
    bndl.add("/t").add(timetag[0]).add(timetag[4]);
    
    // Send the OSC Bundle through the Serial Port
    SLIPSerial.beginPacket();
          bndl.send(SLIPSerial); // send the bytes to the SLIP stream
      SLIPSerial.endPacket(); // mark the end of the OSC Packet
      bndl.empty(); // empty the bundle to free room for a new one
      
  }
  

}



