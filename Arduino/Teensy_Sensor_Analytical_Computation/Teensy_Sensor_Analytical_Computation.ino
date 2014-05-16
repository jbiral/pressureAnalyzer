#include <OSCBundle.h>
#include <OSCTiming.h>
#include <IntervalTimer.h>
#include <OSCBundle.h>
#include <OSCTiming.h>
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
const int R = pow(2,7);
long oldT;

// Timer variables
IntervalTimer timerADC;
IntervalTimer timerSerial;
volatile boolean isADCReady = false;

// Global variables
int dcComponent[NUMSENSORS + 1];
int counter = 0;
int adcSample = 0;
int16_t timetagSent;
uint16_t adcValue[NUMSENSORS+1];
int16_t temp;
FIR<BL> fir[NUMSENSORS + 1];

// Interrupt timer function
void timerCallback() {
  if(counter != 0)
  {
    temp = fir[adcSample].process((uint16_t)analogRead(adcSample));      
  }
  else
  {
    adcValue[adcSample] = fir[adcSample].process((uint16_t)analogRead(adcSample));     
  }
  

  adcSample = (adcSample + 1)%(NUMSENSORS+1);
  
  if(adcSample == 0)
  {
    counter = (counter + 1)%10;
    
    if(counter == 0)
    {
      timetagSent = micros() - oldT;
      oldT = micros();
      isADCReady = true;
    }
    
  }
}



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
  
  //Filter
  
  for(int i =0; i<NUMSENSORS+1;i++)
  {
    fir[i].setCoefficients(B);
    fir[i].setGain(1024.0/1111.0);
  }
  
  
  // Calibration
  analogRead(0);
  
  // Get the DC Level
  for(int j = 0; j<NUMSENSORS + 1; j++)
  {
    dcComponent[j] = 0; 
  }
  
  for(int i = 0; i<4000+(BL-1)/2; i++)
  {
    for(int j = 0; j<NUMSENSORS + 1; j++)
    {
      if(i>(BL-1)/2)
        dcComponent[j] += fir[j].process(analogRead(j)); 
      else
      {
        dcComponent[j] = fir[j].process(analogRead(j)); 
        dcComponent[j]=0;
      }
    }
  }
  
  for(int j = 0; j<NUMSENSORS + 1; j = j++)
  {
    dcComponent[j] /= 4000; 
    
    if(j%2==1)
      dcComponent[j] = pow(2, ADCRESOLUTION) - 1 - dcComponent[j];
  }
  
  // IntervalTimer setup
  timerADC.begin(timerCallback, 40);
}

void loop()                     
{
  
  if(isADCReady == true)
  {
    isADCReady = false;
    
    OSCBundle bndl;
    uint16_t pressure[NUMSENSORS];
    
    // Calculation of the variable resistor
    pressure[0] = (uint16_t) ((float)(R*adcValue[1] - R*adcValue[0]) / (float)abs(adcValue[0] - dcComponent[0]));
    if(pressure[0]>30000)
      pressure[0] = 30000;
    
    if(pow(2, ADCRESOLUTION) - 1  - adcValue[0] - adcValue[1] + dcComponent[0] - dcComponent[1] <= 0)
      pressure[1] = pow(2,16)-1;
    else
      pressure[1] = (uint16_t) ((float)(R*adcValue[1] - R*adcValue[2]) / (float)abs(pow(2, ADCRESOLUTION) - 1  - adcValue[0] - adcValue[1] + dcComponent[0] - dcComponent[1]));
    if(pressure[1]>30000)
      pressure[1] = 30000;
      
    if(pow(2, ADCRESOLUTION) - 1  - adcValue[3] - adcValue[4] + dcComponent[4] - dcComponent[3] <= 0)
      pressure[2] = pow(2,16)-1;
    else
       pressure[2] = (uint16_t) ((float)(R*adcValue[3] - R*adcValue[2]) / (float)(pow(2, ADCRESOLUTION) - 1  - adcValue[3] - adcValue[4] + dcComponent[4] - dcComponent[3]));
    if(pressure[2]>30000)
      pressure[2] = 30000;
      
    pressure[3] = (uint16_t) ((float)(R*adcValue[3] - R*adcValue[4]) / (float)abs(adcValue[4] - dcComponent[4]));
    if(pressure[3]>30000)
      pressure[3] = 30000;
    
    
    
    bndl.add("/i").add(pressure[0]).add(pressure[1]).add(pressure[2]).add(pressure[3]);
    bndl.add("/adc").add(adcValue[0]).add(adcValue[1]).add(adcValue[2]).add(adcValue[3]).add(adcValue[4]);
    bndl.add("/dc").add(dcComponent[0]).add(dcComponent[1]).add(dcComponent[2]).add(dcComponent[3]).add(dcComponent[4]);
    bndl.add("/f").add(1000000/timetagSent);
    // Send the OSC Bundle through the Serial Port
    SLIPSerial.beginPacket();
    bndl.send(SLIPSerial); // send the bytes to the SLIP stream
    SLIPSerial.endPacket(); // mark the end of the OSC Packet
    bndl.empty(); // empty the bundle to free room for a new one

  }
  

}



