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
const int R = 7830;
long oldT;

// Timer variables
IntervalTimer timerADC;
IntervalTimer timerSerial;
volatile boolean isADCReady = false;

// Global variables
int dcComponent[NUMSENSORS + 1];
int counter = 0;
int adcSample = 0;

// Interrupt timer function
void timerCallback() {
  isADCReady = true;
}

FIR<BL> fir[NUMSENSORS + 1];

void setup()
{                
  //begin SLIPSerial just like Serial
Serial.begin(38400);
  
  // ADC Configuration
  analogReadRes(ADCRESOLUTION);
  analogReadAveraging(1); // For smoothing the input --> 4 by default
  analogReference(DEFAULT);
  
  //Filter
  int gain = 1;
  for(int i =0; i<NUMSENSORS+1;i++)
  {
    fir[i].setCoefficients(B);
    fir[i].setGain(gain);
  }
  
  
  // Calibration
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
  
  // IntervalTimer setup
  timerADC.begin(timerCallback, 40);
}

void loop()                     
{
  
  if(isADCReady == true)
  {
    isADCReady = false;
    
    uint64_t timetag;
    int16_t adcValue[NUMSENSORS+1];
    

    if(adcSample != 0)
       adcValue[adcSample] = fir[adcSample].process((int16_t) analogRead(adcSample));      
    else
       adcValue[0] = fir[0].process((int16_t) adcRead(0, &timetag));
       
    
    
    
    adcSample = (adcSample + 1)%(NUMSENSORS+1);
    
    if(adcSample == 0)
      counter = (counter + 1)%10;
    
    if(counter == 0 && adcSample == 0)
    {
        long dur = micros() - oldT;
        oldT = micros();
        Serial.println(dur);    
    }
    
    
  }
  

}



