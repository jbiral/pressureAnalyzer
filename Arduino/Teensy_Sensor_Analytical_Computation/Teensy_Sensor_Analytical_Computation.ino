#include <OSCBundle.h>
#include <OSCTiming.h>
#include "IntervalTimer.h"
//#include "fdacoefs.h"

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

const int FILTERTAPS = 1001;
const int32_t B[1001] = {
       2084416,    -1527196,    -1447969,    -1566026,    -1790854,    -2061956,
      -2335825,    -2579315,    -2766599,    -2878017,    -2899956,    -2825075,
      -2652536,    -2388030,    -2043463,    -1636214,    -1188048,     -723661,
       -268970,      150678,      512348,      797103,      991454,     1088267,
       1087311,      995209,      824892,      594596,      326366,       44369,
       -227074,     -465349,     -651361,     -770993,     -816177,     -785441,
       -683890,     -522665,     -317864,      -89044,      142536,      355785,
        531654,      654773,      714588,      707171,      632001,      501798,
        320670,      115326,      -99947,     -306513,     -481920,     -609337,
       -678780,     -684887,     -626779,     -509312,     -343387,     -144812,
         67530,      273615,      454127,      592124,      674561,      693529,
        646992,      539070,      379626,      183443,      -31195,     -244180,
       -435441,     -586843,     -683850,     -716973,     -682624,     -583547,
       -428563,     -231819,      -11496,      211860,      417243,      585153,
        699432,      748813,      728039,      638401,      487669,      289433,
         61947,     -173850,     -395372,     -582646,     -715990,     -784542,
       -779125,     -700314,     -555292,     -355912,     -120501,      128536,
        368226,      576368,      732783,      821789,      834354,      768760,
        630350,      431092,      188816,      -74228,     -333437,     -564337,
       -744882,     -857578,     -891053,     -841211,     -711717,     -513737,
       -265044,       11632,      290610,      545722,      752710,      891549,
        948364,      916809,      798764,      604245,      350576,       60839,
       -238215,     -518621,     -753824,     -921182,    -1004161,     -993836,
       -890212,     -701481,     -444940,     -142955,      175737,      482124,
        747141,      945031,     1056871,     1071004,      984579,      804690,
        547195,      235043,     -103219,     -435958,     -731825,     -962766,
      -1106338,    -1147835,    -1082029,     -913834,     -657877,     -337039,
         19482,      378733,      707004,      973016,     1151023,     1223229,
       1181594,     1028587,      777169,      449670,       75854,     -309780,
       -671180,     -974118,    -1189440,    -1295846,    -1281991,    -1147668,
       -903969,     -572391,     -182940,      228528,      623781,      965338,
       1220783,     1364638,     1382513,     1270685,     1038277,      705632,
        302456,     -134053,     -563372,     -945306,    -1243273,    -1428203,
      -1481496,    -1396490,    -1179303,     -848707,     -434413,       25816,
        489458,      912967,     1256037,     1485488,     1578445,     1524449,
       1326658,     1001847,      578943,       96455,     -401144,     -867380,
      -1258122,    -1535693,    -1672590,    -1654207,    -1480348,    -1165390,
       -737105,     -234249,      296943,      806975,     1247699,     1576867,
       1762171,     1784429,     1639453,     1338845,      908674,      387925,
       -175964,     -730306,    -1223178,    -1607460,    -1845609,    -1913601,
      -1802761,    -1521203,    -1093310,     -557299,       37920,      637111,
       1184105,     1626958,     1922709,     2041657,     1970444,     1713361,
       1292067,      744018,      119087,     -525038,    -1128179,    -1633199,
      -1991341,    -2167103,    -2141643,    -1914798,    -1505220,     -948990,
       -296446,      392356,     1053200,     1623703,     2049135,     2287634,
       2314327,     2123936,     1731522,     1171304,      493697,     -239228,
       -959352,    -1598938,    -2096737,    -2404262,    -2489910,    -2342899,
      -1973778,    -1414368,     -714672,       61570,      842341,     1554432,
       2130068,     2513256,     2665463,     2569307,     2230390,     1677286,
        959088,      140974,     -701580,    -1489698,    -2148580,    -2614631,
      -2841378,    -2804186,    -2502937,    -1962302,    -1229751,     -371385,
        533842,     1401530,     2149628,     2706198,     3016215,     3047047,
       2791863,     2270609,     1528490,      632077,     -336603,    -1287536,
      -2131074,    -2786397,    -3189224,    -3298022,    -3098282,    -2604232,
      -1858081,     -926177,      106474,     1144446,     2090123,     2853333,
       3359568,     3557347,     3423523,     2965596,     2221760,     1257663,
        160526,     -968504,    -2023731,    -2904847,    -3526286,    -3825559,
      -3769542,    -3357953,    -2623893,    -1631298,     -469483,      755025,
       1928084,     2938491,     3688613,     4103701,     4139324,     3785887,
       3070297,     2053930,      827437,     -497320,    -1797340,    -2949875,
      -3844012,    -4391264,    -4534235,    -4252539,    -3565377,    -2530410,
      -1238995,      191822,     1629879,     2939956,     3996512,     4695511,
       4964868,     4771375,     4124939,     3078045,     1721924,      178536,
      -1410348,    -2895934,    -4136484,    -5010556,    -5428825,    -5343213,
      -4752027,    -3701078,    -2280388,     -616468,     1138813,     2821933,
       4272907,     5350342,     5945109,     5991178,     5472846,     4427151,
       2941430,     1146351,     -795590,    -2704558,    -4400008,    -5717431,
      -6524293,    -6733163,    -6310915,    -5282990,    -3732383,    -1793196,
        360823,     2531676,     4515098,     6119555,     7184572,     7596529,
       7300429,     6306532,     4690948,     2590269,      190110,    -2290796,
      -4620609,    -6575418,    -7960688,    -8630151,    -8500882,    -7562858,
      -5881739,    -3595007,     -901012,     1957862,     4716524,     7111475,
       8905501,     9910568,    10007042,     9156959,     7410090,     4902152,
       1844831,    -1491477,    -4801077,    -7770274,   -10106416,   -11566055,
     -11979188,   -11267961,    -9457162,    -6675806,    -3148881,      819801,
       4872847,     8630447,    11724785,    13834651,    14717219,    14233982,
      12368112,     9231371,     5059616,      196790,    -4931734,    -9854475,
     -14094879,   -17214948,   -18857302,   -18782364,   -16896388,   -13268406,
      -8132962,    -1878707,     4977512,    11829348,    18029610,    22945541,
      26015866,    26805419,    25052384,    20703702,    13935520,     5155929,
      -5011023,   -15759371,   -26151864,   -35183776,   -41855296,   -45247590,
     -44596459,   -39358228,   -29262289,   -14345954,     5031308,    28196628,
      54194579,    81839371,   109784410,   136605084,   160888652,   181325301,
     196793755,   206435210,   209710326,   206435210,   196793755,   181325301,
     160888652,   136605084,   109784410,    81839371,    54194579,    28196628,
       5031308,   -14345954,   -29262289,   -39358228,   -44596459,   -45247590,
     -41855296,   -35183776,   -26151864,   -15759371,    -5011023,     5155929,
      13935520,    20703702,    25052384,    26805419,    26015866,    22945541,
      18029610,    11829348,     4977512,    -1878707,    -8132962,   -13268406,
     -16896388,   -18782364,   -18857302,   -17214948,   -14094879,    -9854475,
      -4931734,      196790,     5059616,     9231371,    12368112,    14233982,
      14717219,    13834651,    11724785,     8630447,     4872847,      819801,
      -3148881,    -6675806,    -9457162,   -11267961,   -11979188,   -11566055,
     -10106416,    -7770274,    -4801077,    -1491477,     1844831,     4902152,
       7410090,     9156959,    10007042,     9910568,     8905501,     7111475,
       4716524,     1957862,     -901012,    -3595007,    -5881739,    -7562858,
      -8500882,    -8630151,    -7960688,    -6575418,    -4620609,    -2290796,
        190110,     2590269,     4690948,     6306532,     7300429,     7596529,
       7184572,     6119555,     4515098,     2531676,      360823,    -1793196,
      -3732383,    -5282990,    -6310915,    -6733163,    -6524293,    -5717431,
      -4400008,    -2704558,     -795590,     1146351,     2941430,     4427151,
       5472846,     5991178,     5945109,     5350342,     4272907,     2821933,
       1138813,     -616468,    -2280388,    -3701078,    -4752027,    -5343213,
      -5428825,    -5010556,    -4136484,    -2895934,    -1410348,      178536,
       1721924,     3078045,     4124939,     4771375,     4964868,     4695511,
       3996512,     2939956,     1629879,      191822,    -1238995,    -2530410,
      -3565377,    -4252539,    -4534235,    -4391264,    -3844012,    -2949875,
      -1797340,     -497320,      827437,     2053930,     3070297,     3785887,
       4139324,     4103701,     3688613,     2938491,     1928084,      755025,
       -469483,    -1631298,    -2623893,    -3357953,    -3769542,    -3825559,
      -3526286,    -2904847,    -2023731,     -968504,      160526,     1257663,
       2221760,     2965596,     3423523,     3557347,     3359568,     2853333,
       2090123,     1144446,      106474,     -926177,    -1858081,    -2604232,
      -3098282,    -3298022,    -3189224,    -2786397,    -2131074,    -1287536,
       -336603,      632077,     1528490,     2270609,     2791863,     3047047,
       3016215,     2706198,     2149628,     1401530,      533842,     -371385,
      -1229751,    -1962302,    -2502937,    -2804186,    -2841378,    -2614631,
      -2148580,    -1489698,     -701580,      140974,      959088,     1677286,
       2230390,     2569307,     2665463,     2513256,     2130068,     1554432,
        842341,       61570,     -714672,    -1414368,    -1973778,    -2342899,
      -2489910,    -2404262,    -2096737,    -1598938,     -959352,     -239228,
        493697,     1171304,     1731522,     2123936,     2314327,     2287634,
       2049135,     1623703,     1053200,      392356,     -296446,     -948990,
      -1505220,    -1914798,    -2141643,    -2167103,    -1991341,    -1633199,
      -1128179,     -525038,      119087,      744018,     1292067,     1713361,
       1970444,     2041657,     1922709,     1626958,     1184105,      637111,
         37920,     -557299,    -1093310,    -1521203,    -1802761,    -1913601,
      -1845609,    -1607460,    -1223178,     -730306,     -175964,      387925,
        908674,     1338845,     1639453,     1784429,     1762171,     1576867,
       1247699,      806975,      296943,     -234249,     -737105,    -1165390,
      -1480348,    -1654207,    -1672590,    -1535693,    -1258122,     -867380,
       -401144,       96455,      578943,     1001847,     1326658,     1524449,
       1578445,     1485488,     1256037,      912967,      489458,       25816,
       -434413,     -848707,    -1179303,    -1396490,    -1481496,    -1428203,
      -1243273,     -945306,     -563372,     -134053,      302456,      705632,
       1038277,     1270685,     1382513,     1364638,     1220783,      965338,
        623781,      228528,     -182940,     -572391,     -903969,    -1147668,
      -1281991,    -1295846,    -1189440,     -974118,     -671180,     -309780,
         75854,      449670,      777169,     1028587,     1181594,     1223229,
       1151023,      973016,      707004,      378733,       19482,     -337039,
       -657877,     -913834,    -1082029,    -1147835,    -1106338,     -962766,
       -731825,     -435958,     -103219,      235043,      547195,      804690,
        984579,     1071004,     1056871,      945031,      747141,      482124,
        175737,     -142955,     -444940,     -701481,     -890212,     -993836,
      -1004161,     -921182,     -753824,     -518621,     -238215,       60839,
        350576,      604245,      798764,      916809,      948364,      891549,
        752710,      545722,      290610,       11632,     -265044,     -513737,
       -711717,     -841211,     -891053,     -857578,     -744882,     -564337,
       -333437,      -74228,      188816,      431092,      630350,      768760,
        834354,      821789,      732783,      576368,      368226,      128536,
       -120501,     -355912,     -555292,     -700314,     -779125,     -784542,
       -715990,     -582646,     -395372,     -173850,       61947,      289433,
        487669,      638401,      728039,      748813,      699432,      585153,
        417243,      211860,      -11496,     -231819,     -428563,     -583547,
       -682624,     -716973,     -683850,     -586843,     -435441,     -244180,
        -31195,      183443,      379626,      539070,      646992,      693529,
        674561,      592124,      454127,      273615,       67530,     -144812,
       -343387,     -509312,     -626779,     -684887,     -678780,     -609337,
       -481920,     -306513,      -99947,      115326,      320670,      501798,
        632001,      707171,      714588,      654773,      531654,      355785,
        142536,      -89044,     -317864,     -522665,     -683890,     -785441,
       -816177,     -770993,     -651361,     -465349,     -227074,       44369,
        326366,      594596,      824892,      995209,     1087311,     1088267,
        991454,      797103,      512348,      150678,     -268970,     -723661,
      -1188048,    -1636214,    -2043463,    -2388030,    -2652536,    -2825075,
      -2899956,    -2878017,    -2766599,    -2579315,    -2335825,    -2061956,
      -1790854,    -1566026,    -1447969,    -1527196,     2084416
};


template<int filterTaps>
class FIR {
      public:
            //construct without coefs
            FIR() {
                  k = 0; //initialize so that we start to read at index 0
                  for (int i=0; i<filterTaps; i++) {      
                        values[i] = 0; // to have a nice start up, fill the array with 0's
                  }
                  //TODO calculate default gain?
                  //TODO calculate default coefs?
            }
            //construct with coefs
            FIR(float newGain, float *newCoefs) {
                  k = 0; //initialize so that we start to read at index 0
                  setGain(newGain);
                  for (int i=0; i<filterTaps; i++) {      
                        values[i] = 0; // to have a nice start up, fill the array with 0's
                  }
                  setCoefficients(newCoefs);
            }
            
            void setGain(float newGain) {gain = newGain;}

            void setCoefficients(const int32_t *newCoefs) {
                  for (int i=0; i<filterTaps; i++) {      
                        coef[i] = newCoefs[i];
                  }
            }
            //set coefficient at specified index
            void setCoefficient(int idx, float newCoef) { coef[idx] = newCoef; }
            
            float process(int16_t in) {
                  int32_t out = 0;                        // out is the return variable. It is set to 0 every time we call the filter!

                  values[k] = in;                        // store the input of the routine (contents of the 'in' variable) in the array at the current pointer position

                  for (int i=0; i<filterTaps; i++) {                              // we step through the array
                        out += (int32_t) coef[i] * (int32_t) values[(i + k) % filterTaps];      // ... and add and multiply each value to accumulate the output
                                                                                                //  (i + k) % filterTaps creates a cyclic way of getting through the array
                  }
                  
                  out /= gain;                        // We need to scale the output (unless the coefficients provide unity gain in the passband)

                  k = (k+1) % filterTaps;            // k is increased and wraps around the filterTaps, so next time we will overwrite the oldest saved sample in the array

                  return out;                              // we send the output value back to whoever called the routine
            }
            
      private:
            int16_t values[filterTaps];
            
            int16_t coef[filterTaps];
            
            //declare gain coefficient to scale the output back to normal
            float gain; // set to 1 and input unity to see what this needs to be
            
            int k; // k stores the index of the current array read to create a circular memory through the array
};
/*
  LIBRARY CODE ENDS HERE
*/

FIR<FILTERTAPS> fir;

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
      
      isADCReady = false;
  }
  

}



