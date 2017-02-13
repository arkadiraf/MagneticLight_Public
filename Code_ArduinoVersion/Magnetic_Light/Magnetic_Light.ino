
/* Pinout:
    Arduino Nano 5V: 
    D9-->CE NRF
    D10-->CSN NRF
    D13 (SCK)-->SCK NRF
    D11 (MOSI)-->MOSI NRF
    D12 (MISO)-->MISO NRF
    D3-->IRQ
*/

#include <Wire.h>
#include <HMC5883L.h>
#include <Adafruit_NeoPixel.h>

HMC5883L compass;
float Mag_Bias[3]={-162,-505,464}; // Bias to vector size // change later to HPF filter 
//float Mag_Bias[3]={0,0,0};
float Mag[3]={0,0,0}; //Magnetometer

float MagOut[3]={0,0,0}; //Magnetometer
float LMagOut[3]={0,0,0}; //Last Magnetometer reading
float MagIn[3]={0,0,0}; //Magnetometer
float LMagIn[3]={0,0,0}; //Last Magnetometer reading
float AHPF=0.99; 


float Mag_Norm[3]={0,0,0}; //Magnetometer normalized
float Mag_ABS=0; // Vector size indication how far the magnet is
float CosAngle=0; // cosin of the angle between the two vectors
float LedPower=0; // variable to store LED Power
byte LedPower_Byte=0; // Byte varible to set led power

#define SAMPLEDELAY 30 // roughly 30 hz
unsigned long sampleMillis=0;
unsigned long timeMillis=0;
////////////////
// Neo Pixels //
////////////////
//#define NEOPIXELS 8
#define NEOPIXELS 74
#define NEODATAPIN 4
// neo Pixel initialize
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXELS, NEODATAPIN, NEO_GRB + NEO_KHZ800);

// define array of pixel vectors based on the compas:
float Pixel_Vect_Float[3]={0,0,0}; // variable to store LED Power
const int Pixel_Vect[NEOPIXELS][3]={ 
{  -41,  -86,  -31 }, 
{  -54,  -71,  -45 }, 
{  -66,  -47,  -58 }, 
{  -75,  -18,  -63 }, 
{  -76,  17,  -63 }, 
{  -68,  44,  -58 }, 
{  -56,  67,  -48 }, 
{  -41,  86,  -32 }, 
{  -35,  94,  -2 }, 
{  -61,  79,  -4 }, 
{  -82,  57,  -4 }, 
{  -99,  10,  -10 }, 
{  -97,  -20,  -6 }, 
{  -87,  -47,  -6 }, 
{  -68,  -72,  -7 }, 
{  -24,  -97,  -3 }, 
{  -50,  -81,  30 }, 
{  -66,  -63,  41 }, 
{  -80,  -37,  49 }, 
{  -85,  0,  53 }, 
{  -80,  32,  51 }, 
{  -71,  56,  42 }, 
{  -50,  79,  33 }, 
{  -30,  94,  19 }, 
{  -20,  94,  29 }, 
{  -25,  81,  54 }, 
{  -34,  54,  78 }, 
{  -36,  24,  90 }, 
{  -36,  -15,  92 }, 
{  -34,  -48,  81 }, 
{  -25,  -69,  68 }, 
{  -23,  -83,  54 }, 
{  22,  -86,  46 }, 
{  35,  -72,  60 }, 
{  44,  -38,  80 }, 
{  53,  -8,  86 }, 
{  45,  31,  83 }, 
{  35,  62,  70 }, 
{  24,  84,  50 }, 
{  10,  96,  24 }, 
{  42,  91,  12 }, 
{  68,  71,  19 }, 
{  85,  44,  24 }, 
{  95,  8,  28 }, 
{  91,  -32,  28 }, 
{  80,  -55,  25 }, 
{  62,  -75,  20 }, 
{  50,  -84,  21 }, 
{  48,  -86,  -19 }, 
{  67,  -70,  -25 }, 
{  83,  -51,  -26 }, 
{  93,  -21,  -31 }, 
{  94,  13,  -30 }, 
{  85,  43,  -29 }, 
{  67,  70,  -25 }, 
{  42,  90,  -16 }, 
{  19,  91,  -35 }, 
{  38,  72,  -58 }, 
{  48,  45,  -74 }, 
{  55,  20,  -81 }, 
{  55,  -20,  -81 }, 
{  47,  -54,  -70 }, 
{  31,  -76,  -56 }, 
{  20,  -87,  -43 }, 
{  -10,  -88,  -45 }, 
{  -14,  -73,  -67 }, 
{  -13,  -50,  -85 }, 
{  -17,  -14,  -98 }, 
{  -12,  17,  -98 }, 
{  -14,  50,  -86 }, 
{  -13,  72,  -68 }, 
{  -9,  89,  -47 }, 
{  -12,  99,  -10 }, 
{  -12,  99,  -10 } 
}; 


void setup()
{
  Serial.begin(57600);

  // Initialize HMC5883L
  Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
 
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
 
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_1);

  // Magnetometer set offset:
  compass.setOffset(0,0);
  
  // Check settings
  checkSettings();

  // neo Pixel initialize
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // clear strip
  strip.setBrightness(255);
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));  
  }
  strip.show();
} // end setup


void loop()
{
      timeMillis=millis();
   if (((timeMillis-sampleMillis)>SAMPLEDELAY)||(sampleMillis>timeMillis)){ // overflow of millis, sample time
      sampleMillis=timeMillis;
      // Read Mag sensor
      Vector raw = compass.readRaw();
 
      /////////////////
      // HPF filter: //
      /////////////////
      LMagIn[0]=MagIn[0];
      LMagIn[1]=MagIn[1];
      LMagIn[2]=MagIn[2];
      LMagOut[0]=MagOut[0];
      LMagOut[1]=MagOut[1];
      LMagOut[2]=MagOut[2];
      // update reading
      MagIn[0]=raw.XAxis;
      MagIn[1]=raw.YAxis;
      MagIn[2]=raw.ZAxis;
      // update filter
      MagOut[0]=AHPF*(LMagOut[0]+MagIn[0]-LMagIn[0]);
      MagOut[1]=AHPF*(LMagOut[1]+MagIn[1]-LMagIn[1]);
      MagOut[2]=AHPF*(LMagOut[2]+MagIn[2]-LMagIn[2]);

      // Normalize vector and calculate ABS value
      Mag_ABS=sqrt(MagOut[0]*MagOut[0]+MagOut[1]*MagOut[1]+MagOut[2]*MagOut[2]);
      Mag_Norm[0]=MagOut[0]/Mag_ABS;
      Mag_Norm[1]=MagOut[1]/Mag_ABS;
      Mag_Norm[2]=MagOut[2]/Mag_ABS;

      
        //////////////////////////// 
//      // bias samples and scale //
//      ////////////////////////////
//      Mag[0]=raw.XAxis-Mag_Bias[0];
//      Mag[1]=raw.YAxis-Mag_Bias[1];
//      Mag[2]=raw.ZAxis-Mag_Bias[2];

//      Mag_ABS=sqrt(Mag[0]*Mag[0]+Mag[1]*Mag[1]+Mag[2]*Mag[2]);
//      Mag_Norm[0]=Mag[0]/Mag_ABS;
//      Mag_Norm[1]=Mag[1]/Mag_ABS;
//      Mag_Norm[2]=Mag[2]/Mag_ABS;

        // Calculate angle between magnetic vector and LED vectors
      for (uint16_t ii=0 ; ii<NEOPIXELS ; ii++){
        Pixel_Vect_Float[0]=((float)Pixel_Vect[ii][0])/100;
        Pixel_Vect_Float[1]=((float)Pixel_Vect[ii][1])/100;
        Pixel_Vect_Float[2]=((float)Pixel_Vect[ii][2])/100;
        CosAngle=Mag_Norm[0]*Pixel_Vect_Float[0] + Mag_Norm[1]*Pixel_Vect_Float[1] + Mag_Norm[2]*Pixel_Vect_Float[2];
        //LedPower=Mag_ABS*CosAngle*CosAngle*CosAngle*CosAngle*CosAngle;
        LedPower=Mag_ABS*pow(CosAngle,5);
        if (LedPower>=0){
          if (LedPower>255) LedPower=255; 
          LedPower_Byte=(byte)(LedPower);
          strip.setPixelColor(ii, strip.Color(LedPower_Byte, 0, 0)); 
        }
        if (LedPower<0){
          if (LedPower<-255) LedPower=-255;
          LedPower_Byte=(byte)(-LedPower);
          strip.setPixelColor(ii, strip.Color(0, 0, LedPower_Byte)); 
        }
      }
      strip.show();

//      // send to matlab/simulink:
//      Serial.print("MSG: ");
//      Serial.print(timeMillis);
//      Serial.print(",");
//      Serial.print((int)raw.XAxis);
//      Serial.print(",");
//      Serial.print((int)raw.YAxis);
//      Serial.print(",");
//      Serial.print((int)raw.ZAxis);
//      Serial.println();

      // send to screen some debug data
      Serial.print("MSG: ");
      Serial.print(timeMillis);
      Serial.print(" , ");
      Serial.print(MagOut[0]);
      Serial.print(" , ");
      Serial.print(MagOut[1]);
      Serial.print(" , ");
      Serial.print(MagOut[2]);
      Serial.print(" , ");
      Serial.print(CosAngle);
      Serial.println();
   }
} // end main


// Magnetometer settings print
void checkSettings()
{
  Serial.print("Selected range: ");
  
  switch (compass.getRange())
  {
    case HMC5883L_RANGE_0_88GA: Serial.println("0.88 Ga"); break;
    case HMC5883L_RANGE_1_3GA:  Serial.println("1.3 Ga"); break;
    case HMC5883L_RANGE_1_9GA:  Serial.println("1.9 Ga"); break;
    case HMC5883L_RANGE_2_5GA:  Serial.println("2.5 Ga"); break;
    case HMC5883L_RANGE_4GA:    Serial.println("4 Ga"); break;
    case HMC5883L_RANGE_4_7GA:  Serial.println("4.7 Ga"); break;
    case HMC5883L_RANGE_5_6GA:  Serial.println("5.6 Ga"); break;
    case HMC5883L_RANGE_8_1GA:  Serial.println("8.1 Ga"); break;
    default: Serial.println("Bad range!");
  }
  
  Serial.print("Selected Measurement Mode: ");
  switch (compass.getMeasurementMode())
  {  
    case HMC5883L_IDLE: Serial.println("Idle mode"); break;
    case HMC5883L_SINGLE:  Serial.println("Single-Measurement"); break;
    case HMC5883L_CONTINOUS:  Serial.println("Continuous-Measurement"); break;
    default: Serial.println("Bad mode!");
  }

  Serial.print("Selected Data Rate: ");
  switch (compass.getDataRate())
  {  
    case HMC5883L_DATARATE_0_75_HZ: Serial.println("0.75 Hz"); break;
    case HMC5883L_DATARATE_1_5HZ:  Serial.println("1.5 Hz"); break;
    case HMC5883L_DATARATE_3HZ:  Serial.println("3 Hz"); break;
    case HMC5883L_DATARATE_7_5HZ: Serial.println("7.5 Hz"); break;
    case HMC5883L_DATARATE_15HZ:  Serial.println("15 Hz"); break;
    case HMC5883L_DATARATE_30HZ: Serial.println("30 Hz"); break;
    case HMC5883L_DATARATE_75HZ:  Serial.println("75 Hz"); break;
    default: Serial.println("Bad data rate!");
  }
  
  Serial.print("Selected number of samples: ");
  switch (compass.getSamples())
  {  
    case HMC5883L_SAMPLES_1: Serial.println("1"); break;
    case HMC5883L_SAMPLES_2: Serial.println("2"); break;
    case HMC5883L_SAMPLES_4: Serial.println("4"); break;
    case HMC5883L_SAMPLES_8: Serial.println("8"); break;
    default: Serial.println("Bad number of samples!");
  }

}
