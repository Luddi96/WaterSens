//Cables: 19cm / 15cm

/**Includes**/
#include <Wire.h> //Needed for BME
#include <SPI.h> //Needed for NRF
#include <nRF24L01.h> //Needed for NRF
#include <RF24.h> //Needed for NRF
#include <Adafruit_Sensor.h> //Needed for BME
#include <Adafruit_BME280.h> //Needed for BME
#include "Adafruit_MPRLS.h" //Needed for MPRLS

/**Pin Assignment**/
#define P_NRF_CE 7 //CE of NRF Module
#define P_NRF_CSN 8 //CSN of NRF Module
#define P_UBAT_EN A3 //This pin enables UBAT measurement (approx. 200uA)
#define P_UBAT_MEAS A2 //Analog pin to read UBAT via divider
#define P_LED_USER 5 //Led for button illumination (approx. 20mA)
#define P_SOL 4 //Solenoid enable (approx. 400mA)
#define P_PUMP 3 //Pump enable (approx. 600mA)

#define T20TO6 1
#define T6TO13 2
#define T13TO20 3

/**Adress definition**/
#define A_BME_I2C 0x76
#define A_MPRLS_I2C 0x18
#define A_BROADC "0XXXX"
#define MODULE_ID 0xA2

/**Instance creation**/
RF24 radio(P_NRF_CE, P_NRF_CSN);
Adafruit_BME280 bme;
Adafruit_MPRLS mpr = Adafruit_MPRLS(-1,-1);

/**Global variables**/
uint8_t txAddr[6] = {A_BROADC};
uint8_t rxAddr[6] = {A_BROADC};
float volt = 0;
float mprlsOffset = 0.0;
int errCnt = 0;

int timestep = T20TO6;

//#define M_VERB 1 //Verbose meas output
//#define S_VERB 1 //Verbose sys output
//#define S_VERB2 1 //Very Verbose sys output

/**Functions**/
void initReg();
void initPins();
void initNRF();
void initBME();
void initMPR();
void deepSleep(long duration);
uint8_t measUBat();

bool checkMprlsStatus();
float measMprlsLevel(float offset);
void setMprlsZero(int num);
float measDepth(int cnt);
int reResp = 0;


void setup() {

  #ifdef S_VERB
  Serial.begin(9600);
  while (!Serial) //Wait for serial
  {
  }
  Serial.println("\n\n\nReset!");
  #endif
  rxAddr[0] = MODULE_ID;
  initReg();
  initPins();
  initNRF();
  initBME();
  initMPR();
}

unsigned long after = 0;
unsigned long before = 0;
unsigned long before2 = 0;

void loop() {

  //Messen
  float dpt = 0.0;
  dpt = measDepth(1);
  //Ausgeben
  Serial.println(dpt);
  //1 Stunde warten
  delay(3600000);
}

/** Functions **/

/* Set all registers for operation */
void initReg()
{
  #ifdef S_VERB
  Serial.print("Setting register...");
  #endif
  WDTCSR = (24); //change enable and WDE - also resets
  WDTCSR = (33); //prescalers only - get rid of the WDE and WDCE bit
  WDTCSR |= (1<<6); //enable interrupt mode

  //Disable ADC - don't forget to flip back after waking up if using ADC in your application ADCSRA |= (1 << 7);
  //ADCSRA &= ~(1 << 7);
  
  //ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep
  #ifdef S_VERB
  Serial.println("Done");
  #endif
}

/* Init all Pins */
void initPins()
{
  pinMode(P_UBAT_EN, OUTPUT);
  pinMode(P_LED_USER, OUTPUT);

  pinMode(P_SOL, OUTPUT);
  digitalWrite(P_SOL, LOW);
  
  pinMode(P_PUMP, OUTPUT);
  digitalWrite(P_PUMP, LOW);
}

/*Init NRF Module */
void initNRF()
{
  #ifdef S_VERB
  Serial.print("Enabling NRF...");
  #endif
  radio.begin();
  radio.stopListening();
  radio.openReadingPipe(1,rxAddr);
  radio.openWritingPipe(txAddr);
  #ifdef S_VERB
  Serial.println("Done");
  #endif
}

/* Init BME Module */
void initBME()
{
  #ifdef S_VERB
  Serial.print("Enabling BME...");
  #endif
  bme.begin(A_BME_I2C, &Wire);
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
  #ifdef S_VERB
  Serial.println("Done");
  #endif
}

/* Deep sleep for approx. (duration*8.75) seconds */
void deepSleep(int duration)
{
  #ifdef S_VERB
  Serial.println("Going to sleep...");
  delay(200);
  Serial.end();
  #endif
  //digitalWrite(P_UBAT_EN, LOW); //Diable UBAT Meas
  radio.powerDown(); //Disable NRF
  ADCSRA &= ~(1 << 7); //Disable ADC
  //Actual sleep
  for(long i = 0; i < duration; i++)
  {
    //BOD DISABLE - this must be called right before the __asm__ sleep instruction
    MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
    MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time
    __asm__  __volatile__("sleep");//in line assembler to go to sleep
  }
  //Actual wake up
  delay(1);
  #ifdef S_VERB
  Serial.begin(9600);
  while (!Serial) //Wait for serial
  {
  }
  Serial.println("Woke up...");
  #endif
  ADCSRA |= (1 << 7); //Enable ADC
  radio.powerUp(); //Enable NRF
  //digitalWrite(P_UBAT_EN, HIGH); //Enable UBAT Meas
}

/* Measure UBat and give back percent */
uint8_t measUBat()
{
  digitalWrite(P_UBAT_EN, HIGH); //Enable UBAT Meas
  delay(5);
  #ifdef S_VERB2
  Serial.println("Measuring UBat...");
  #endif
  float rawValue = 0;
  for(int i = 0; i < 3; i++)
  {
    delay(1);
    rawValue += analogRead(P_UBAT_MEAS);
  }
  digitalWrite(P_UBAT_EN, LOW); //Diable UBAT Meas
  volt = rawValue / 3072.0 * 6.6 * 1.025586;
  float offsVolt = volt - 3.3;
  if(offsVolt > 1.2) offsVolt = 1.2;
  if(offsVolt < 0.0) offsVolt = 0.0;
  float fPercent = -(13.27 * pow(offsVolt,5)) - (37.49 * pow(offsVolt,4)) + (57.53 * pow(offsVolt,3)) + (42.27 * pow(offsVolt,2)) + (44.17 * offsVolt) - 1.9;
  if(fPercent > 100.0) fPercent = 100.0;
  if(fPercent < 0.00) fPercent = 0.0;
  uint8_t percent = round(fPercent);
  
  #ifdef S_VERB2
  Serial.print("UBat: ");
  Serial.print(volt);
  Serial.print("V, Percent: ");
  Serial.print(percent);
  Serial.println("%\n");
  #endif

  return percent;
}

/* Check MPRLS Status (deprecated) */
bool checkMprlsStatus()
{
  return 1;
}

/*Measure current pressure and return water level equivalent */
float measMprlsLevel(float offset)
{
  int scnt = 0;
  begMeas:
  if(1)
  {  
    float psi =  mpr.readPressure() / 68.947572932;
    if( (psi * 703.06957829636) > 20000 || (psi * 703.06957829636) < -1000)
    {
      scnt ++;
      errCnt++;
      if(scnt > 10)
      {
        #ifdef M_VERB
        Serial.println("!!!REMEASURE FAIL!!!");
        #endif
        return -99999;
      }
      else
      {
        #ifdef M_VERB
        Serial.println("!!!REMEASURE!!!");
        #endif
        delay(100);
        goto begMeas;
      }
    }
    else
    {
      return (psi * 703.06957829636)-offset;
    }
  }
}

/* Init MPRLS Sensor */
void initMPR()
{
  #ifdef S_VERB
  Serial.print("Enabling MPR...");
  #endif
  
  if (! mpr.begin()) 
  {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    while (1) 
    {
      delay(1000);
      Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    }
  }
  delay(100);
  setMprlsZero(10);

  #ifdef S_VERB
  Serial.println("Done");
  #endif
}

/* Get offset and save to global offset variable */
void setMprlsZero(int num)
{
  for(int i = 0; i < num; i++)
  {
    measMprlsLevel(0.0);
    delay(100);
  }
  float offs = measMprlsLevel(0.0);
  delay(100);
  offs += measMprlsLevel(0.0);
  delay(100);
  offs += measMprlsLevel(0.0);
  offs /= 3.0;
  mprlsOffset = offs;
  delay(100);
}

/* Activate pump and solenoids to get water heigt (temp. drift compensated) */
float measDepth(int cnt)
{
  float before[cnt];
  float top[cnt];
  float aftr[cnt];

  int pump_delay = 20;
  int settle_delay = 10;
  int switchover_delay = 1;
  int empty_delay = 50;
  int safecnt = 0;
  
  setMprlsZero(5);
  
  digitalWrite(P_SOL, LOW);
    
  for(int i = 0; i < cnt; i++)
  {
    #ifdef M_VERB
    Serial.println(-20);
    #endif
    //Get 0 press
    before[i] = measMprlsLevel(mprlsOffset)/3.0;
    delay(100);
    before[i] += measMprlsLevel(mprlsOffset)/3.0;
    delay(100);
    before[i] += measMprlsLevel(mprlsOffset)/3.0;
    delay(100);

    safecnt = 0;
    while(before[i] > 3000 || before[i] < -1000)
    {
      #ifdef M_VERB
      Serial.println("Retry measure");
      #endif
      //Get 0 press
      before[i] = measMprlsLevel(mprlsOffset)/3.0;
      delay(100);
      before[i] += measMprlsLevel(mprlsOffset)/3.0;
      delay(100);
      before[i] += measMprlsLevel(mprlsOffset)/3.0;
      delay(100);
      safecnt++;
      if(safecnt == 10)
      {
        #ifdef M_VERB
        Serial.println("Error measure");
        #endif
        before[i] = 0;
      }
    }
    
    digitalWrite(P_PUMP, HIGH);

    //Wait for pump
    for(int i = 0; i < pump_delay; i++)
    {
      #ifdef M_VERB
      Serial.println(measMprlsLevel(mprlsOffset));
      #else
      measMprlsLevel(mprlsOffset);
      #endif
      delay(100);
    }
  
    digitalWrite(P_SOL, HIGH);
    
    for(int i = 0; i < switchover_delay; i++)
    {
      #ifdef M_VERB
      Serial.println(measMprlsLevel(mprlsOffset));
      #else
      measMprlsLevel(mprlsOffset);
      #endif
      delay(100);
    }
    
    digitalWrite(P_PUMP, LOW);

    //Wait for pressure to settle after pump
    for(int i = 0; i < settle_delay; i++)
    {
      #ifdef M_VERB
      Serial.println(measMprlsLevel(mprlsOffset));
      #else
      measMprlsLevel(mprlsOffset);
      #endif
      delay(100);
    }
    
    #ifdef M_VERB
    Serial.println(-20);
    #endif
    
    //Get top press
    top[i] = measMprlsLevel(mprlsOffset)/3.0;
    delay(100);
    top[i] += measMprlsLevel(mprlsOffset)/3.0;
    delay(100);
    top[i] += measMprlsLevel(mprlsOffset)/3.0;
    delay(100);

    safecnt = 0;
    while(top[i] > 3000 || top[i] < -1000)
    {
      #ifdef M_VERB
      Serial.println("Retry measure");
      #endif
      //Get top press
      top[i] = measMprlsLevel(mprlsOffset)/3.0;
      delay(100);
      top[i] += measMprlsLevel(mprlsOffset)/3.0;
      delay(100);
      top[i] += measMprlsLevel(mprlsOffset)/3.0;
      delay(100);
      safecnt++;
      if(safecnt == 10)
      {
        #ifdef M_VERB
        Serial.println("Error measure");
        #endif
        top[i] = 0;
      }
    }
    
    digitalWrite(P_SOL, LOW);
    //Wait for pressure to go back to 0
    for(int i = 0; i < empty_delay; i++)
    {
      #ifdef M_VERB
      Serial.println(measMprlsLevel(mprlsOffset));
      #else
      measMprlsLevel(mprlsOffset);
      #endif
      delay(100);
    }
    
    #ifdef M_VERB
    Serial.println(-20);
    #endif
    
    //Get 0 press after
    aftr[i] = measMprlsLevel(mprlsOffset)/3.0;
    delay(100);
    aftr[i] += measMprlsLevel(mprlsOffset)/3.0;
    delay(100);
    aftr[i] += measMprlsLevel(mprlsOffset)/3.0;
    delay(100);

    safecnt = 0;
    while(aftr[i] > 3000 || aftr[i] < -1000)
    {
      #ifdef M_VERB
      Serial.println("Retry measure");
      #endif
      //Get 0 press after
      aftr[i] = measMprlsLevel(mprlsOffset)/3.0;
      delay(100);
      aftr[i] += measMprlsLevel(mprlsOffset)/3.0;
      delay(100);
      aftr[i] += measMprlsLevel(mprlsOffset)/3.0;
      delay(100);
      safecnt++;
      if(safecnt == 10)
      {
        #ifdef M_VERB
        Serial.println("Error measure");
        #endif
        aftr[i] = 0;
      }
    }
    
  }
  
  float res = 0;
  for(int i = 0; i < cnt; i++)
  {
    res += top[i] - (before[i] + (((aftr[i]-before[i])/(float)(pump_delay+settle_delay+switchover_delay))*(float)empty_delay));//top[i]-((aftr[i] + before[i])/2.0); //top[i]-( before[i] + (aftr[i]-before[i])*( (float)(settle_delay+pump_delay)/5.0 ) );// top[i] - (before[i] + ((aftr[i]-before[i])/(float)(pump_delay+settle_delay+switchover_delay))*(float)empty_delay
    //res += top[i] - (before[i] + (aftr[i]-before[i])/2.0);
    #ifdef M_VERB
      Serial.println("top: " + String(top[i]) + " before: " + String(before[i]) + "aftr: " + String(aftr[i]));
    #endif
  }
  res /= cnt;
  return res;
}

uint8_t msb(uint16_t input)
{
  return (input >> 8 ) & 0xFF;
}
uint8_t lsb(uint16_t input)
{
  return input & 0xFF;
}

/* Leave this untouched */
ISR(WDT_vect) 
{
}
