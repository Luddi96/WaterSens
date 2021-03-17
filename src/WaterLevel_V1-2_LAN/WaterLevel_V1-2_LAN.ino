/**Includes**/
#include <Wire.h> //Needed for BME
#include <SPI.h> //Needed for NRF
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Adafruit_Sensor.h> //Needed for BME
#include <Adafruit_BME280.h> //Needed for BME
#include "Adafruit_MPRLS.h" //Needed for MPRLS

/**Pin Assignment**/
#define P_LAN_CE 8 //CE pin of LAN module
#define P_LED_USER 5 //Led for button illumination (approx. 20mA)
#define P_SOL 4 //Solenoid enable (approx. 400mA)
#define P_PUMP 3 //Pump enable (approx. 600mA)
#define P_CT A0 //Readout of external load current (500W = cnt)
#define CT_TRESH 100 //Threshold of current transformer

/**Adress definition**/
#define A_BME_I2C 0x76
#define A_MPRLS_I2C 0x18
#define A_UDP_PORT 8888
#define MODULE_ID 0x0A
#define UDP_PACK_ID_1 0x4F
#define UDP_PACK_ID_2 0x54
#define UDP_PACK_ID_3 0x54
#define UDP_PACK_ID_4 0x52

/**Instance creation**/
//Adafruit_BME280 bme;
Adafruit_MPRLS mpr = Adafruit_MPRLS(-1,-1);
EthernetUDP Udp;

/**Global variables**/
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; //LAN MAC
float mprlsOffset = 0.0;
int errCnt = 0;
unsigned int localPort = A_UDP_PORT;       // local port to listen for UDP packets
char udpPacketBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet
float lastMeasurement = 0.00;
IPAddress udpRemoteIP;
int udpRemotePort;
bool pumpState = 0;

//#define M_VERB 1 //Verbose meas output
//#define S_VERB 1 //Verbose sys output
//#define S_VERB2 1 //Very Verbose sys output

/**Functions**/
void initReg();
void initPins();
void initBME();
void initMPR();
void initLAN();

bool checkMprlsStatus();
float measMprlsLevel(float offset);
void setMprlsZero(int num);
float measDepth(int cnt);

void checkUdp();
void checkPump();

void setup() {

  #ifdef S_VERB
  Serial.begin(9600);
  while (!Serial) //Wait for serial
  {
  }
  Serial.println("\n\n\nReset!");
  #endif
  initReg();
  initPins();
  //initBME();
  initMPR();
  initLAN();
}

void loop() {
  
  checkUdp();
  checkPump(); //this delays 10ms
}

/** Functions **/

/* Set all registers for operation */
void initReg()
{
  #ifdef S_VERB
  Serial.print("Setting register...");
  #endif
  
  #ifdef S_VERB
  Serial.println("Done");
  #endif
}

/* Init all Pins */
void initPins()
{
  pinMode(P_LED_USER, OUTPUT);

  pinMode(P_SOL, OUTPUT);
  digitalWrite(P_SOL, LOW);
  
  pinMode(P_PUMP, OUTPUT);
  digitalWrite(P_PUMP, LOW);
}

/* Init BME Module */
/*void initBME()
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
}*/

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

void initLAN()
{
  Ethernet.init(8);
  #ifdef S_VERB
  Serial.print("Enabling LAN...");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
    // no point in carrying on, so do nothing forevermore:
    while (true) {
      delay(1);
      Serial.print(".");
    }
  }
  else
  {
    Serial.println("Done");
  }
  #else
  Ethernet.begin(mac);
  #endif
  Udp.begin(localPort);
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

void checkUdp()
{
  int packetSize = Udp.parsePacket();
  if (packetSize) 
  {
    // read the packet into packetBufffer
    Udp.read(udpPacketBuffer, UDP_TX_PACKET_MAX_SIZE);
    
    for(int i = 0; i < 10; i++)
    {
      //Serial.print(udpPacketBuffer[i], HEX);
      //Serial.print(", ");
    }
    //Serial.println("");
    
    if(udpPacketBuffer[0] == UDP_PACK_ID_1 && udpPacketBuffer[1] == UDP_PACK_ID_2 && udpPacketBuffer[2] == UDP_PACK_ID_3 && udpPacketBuffer[3] == UDP_PACK_ID_4 && udpPacketBuffer[5] == 0x00)
    {
      udpRemoteIP = Udp.remoteIP();
      udpRemotePort = Udp.remotePort();
      udpRemotePort = 8888;
      //Serial.println(udpRemoteIP);
      //Serial.println(udpRemotePort);
      char uID1 = udpPacketBuffer[6];
      char uID2 = udpPacketBuffer[7];
      if( udpPacketBuffer[4] == 0x02 )
      {
        //Serial.println("Get last data");
        uint32_t long_res = round(100.0*lastMeasurement);
        //Serial.println(lastMeasurement);
        uint8_t res_b0 = long_res & 0xFF;
        uint8_t res_b1 = (long_res >> 8) & 0xFF;
        uint8_t res_b2 = (long_res >> 16) & 0xFF;
        char udpReplyBuffer2[] = {UDP_PACK_ID_1,UDP_PACK_ID_2,UDP_PACK_ID_3,UDP_PACK_ID_4,0x02,0x01,uID1,uID2,0x04,res_b2,res_b1,res_b0};
        Udp.beginPacket(Udp.remoteIP(), udpRemotePort);
        Udp.write(udpReplyBuffer2,sizeof(udpReplyBuffer2));
        Udp.endPacket();
      }
      if( udpPacketBuffer[4] == 0x03 )
      {
        //Serial.println("Measure");
        lastMeasurement = measDepth(1);
        if(lastMeasurement < 0.0) lastMeasurement = 0.0;
        char udpReplyBuffer3[] = {UDP_PACK_ID_1,UDP_PACK_ID_2,UDP_PACK_ID_3,UDP_PACK_ID_4,0x03,0x01,uID1,uID2};
        Udp.beginPacket(Udp.remoteIP(), udpRemotePort);
        Udp.write(udpReplyBuffer3,sizeof(udpReplyBuffer3));
        Udp.endPacket();
      }
      if( udpPacketBuffer[4] == 0x04 )
      {
        //Serial.println("Measure and get last data");
        lastMeasurement = measDepth(1);
        if(lastMeasurement < 0.0) lastMeasurement = 0.0;
        uint32_t long_res = round(100.0*lastMeasurement);
        uint8_t res_b0 = long_res & 0xFF;
        uint8_t res_b1 = (long_res >> 8) & 0xFF;
        uint8_t res_b2 = (long_res >> 16) & 0xFF;
        char udpReplyBuffer4[] = {UDP_PACK_ID_1,UDP_PACK_ID_2,UDP_PACK_ID_3,UDP_PACK_ID_4,0x04,0x01,uID1,uID2,0x04,res_b2,res_b1,res_b0};
        Udp.beginPacket(Udp.remoteIP(), udpRemotePort);
        Udp.write(udpReplyBuffer4,sizeof(udpReplyBuffer4));
        Udp.endPacket();
      }
    }
  }
}

void checkPump()
{
  int meanCT = 0;
  for(int i = 0; i < 10; i++)
  {
    meanCT += analogRead(P_CT);
    delay(1);
  }
  if(pumpState)
  {
    if(meanCT < CT_TRESH)
    {
      char udpReplyBuffer4[] = {UDP_PACK_ID_1,UDP_PACK_ID_2,UDP_PACK_ID_3,UDP_PACK_ID_4,0x05,0x00,0x12,0x34,0x02,0x00,0x00,0x00};
      Udp.beginPacket(udpRemoteIP, udpRemotePort);
      Udp.write(udpReplyBuffer4,sizeof(udpReplyBuffer4));
      Udp.endPacket();
      pumpState = 0;
    }
  }
  else
  {
    if(meanCT > CT_TRESH)
    {
      char udpReplyBuffer4[] = {UDP_PACK_ID_1,UDP_PACK_ID_2,UDP_PACK_ID_3,UDP_PACK_ID_4,0x05,0x00,0x12,0x34,0x01,0x00,0x00,0x00};
      Udp.beginPacket(udpRemoteIP, udpRemotePort);
      Udp.write(udpReplyBuffer4,sizeof(udpReplyBuffer4));
      Udp.endPacket();
      pumpState = 1;
    }
  }
}
