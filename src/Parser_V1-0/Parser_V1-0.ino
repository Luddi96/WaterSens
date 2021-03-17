#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>

RF24 radio(D4, D8); // CE, CSN

uint8_t rxAddr[6] = {"0XXXX"}; //Adress of this Node
uint8_t txAddr[6] = {"5XXXX"}; //Adress of Node5

const char *ssid = "XXXX";  //ENTER YOUR WIFI SETTINGS
const char *password = "XXXX";

uint16_t getRemain(uint8_t mod_id);

void setup() 
{
  Serial.begin(9600);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();
  //radio.openWritingPipe(txAddr);
  radio.openReadingPipe(1,rxAddr);

  WiFi.mode(WIFI_OFF);        
  delay(1000);
  WiFi.mode(WIFI_STA);        
  
  WiFi.begin(ssid, password);     
  Serial.println("");
  Serial.print("Connecting");
  
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n\nGO!");
}

int16_t zwerg = 0;
float fzwerg = 0;
uint16_t tx_remain = 4186;

char mod_id = 0x00;
uint8_t tx_arr[] = {0x00, 0x00, 0x0E, 0x00, 0x02, 0xFF};

void loop() 
{
  String valToSend = "?INI=1";
  radio.startListening();
  if(radio.available())
  {    
    Serial.println("");
    Serial.println("_____________");
    Serial.println("Hex Rep:");
    
    uint8_t rcv_arr[35];
    for(int i = 0; i < 35; i++)
    {
      rcv_arr[i] = 0;
    }
    
    radio.read(&rcv_arr, sizeof(rcv_arr));
    
    for(int i = 0; i < 35; i++)
    {
      Serial.print(rcv_arr[i], HEX);
      Serial.print(", ");
    }
    
    Serial.println("");
    Serial.println("_____________");
    Serial.println("Decode:");

    bool rcvCorr = 0;
    for(int i = 0; i < 35; i++)
    {
      switch (rcv_arr[i])
      {
        case 0x00:  i++;
                    Serial.print("Mod ID: ");
                    Serial.println(rcv_arr[i],HEX);
                    mod_id = rcv_arr[i];
                    valToSend += "&MOD=" + String(mod_id, HEX);
                    break;
        case 0x01:  i+=2;
                    zwerg = (rcv_arr[i] | (rcv_arr[i-1] << 8));
                    fzwerg = float(zwerg)/10.0;
                    Serial.print("Temp: ");
                    Serial.println(fzwerg,1);
                    valToSend += "&T=" + String(fzwerg);
                    break;
        case 0x02:  i+=2;
                    zwerg = (rcv_arr[i] | (rcv_arr[i-1] << 8));
                    fzwerg = float(zwerg)/10.0;
                    Serial.print("Hum: ");
                    Serial.println(fzwerg,1);
                    valToSend += "&H=" + String(fzwerg);
                    break;
        case 0x03:  i+=2;
                    zwerg = (rcv_arr[i] | (rcv_arr[i-1] << 8));
                    fzwerg = float(zwerg)/10.0;
                    Serial.print("Press: ");
                    Serial.println(fzwerg,1);
                    valToSend += "&P=" + String(fzwerg);
                    break;
        case 0x04:  i+=2;
                    zwerg = (rcv_arr[i] | (rcv_arr[i-1] << 8));
                    fzwerg = float(zwerg)/10.0;
                    Serial.print("Water: ");
                    Serial.println(fzwerg,1);
                    valToSend += "&W=" + String(fzwerg);
                    break;            
        case 0x05:  if(mod_id < 0xA0)
                    {
                      i++;
                      Serial.print("Batt: ");
                      Serial.println(rcv_arr[i]);
                      valToSend += "&B=" + String(rcv_arr[i],DEC);
                    }
                    else
                    {
                      i+=2;
                      zwerg = (rcv_arr[i] | (rcv_arr[i-1] << 8));
                      fzwerg = float(zwerg)/100.0;
                      Serial.print("Batt: ");
                      Serial.println(fzwerg,2);
                      valToSend += "&BF=" + String(fzwerg);
                    }
                    break;
        case 0x06:  i+=2;
                    zwerg = (rcv_arr[i] | (rcv_arr[i-1] << 8));
                    fzwerg = float(zwerg);
                    Serial.print("UV: ");
                    Serial.println(zwerg);
                    valToSend += "&UV=" + String(zwerg,DEC);
                    break;
        case 0x07:  i+=2;
                    zwerg = (rcv_arr[i] | (rcv_arr[i-1] << 8));
                    fzwerg = float(zwerg);
                    Serial.print("Rain: ");
                    Serial.println(zwerg);
                    valToSend += "&RA=" + String(zwerg,DEC);
                    break;
        case 0x0D:  if(mod_id < 0xA0)
                    {
                      i++;
                      Serial.print("Runtime: ");
                      Serial.println(rcv_arr[i]);
                      valToSend += "&RT=" + String(rcv_arr[i],DEC);
                    }
                    else
                    {
                      i+=2;
                      zwerg = (rcv_arr[i] | (rcv_arr[i-1] << 8));
                      Serial.print("Runtime: ");
                      Serial.println(zwerg);
                      valToSend += "&RT=" + String(zwerg,DEC);
                    }                 
                    break;
        case 0x0E:  //delay(100);
                    radio.stopListening();
                    tx_remain = getRemain(mod_id);
                    txAddr[0] = mod_id;
                    tx_arr[1] = mod_id;
                    tx_arr[3] = msb(tx_remain);
                    tx_arr[4] = lsb(tx_remain);
                    txAddr[0] = mod_id;
                    radio.openWritingPipe(txAddr);
                    Serial.println("Sending..");
                    if(!radio.write(&tx_arr, sizeof(tx_arr)))
                    { Serial.println("T1");
                      delay(100);
                      if(!radio.write(&tx_arr, sizeof(tx_arr)))
                      {Serial.println("T2");
                        delay(100);
                        if(!radio.write(&tx_arr, sizeof(tx_arr)))
                        {Serial.println("T3");
                          delay(100);
                        }
                      }
                    }
                    radio.startListening();
                    break;
        case 0x08:  Serial.println("Gauge Int!");
                    valToSend += "&GA=1";
                    break;
        case 0xFF:  Serial.println("End");
                    rcvCorr = 1;
                    i = 36;
                    break;
        default:    Serial.println("Error decoding!");
                    rcvCorr = 0;
                    i=36;
      }
    }
    
    Serial.println("_____________");
    Serial.println("Finished");
    Serial.println("_____________");
    Serial.println("");

    if(rcvCorr)
    {
      Serial.println(valToSend);
      callUpdate(valToSend);
      Serial.println("_____________");
      Serial.println("");
      
    }
    else
    {
      Serial.println("Print Error");
    }
  }
}

bool callUpdate(String valToSend)
{
  WiFiClient client;
  HTTPClient http;

  String addr = "http://192.168.178.4/hs/updateNG.php"+valToSend;
  
  http.begin(client,addr);

  int httpCode = http.GET();            //Send the request
  String payload = http.getString();

  Serial.println(httpCode);
  Serial.println(payload);

  if(httpCode == 200 && payload == "OK")
  {
    return 1;
  }
  else
  {
    sendErrLog(valToSend + "- PL -" + payload);
    return 0;
  }
}

void sendErrLog(String toSend)
{
  WiFiClient client;
  HTTPClient http;

  String addr = "http://192.168.178.4/hs/errlogNG.php?log="+urlencode(toSend);
  Serial.println(addr);
  http.begin(client,addr);

  int httpCode = http.GET();            //Send the request
  String payload = http.getString();

  Serial.println(httpCode);
  Serial.println(payload);
}
uint16_t getRemain(uint8_t mod_id)
{
  WiFiClient client;
  HTTPClient http;

  String addr = "http://192.168.178.4/hs/ttn.php";
  
  http.begin(client,addr);

  int httpCode = http.GET();            //Send the request
  String payload = http.getString();

  Serial.println(payload);

  uint16_t remain = payload.toInt();
  remain += (mod_id & 0x0F) * 2;
  Serial.println(remain);
  if(remain > 0 && remain < 10000)
  {
    return remain;
  }
  else
  {
    return 4186;
  }
}

uint8_t msb(uint16_t input)
{
  return (input >> 8 ) & 0xFF;
}
uint8_t lsb(uint16_t input)
{
  return input & 0xFF;
}
String urlencode(String str)
{
    String encodedString="";
    char c;
    char code0;
    char code1;
    char code2;
    for (int i =0; i < str.length(); i++){
      c=str.charAt(i);
      if (c == ' '){
        encodedString+= '+';
      } else if (isalnum(c)){
        encodedString+=c;
      } else{
        code1=(c & 0xf)+'0';
        if ((c & 0xf) >9){
            code1=(c & 0xf) - 10 + 'A';
        }
        c=(c>>4)&0xf;
        code0=c+'0';
        if (c > 9){
            code0=c - 10 + 'A';
        }
        code2='\0';
        encodedString+='%';
        encodedString+=code0;
        encodedString+=code1;
        //encodedString+=code2;
      }
      yield();
    }
    return encodedString;
    
}

unsigned char h2int(char c)
{
    if (c >= '0' && c <='9'){
        return((unsigned char)c - '0');
    }
    if (c >= 'a' && c <='f'){
        return((unsigned char)c - 'a' + 10);
    }
    if (c >= 'A' && c <='F'){
        return((unsigned char)c - 'A' + 10);
    }
    return(0);
}
