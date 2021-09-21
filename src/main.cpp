#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <DFRobot_SHT20.h>
#include <NTPClient.h>
#include <EthernetUdp.h>
#include <NoDelay.h>

#define W5100_CS  10
#define SDCARD_CS 4

//setup noDelay
void time();
noDelay timedelay(10000, time);

DFRobot_SHT20    sht20;

// Update these with values suitable for your network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
IPAddress ip(192, 168, 68, 151); //ip address of device
IPAddress server(192, 168, 68, 65); //ip address of mqtt server


unsigned int localPort = 8888;       // local port to listen for UDP packets

const char timeServer[] = "time.nist.gov"; // time.nist.gov NTP server

//const int 48 = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[48]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

unsigned long Tempdelay =millis();  //for delaying temp/humidity virtualWrite
unsigned long rainDelay =millis();  //for delaying rain virtualWrite.
//motion sensor
int pir = 0;
int lastpir = 0;
char t2[16];
char h2[16];
//door sensors
int Rdoor1down = 0;
int Rdoor1up = 0;
int Rdoor2down = 0;
int Rdoor2up = 0;
//door counters
int door1 = 1;
int lastdoor1 = 2;
int door2 = 1;
int lastdoor2 = 2;
//PIR counter
int count = 1;
//Rain sensor state
bool RainSensor = 0;   
bool LastRainSensor = 0;  
int  Debounce = 0;   
int  Debounce2 = 1;                           
float raincountermm = 0;
float rainYesterdaymm = 0;
float raincounter = 0;       // counter for the number of rain pulses
int lastRainState = 0;     // previous state of the button
int rainYesterday = 0;     //location of yesterdays rainCount
float Total = 0;
float rainTotalmm = 0;             //running rain total counter
float conversion = 2.8;     //Value to convert rain pulses to mm
int SensorLight = 0;        //SensorLight set to off
int lastsensor = 0;
//Time        
int min = 0;               //minutes from NTP server
int hrs = 0;                //hour of day from NTP server (24hr clock)

//NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i=0; i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

EthernetClient ethClient;
PubSubClient client(ethClient);

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(9600);
  //set up mqtt
  client.setServer(server, 1883);
  client.setCallback(callback);
  Ethernet.begin(mac, ip);
  // Allow the hardware to sort itself out
  delay(1500);
  pinMode(3, INPUT_PULLUP);           //PIR pin
  pinMode(4, OUTPUT);                 //Make output and high for use as pull up resitor
  digitalWrite(4, HIGH);
  pinMode(5, INPUT_PULLUP);           //Roller Door 2 down
  pinMode(6, INPUT_PULLUP);           //Roller Door 2 up
  pinMode(7, OUTPUT);                 //Make output and high for use as pull up resitor
  digitalWrite(7, HIGH);
  pinMode(8, OUTPUT);                 //Make output and high for use as pull up resitor
  digitalWrite(8, HIGH);
  pinMode(9, INPUT_PULLUP);            //Roller Door 1 down
  //*****does not like this****pinMode(10, INPUT_PULLUP);            //Roller Door 1 up
  pinMode(11, OUTPUT);                 //Make output and high for use as pull up resitor
  digitalWrite(11, HIGH);
  pinMode(INPUT, 13);                  //Sensor light input, 5V = on
  pinMode(19, INPUT_PULLUP);          //Pin 19 rain sensor (pull high)
  digitalWrite(19, HIGH);
  sht20.initSHT20();                  // Init SHT20 Sensor
  delay(100);
  sht20.checkSHT20();                  // Check SHT20 Sensor
  Udp.begin(localPort);               //timeClient.begin();
}

void sendNTPpacket(const char * address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, 48);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); // NTP requests are to port 123
  Udp.write(packetBuffer, 48);
  Udp.endPacket();
}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  // Run through noDelay functions
  timedelay.update();
  //****************** Temp/Humidity ***********************

  if(millis() > Tempdelay + 2000){                  //delay virtualWrite by 2sec
    Tempdelay = millis(); 
    float h = sht20.readHumidity();                  // Read Humidity
    float t = sht20.readTemperature();               // Read Temperature
        dtostrf(t, 1, 1, t2);                         //convert to 2 dec places
    dtostrf(h, 1, 1, h2);                             //convert to 2 dec places                    
    client.publish("garage/Humidity",h2);             //Publish to MQTT
    client.publish("garage/Temp",t2);                  //Publish to MQTT
  }

  //********************* Rain Counter ******************************

  if(millis() > rainDelay + 2000){    //delay digitalRead by 2sec
    RainSensor = digitalRead(19);
    
    if(RainSensor != LastRainSensor){
        Debounce++;
         
      if (Debounce >10){                //Debounce
        raincounter++;                  // increment rain counter
        LastRainSensor = RainSensor;
        Debounce = 0;
        rainDelay = millis();
      }
    }
  }
      
  if(raincounter!=lastRainState){
    static char raincounter1[3];
    dtostrf(raincounter, 5, 0, raincounter1);
    client.publish("garage/raincounter",raincounter1);  //Publish to MQTT
    static char raincountermm1[5];
    raincountermm = raincounter/conversion;
    dtostrf(raincountermm, 5, 1, raincountermm1);
    client.publish("garage/raincountermm",raincountermm1);  //Publish rain in mm to MQTT
    lastRainState = raincounter;
      }
  
  if(hrs == 9 && min == 00)  {   
    if(Debounce2 != min){
      rainYesterday = raincounter;
      Total = Total + raincounter;
      raincounter = 0;
      //Debounce2 = 1;
      static char rainYesterday1[3];
      dtostrf(rainYesterday, 5, 1, rainYesterday1);
      client.publish("garage/rainYesterday",rainYesterday1);  //Publish to MQTT
      static char rainYesterdaymm1[5];
      rainYesterdaymm = rainYesterday/conversion;
      dtostrf(rainYesterdaymm, 4, 1, rainYesterdaymm1);
      client.publish("garage/rainYesterdaymm",rainYesterdaymm1);  //Publish rain in mm to MQTT
      static char rainTotalmm1[5];
      rainTotalmm = Total/conversion;
      dtostrf(rainTotalmm, 5, 2, rainTotalmm1);
      client.publish("garage/Totalmm",rainTotalmm1);  //Publish to MQTT
      Debounce2 = min;
    }
  }

  //********************* Roller Door *****************************  
                                                          
  Rdoor1down = digitalRead(9);     //Roller Door 1 is house side Roller door 2 is Middle
  //Rdoor1up = digitalRead(10);     //Read the reed switches,create three levels 1,2,3
                                     //Test for change and publish
  if (Rdoor1up == LOW){            
    door1 = 1;   }
  if (Rdoor1down == HIGH && Rdoor1up == HIGH){
    door1 = 2;   }
  if (Rdoor1down == LOW){            
    door1 = 3;  }

  if(door1 != lastdoor1){
    static char door1a[2];
    dtostrf(door1, 2, 0, door1a);

    client.publish("garage/door1",door1a);  //Publish to MQTT
    lastdoor1 = door1;  }

  Rdoor2down = digitalRead(5);   
  Rdoor2up = digitalRead(6);
      
  if (Rdoor2up == LOW){            
    door2 = 1;  }
  if (Rdoor2down == HIGH && Rdoor2up == HIGH){
    door2 = 2;  }
  if (Rdoor2down == LOW){            
    door2 = 3;  }

  if(door2 != lastdoor2){
    static char door2a[2];
    dtostrf(door2, 2, 0, door2a);
    client.publish("garage/door2",door2a);  //Publish to MQTT
    lastdoor2 = door2;    
  }                                

  //************************************* PIR **************************************                                  
    
  pir = digitalRead(3);              //read PIR, If it changes send result.wait for chnage
                                                                      
  if (pir != lastpir) {           //check for change
    static char pir2[2];
    dtostrf(pir, 2, 0, pir2);
    client.publish("garage/pir",pir2);  //Publish to MQTT
    lastpir = pir;   
  }    
  //*************************************Sensor Light*********************************    
  int sensor = digitalRead(13);

  if (sensor !=lastsensor) {           //check for sensorlight change
    static char sensor2[2];
    dtostrf(sensor, 2, 0, sensor2);
    client.publish("garage/sensor",sensor2);  //Publish to MQTT
    Serial.println("sensor");
    lastsensor = sensor;                  
  }
}
//************************** end of void loop ********************** 

//**********************************Time***************************************
void time(){
sendNTPpacket(timeServer); // send an NTP packet to a time server

  // wait to see if a reply is available
  delay(1000);
  if (Udp.parsePacket()) {
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, 48); // read the packet into the buffer

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, extract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
   // Serial.print("Seconds since Jan 1 1900 = ");
    //Serial.println(secsSince1900);

                  // now convert NTP time into everyday time:
   // Serial.print("Unix time = ");
                 // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
                  // subtract seventy years:
    unsigned long epoch = (secsSince1900 - seventyYears) +36000;
                // print Unix time:
    //Serial.println(epoch);

       
    // print the hour, minute and second:
    Serial.print("The Buninyong time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    hrs = ((epoch  % 86400L) / 3600);
    if (((epoch % 3600) / 60) < 10) {
    // In the first 10 minutes of each hour, we'll want a leading '0'
    Serial.print('0');
    }
    
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    min = ((epoch  % 3600) / 60);
    if ((epoch % 60) < 10) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second
  }
  //publish time to mqtt server
    //hrs
  static char hrs2[3];
  dtostrf(hrs, 3, 0, hrs2);
  client.publish("time/hrs",hrs2);
    //min
  static char min2[3];
  dtostrf(min, 4, 0, min2);
  client.publish("time/min",min2);
  
}