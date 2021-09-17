#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <DFRobot_SHT20.h>

#define W5100_CS  10
#define SDCARD_CS 4

DFRobot_SHT20    sht20;

// Update these with values suitable for your network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
IPAddress ip(192, 168, 68, 151); //ip address of device
IPAddress server(192, 168, 68, 65); //ip address of mqtt server

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
int raincountermm =0;
const int  rainPin = 2;    // the pin that the rain is attached to
int raincounter = 0;       // counter for the number of rain pulses
//int rainState = 0;         // current state of the sensor
int lastRainState = 0;     // previous state of the button
int rainYesterday = 0;     //location of yesterdays rainCount
int Total = 0;             //running rain total counter
float conversion = 2.8;     //Value to convert rain pulses to mm
int resetdelay = -60000;    //timer widget reset delay pre set to approx = millis at start up
int SensorLight = 0;        //SensorLight set to off
int lastsensor = 0;

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0; i<length;i++) {
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
      // Once connected, publish an announcement...
      client.publish("outTopic","hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
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
                                        // Debug console
  Serial.println("SHT20 Example!");
  sht20.initSHT20();                                  // Init SHT20 Sensor
  delay(100);
  sht20.checkSHT20();                                 // Check SHT20 Sensor
    
  //pinMode(SDCARD_CS, OUTPUT);
  //digitalWrite(SDCARD_CS, HIGH);      // Deselect the SD card
}

void loop()
{
      if (!client.connected()) {
      reconnect();
    }
    client.loop();

  //****************** Temp/Humidity ***********************

   if(millis() > Tempdelay + 2000){    //delay virtualWrite by 2sec
      Tempdelay = millis(); 
  
  float h = sht20.readHumidity();                  // Read Humidity
  float t = sht20.readTemperature();               // Read Temperature
 Serial.println("Test after Read Temp and Hum:");
 Serial.println(t, 1);
  dtostrf(t, 1, 1, t2);               //convert to 2 dec places
  dtostrf(h, 1, 1, h2);               //convert to 2 dec places
                                     
  client.publish("garage/Humidity",h2);  //Publish to MQTT
  client.publish("garage/Temp",t2);      //Publish to MQTT
  }

  //********************* Rain Counter ******************************

  if(millis() > rainDelay + 2000){    //delay virtualWrite by 2sec
        RainSensor = digitalRead(19);
    
    if(RainSensor != LastRainSensor){
        Debounce++;
        
    if (Debounce >10){
        raincounter++;                  // increment rain counter
        LastRainSensor = RainSensor;
        Debounce = 0;
        rainDelay = millis();
        }
      }
      }
      
  if(raincounter!=lastRainState){

        static char raincounter1[2];
        dtostrf(raincounter, 1, 0, raincounter1);
        client.publish("garage/raincounter",raincounter1);  //Publish to MQTT

        static char raincountermm1[2];
        raincountermm = raincounter/conversion;
        dtostrf(raincountermm, 1, 0, raincountermm1);
        client.publish("garage/raincountermm",raincountermm1);  //Publish rain in mm to MQTT
        
        lastRainState = raincounter;
    }

  //********************* Roller Door *****************************  
                                    //Roller Door 1 is house side Roller door 2 is Middle
                                    //Read the reed switches,create three levels 1,2,3
                                    //Test for change and publish
    Rdoor1down = digitalRead(9);   
    Rdoor1up = digitalRead(10);
        
    if (Rdoor1up == LOW){            
      door1 = 1;    }
    if (Rdoor1down == HIGH && Rdoor1up == HIGH){
      door1 = 2;    }
    if (Rdoor1down == LOW){            
      door1 = 3;    }

    if(door1 != lastdoor1){
      static char door1a[2];
      dtostrf(door1, 1, 0, door1a);

      client.publish("garage/door1",door1a);  //Publish to MQTT
      lastdoor1 = door1;    }

    Rdoor2down = digitalRead(5);   
    Rdoor2up = digitalRead(6);
        
    if (Rdoor2up == LOW){            
      door2 = 1;    }
    if (Rdoor2down == HIGH && Rdoor2up == HIGH){
      door2 = 2;    }
    if (Rdoor2down == LOW){            
      door2 = 3;    }

    if(door2 != lastdoor2){
      static char door2a[2];
      dtostrf(door2, 1, 0, door2a);

      client.publish("garage/door2",door2a);  //Publish to MQTT
      lastdoor2 = door2;    }                                

  //************************************* PIR **************************************                                  
    {
    pir = digitalRead(3);              //read PIR, If it changes send result.wait for chnage
                                                                      
      if (pir != lastpir) {           //check for change
         static char pir2[2];
        dtostrf(pir, 1, 0, pir2);

        client.publish("garage/pir",pir2);  //Publish to MQTT
        lastpir = pir;   }
    }
    
  //*************************************Sensor Light*********************************
  {
    
  int sensor = digitalRead(13);

  if (sensor !=lastsensor) {           //check for sensorlight change
     static char sensor2[2];
     dtostrf(sensor, 1, 0, sensor2);

    client.publish("garage/sensor",sensor2);  //Publish to MQTT
    Serial.println("sensor");
    lastsensor = sensor;                  
  }
  }
 }
//************************** end of void loop ********************** 