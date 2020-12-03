//By ami-az.github.io 2019

#include "heltec.h"
#include "images.h"
#include "TinyGPS++.h"
#include <BlynkSimpleEsp32.h>
#include "EasyBuzzer.h"

#define BLYNK_PRINT Serial
TinyGPSPlus gps; //xan lora
HardwareSerial gps_serial(1);
WidgetMap myMap(V0); // V0 for virtual pin of Map Widget
BlynkTimer timer;

float spd; //Variable to store the speed
float sats; //Variable to store no. of satellites response
String bearing; //Variable to store orientation or direction of GPS
#define BAND 433E6 //Asia frequency band
//unsigned int counter = 0;
String rssi = "RSSI --";
String packSize = "--";

String packet ;

char auth[] = "xlyKgBp0GriNoBpgEbgByRStBZ0GtWD4"; // Your Project authentication key 
char ssid[] = "wifi MAWAR-TIME2.4Ghz"; // Name of your network (Wifi or Hotspot)
char pass[] = "dekandekanmawar2018"; // Corresponding Password

/* LM35 Temperature Module */
#define PIN_UPTIME V6

BLYNK_READ(PIN_UPTIME)
{
  Blynk.virtualWrite(PIN_UPTIME, millis() / 1000);
}

void myTimerEvent()
{
  int analogValue = analogRead(36); //reading the sensor on pin36
  float millivolts = (analogValue/1024.0) * 3300; //3300 is the voltage provided by ESP32 pin
  float fahrenheit = (millivolts/10);
  float celcius = (fahrenheit-32)*5/9;
  Serial.print("Surrounding Temp: ");
  Serial.print(celcius);
  Serial.println(" 'C");
  Blynk.virtualWrite(V5, celcius); //sending to Blynk
 
}

void emergencytemp()
{
  int analogValue = analogRead(36); //reading the sensor on pin36
  float millivolts = (analogValue/1024.0) * 3300; //3300 is the voltage provided by ESP32
  float fahrenheit = (millivolts/10);
  float celcius = (fahrenheit-32)*5/9;
  
  if (celcius > 200)
  {
    LoRa.print("   Help!");
    Serial.println("Panic Button Pressed!");
  }
 
}

/* PIN for BUZZER & LED*/
int ledpin = 16; 

/* HC-SR501 Motion Detector */
int PIRsensor = 22; // GPIO for PIR

/* Raindrop Sensor  */
int rainPin = 38; // GPIO for Raindrop
int thresholdValue = 2500 ; // Raindrop Reading Value

/* MQ2 Smoke Sensor  */
int mq2 = 39; // smoke sensor is connected with the analog pin A0 
int data = 0; 

/* BUZZER */
unsigned int frequency = 3000; //the higher is louder
unsigned int beeps = 10; //number of beeps
int buzzerpin = 13;

void logo() //for oled display
{
 Heltec.display->clear();
 Heltec.display->drawXbm(0,5,logo_width,logo_height,logo_bits);
 Heltec.display->display();
}
unsigned int move_index = 1; // fixed location for now

void setup()
{
 Serial.begin(9600);
 gps_serial.begin(9600, SERIAL_8N1, 12, 15);
 EasyBuzzer.setPin(buzzerpin);
 
 Blynk.begin(auth, ssid, pass);
 timer.setInterval(1000L, myTimerEvent);
 
 pinMode(ledpin, OUTPUT); // LED as output
 pinMode(PIRsensor, INPUT); // PIR sensor as input  
 pinMode(rainPin, INPUT);  //Raindrop sensor as input  


 //-------------------FOR SMOKE SENSOR-----------------
 timer.setInterval(1000L, getSendData);
 
 //WIFI Kit series V1 not support Vext control
 Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);

 Heltec.display->init();
 Heltec.display->flipScreenVertically();
 Heltec.display->setFont(ArialMT_Plain_10);
 logo();
 delay(1500);
 Heltec.display->clear();

 Heltec.display->drawString(0, 0, "Heltec.LoRa Initial success!");
 Heltec.display->display();
 delay(1000);
}
void loop()
{
 Heltec.display->clear();
 Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
 Heltec.display->setFont(ArialMT_Plain_10);

 Heltec.display->drawString(0, 0, "Sending packet: ");
// Heltec.display->drawString(83, 0, String(counter));

 Heltec.display->drawString(0, 10, "Longitude: ");
 Heltec.display->drawString(83, 10, String(gps.location.lng(), 4));
 
 Heltec.display->drawString(0, 20, "Latitude: ");
 Heltec.display->drawString(83, 20, String(gps.location.lat(), 5));

 Heltec.display->drawString(0, 30, "Satellites: ");
 Heltec.display->drawString(83, 30, String(gps.satellites.value()));
 
 Heltec.display->drawString(0, 40, "Altitude: ");
 Heltec.display->drawString(83, 40, String(gps.altitude.feet() / 3.2808));
 
 Heltec.display->drawString(115, 40, " M");
 Heltec.display->drawString(0, 50, "Time: ");
 Heltec.display->drawString(83, 50, String(gps.time.hour()-4) + ":" + (gps.time.minute()) + ":" + (gps.time.second()));
 
 Heltec.display->display();
 float latitude = (gps.location.lat()); //Storing the Lat. and Lon. 
 float longitude = (gps.location.lng());
 float attude = (gps.altitude.feet() / 3.2808);
 
 
 Blynk.virtualWrite(V1, String(latitude, 6));
 Blynk.virtualWrite(V2, String(longitude, 6));
 Blynk.virtualWrite(V3, String(attude, 2));
 myMap.location(move_index, latitude, longitude, "GPS_Location");

 sats = gps.satellites.value(); //get number of satellites
 Blynk.virtualWrite(V4, sats);

 // transmit packet to receiver
 LoRa.beginPacket();
 LoRa.setTxPower(14,RF_PACONFIG_PASELECT_PABOOST);
 LoRa.print(" Long: ");
 LoRa.print(gps.location.lng());
 LoRa.println("                   ");
 LoRa.println("   Lat: ");
 LoRa.print(gps.location.lat());
 LoRa.println("                       ");
 LoRa.print("   Time: ");
 LoRa.print(gps.time.hour() - 4);
 LoRa.print(".");
 LoRa.print(gps.time.minute()); 
 emergencytemp();
 LoRa.endPacket();
 
// counter++;
 digitalWrite(LED, HIGH); // turn the LED on (HIGH is the voltage level)
 delay(1000); // wait for a second
 digitalWrite(LED, LOW); // turn the LED off by making the voltage LOW
 delay(1000); // wait for a second
 //GPS
 smartDelay(1000);
 if (millis() > 5000 && gps.charsProcessed() < 10)
 Serial.println(F("No GPS data received: check wiring"));

 
 //-------------------FOR PIR SENSOR----------------------
     
  motiondetector();

 //-------------------FOR RAINDROP SENSOR-----------------
  
 raindrop();
 
 Blynk.run();
   
 timer.run();

 EasyBuzzer.update();
 
}

static void smartDelay(unsigned long ms)
{
 unsigned long start = millis();
 do
 {
 while (gps_serial.available())
 gps.encode(gps_serial.read());
 } while (millis() - start < ms);
}

void motiondetector(void)
{
  int state = digitalRead(PIRsensor); //Continuously check the state of PIR sensor
  
    if(state == HIGH){     
      Serial.println("Motion: Detected!"); 
      EasyBuzzer.beep(frequency, beeps);
      Blynk.notify("Warning ==> Motion Detected");          
      delay(4000);                    
              
    }
    else {
      Serial.println("Motion: Absent");
      EasyBuzzer.stopBeep();
      
      }
}

void raindrop(void){
  int sensorValue = analogRead(rainPin);
  Serial.print("Raindrop Sensor: "); 
  Serial.println(analogRead(rainPin));

  if(sensorValue < thresholdValue && sensorValue > 200)
    {
    EasyBuzzer.beep(frequency, beeps);
    Serial.println("(Rain Detected!)");
    Blynk.notify("Warning ==> Rain detected"); 
    delay(4000); 

    }
     else {
    EasyBuzzer.stopBeep();
    }
}

void getSendData()
{
  data = analogRead(mq2); 
  Blynk.virtualWrite(V8, data);
  Serial.print("Gas Threshold: "); 
  Serial.println(analogRead(mq2));
 
  if (data > 2000 )
  {
     EasyBuzzer.beep(frequency, beeps);
    Serial.println("(Smoke detected!)"); 
    Blynk.notify("Warning ==> Smoke Detected!"); 
  }
  else {
    EasyBuzzer.stopBeep();
    }

}
