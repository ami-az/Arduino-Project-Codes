/ GSM DEFINE
#include <GPRS_Shield_Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#define PIN_TX 7
#define PIN_RX 8
#define BAUDRATE 9600
#define PHONE_NUMBER "0133546320"
#define MESSAGE1 "Alert! Motion Detected!"
#define MESSAGE2 "Alert! Water Detected!"
#define MESSAGE3 "Alert! Fire Detected!"
// lcd with 12c define
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#define I2C_ADDR 0x27 // <<- Add your address here.
#define Rs_pin 0
#define Rw_pin 1
#define En_pin 2
#define BACKLIGHT_PIN 3
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7
54
GPRS gprsTest(PIN_TX, PIN_RX, BAUDRATE); //RX,TX,BaudRate
LiquidCrystal_I2C lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);
int led = 13; // the pin that the LED is atteched to
int pirsensor = 2; // the pin that the sensor is atteched to
int pinSpeaker = 10;
int state = LOW; // by default, no motion detected
int val = 0; // variable to store the sensor status (value)
int reading;
int LM35Feed = 0;
float tCelsius;
//WATER
// These constants won't change. They're used to give names
// to the pins used:
const int analogInPin = A2; // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to
int sensorValue = 0; // value read from the pot
int outputValue = 0; // value output to the PWM (analog out)
//the time we give the sensor to calibrate (10-60 secs according to the datasheet)
55
int calibrationTime = 10;
//the time when the sensor outputs a low impulse
long unsigned int lowIn;
//the amount of milliseconds the sensor has to be low
//before we assume all motion has stopped
long unsigned int pause = 5000;
boolean lockLow = true;
boolean takeLowTime;
//
void setup() {
Serial.begin(9600); // initialize serial
pinMode(led, OUTPUT); // initalize LED as an output
pinMode(pirsensor, INPUT); // initialize sensor as an input
pinMode(pinSpeaker, OUTPUT);
digitalWrite(pirsensor, LOW);
//give the sensor some time to calibrate
Serial.print("calibrating sensor ");
for (int i = 0; i < calibrationTime; i++) {
Serial.print(".");
//lcd.print(".");
delay(1000);
}
Serial.println(" done");
56
Serial.println("SENSOR ACTIVE");
delay(50);
//LCD SETUP
lcd.begin (16, 2); // <<-- our LCD is a 20x4, change for your LCD if needed
// LCD Backlight ON
lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
lcd.setBacklight(HIGH);
}
void loop() {
temperaturesensor();
watersensor();
if (digitalRead(pirsensor) == HIGH) {
digitalWrite(led, HIGH); //the led visualizes the sensors output pin state
playTone(2000, 160);
delay(150);
if (lockLow) {
//makes sure we wait for a transition to LOW before any further output is made:
lockLow = false;
Serial.println("---");
Serial.print("motion detected at ");
Serial.print(millis() / 1000);
57
Serial.println(" sec");
delay(50);
}
takeLowTime = true;
if (!gprsTest.init()) {
lcd.clear ();
lcd.print("GSM is OFF!"); //Print GSM is OFF on LCD
playTone(400, 160);
Serial.print("init error\r\n");
//ledblink();
}
else {
Serial.println("gprs init success");
Serial.println("start to send message ...");
gprsTest.sendSMS(PHONE_NUMBER, MESSAGE1); //define phone number and text
}
}
if (digitalRead(pirsensor) == LOW) {
digitalWrite(led, LOW); //the led visualizes the sensors output pin state
if (takeLowTime) {
lowIn = millis(); //save the time of the transition from high to LOW
58
takeLowTime = false; //make sure this is only done at the start of a LOW phase
}
//if the sensor is low for more than the given pause,
//we assume that no more motion is going to happen
if (!lockLow && millis() - lowIn > pause) {
//makes sure this block of code is only executed again after
//a new motion sequence has been detected
lockLow = true;
Serial.print("motion ended at "); //output
Serial.print((millis() - pause) / 1000);
Serial.println(" sec");
delay(50);
}
}
if (!gprsTest.init()) {
Serial.println("GSM is OFF!");
lcd.setCursor (9, 1);
lcd.print("GSM OFF!");
}
else {
lcd.setCursor (9, 1);
lcd.print(" ");
}
}
//water function
void watersensor()
59
{
//water
// read the analog in value:
sensorValue = analogRead(analogInPin);
delay(10);
// map it to the range of the analog out:
outputValue = map(sensorValue, 0, 1023, 0, 255);
// change the analog out value:
analogWrite(analogOutPin, outputValue);
// print the results to the serial monitor:
Serial.print("sensor = " );
Serial.println(sensorValue);
//Serial.print("\t output = ");
//Serial.println(outputValue);
lcd.home ();
lcd.setCursor (0, 1);
lcd.print("Wtr:");
lcd.print(sensorValue);
//if(!gprsTest.init()) {Serial.println("GSM is OFF!");}
// wait 2 milliseconds before the next loop
// for the analog-to-digital converter to settle
// after the last reading:
delay(500);
if (sensorValue > 300) {
60
playTone(400, 160);
if (!gprsTest.init()) {
lcd.clear ();
lcd.print("GSM is OFF!");
playTone(400, 160);
Serial.print("init error\r\n");
//ledblink();
}
else {
Serial.println("gprs init success");
Serial.println("start to send message ...");
gprsTest.sendSMS(PHONE_NUMBER, MESSAGE2); //define phone number and text
}
}
}
void temperaturesensor()
{
reading = analogRead(LM35Feed);
tCelsius = (((reading / 1024.0) * 5000));
tCelsius = (tCelsius / 10); //LM 35 Equation
Serial.print(tCelsius);
Serial.println(" 'C");
61
lcd.home (); // go home on LCD
lcd.print("Temp:");
lcd.print(tCelsius);
// go to start of 2nd line
lcd.print("'C ");
if (tCelsius > 35) {
playTone(400, 160);
if (!gprsTest.init()) {
lcd.clear ();
lcd.print("GSM is OFF!");
//playTone(400, 160);
Serial.print("init error\r\n");
//ledblink();
}
else {
Serial.println("gprs init success");
Serial.println("start to send message ...");
gprsTest.sendSMS(PHONE_NUMBER, MESSAGE3); //define phone number and text
}
}
}
// duration in mSecs, frequency in hertz
void playTone(long duration, int freq) {
duration *= 1000;
62
int period = (1.0 / freq) * 1000000;
long elapsed_time = 0;
while (elapsed_time < duration) {
digitalWrite(pinSpeaker, HIGH);
delayMicroseconds(period / 2);
digitalWrite(pinSpeaker, LOW);
delayMicroseconds(period / 2);
elapsed_time += (period);
}
}
void ledblink()
{
digitalWrite(led, HIGH);
delay(100);
digitalWrite(led, LOW);
delay(100);
digitalWrite(led, HIGH);
delay(100);
digitalWrite(led, LOW);
delay(100);
digitalWrite(led, HIGH);
delay(100);
}
