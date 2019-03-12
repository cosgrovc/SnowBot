
/****************************************
            Code Description

Node code for deployment on 08Mar19 at 
Sagehen Research Site.

Logs Sensors hourly.

****************************************/


/****************************************
               Libraries 
****************************************/
#include <SHT1x.h>
#include <Wire.h>
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>
#include <RH_RF95.h>
//#include "LowPower.h"

/****************************************
                 Pins 
****************************************/
// MaxBotix
const int MB_pwPin = 10;
const int MB_sleepPin = 11;

// SHT10
#define SHT_clockPin 12
#define SHT_dataPin 13
SHT1x sht1x(SHT_dataPin, SHT_clockPin);

// Battery
#define VBATPIN A7

// SD Card
const int SD_Pin = 10;

// LoRa Radio
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

/****************************************
              Definitions
****************************************/
// Debug mode
#define DEBUG 1

// Node number
String node = "SB01";

// Sampling Interval in seconds
int sample_int = 3600; //Hourly 

// RealTimeClock
RTC_PCF8523 rtc;
char iso_dt[20];
char short_dt[15];

// Sensor Readings
long maxbotix, mb_ar[10], z, yyyy, mm, dd, hh, MM, ss, s_unix, f_unix;
float t, rh, t_ar[10], rh_ar[10], vbat;

// Sample id#
int id;

// Char arrays for SD and LoRa
char sd_line[60];
char lora_line[60];

// LoRa Radio
//#define RF95_FREQ 915.0
//RH_RF95 rf95(RFM95_CS, RFM95_INT); // Singleton instance of the radio driver
//int16_t packetnum;
//uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
//uint8_t len = sizeof(buf);
//volatile bool resFlag = false; //Flag is set to true once response is found

/****************************************
                Set-up 
****************************************/

void setup() {
  // Start the Serial Monitor
//  while(!Serial);
  Serial.begin(9600);

  // Set the sample id to 0
  id = 0;

  // Test the RTC
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // LoRa Radio
//  talkToRadio();
//  pinMode(RFM95_RST, OUTPUT);
//  digitalWrite(RFM95_RST, HIGH);
//  Serial.println("Feather LoRa RX Test!");
//  while (!rf95.init()) {
//    Serial.println("LoRa radio init failed");
//    while (1);
//  }
//  Serial.println("LoRa radio init OK!");
//  
//  if (!rf95.setFrequency(RF95_FREQ)) {
//    Serial.println("setFrequency failed");
//    while (1);
//  }
//  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
//  rf95.setTxPower(23, false);
//  Serial.println("Sleeping radio");
//  rf95.sleep();
//  Serial.println();
//  packetnum = 0;  // packet counter, we increment per xmission
  
  // MaxBotix
  pinMode(MB_pwPin, INPUT);
  pinMode(MB_sleepPin, OUTPUT);

  // Test the SD card and write a header
  talkToSD();
  if (!SD.begin(SD_Pin)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
  Serial.println("opening file 'data.txt'");
  File dataFile = SD.open("data.txt", FILE_WRITE);
  Serial.println("writing header");
  dataFile.println("ID NODE DATETIME T(ºC) RH(%) Z(mm) BAT(V)");
  dataFile.close();
  Serial.println();
}

/****************************************
                  Loop 
****************************************/
void loop() {
  // Get the current start unix time in seconds
  DateTime start = rtc.now();
  s_unix = start.unixtime();
  
  // Add one to the id
  id++;

  // Wake up the Node

  // Read the temperature
  read_t();

  // Read the humidity
  read_rh();

  // Read the distance to snow surface
  read_z();

  // Read the battery voltage
  read_vbat();

  // Get the time
  get_time();

  // Create a char array to write to SD card
  make_sd_line();
  
  // Create a char array to send via LoRa
  make_lora_line();

  // Write SD char array to SD card
  SD_write();
  Serial.println();

  // Send char array via LoRa
//  digitalWrite(RFM95_RST, HIGH);
//  Serial.println("Transmitting..."); // Send a message to rf95_server
//  
//  Serial.print("Message = "); Serial.println(lora_line); 
//  Serial.println("Sending...");
//  delay(10);
//  rf95.send((uint8_t *)lora_line, 60);
//
//  Serial.println("Waiting for packet to complete..."); 
//  delay(10);
//  rf95.waitPacketSent();
//  // Now wait for a reply
//  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
//  uint8_t len = sizeof(buf);
//
//  Serial.println("Waiting for reply...");
//  if (rf95.waitAvailableTimeout(1000))
//  { 
//    // Should be a reply message for us now   
//    if (rf95.recv(buf, &len))
//   {
//      Serial.print("Got reply: ");
//      Serial.println((char*)buf);
//      Serial.print("RSSI: ");
//      Serial.println(rf95.lastRssi(), DEC);    
//    }
//    else
//    {
//      Serial.println("Receive failed");
//    }
//  }
//  else
//  {
//    Serial.println("No reply, is there a listener around?");
//  }
//  rf95.sleep();

  // Send to sleep for the Sample interval but subtract 
  // the time that the loop took to run
  DateTime finish = rtc.now();
  f_unix = finish.unixtime();
  int32_t diff = f_unix - s_unix;
  int32_t sleep_int = sample_int - diff;
  Serial.print("Sleep interval (secs) = "); Serial.println(sleep_int);
  delay(sleep_int * 1000UL);
}
/****************************************
               Functions 
****************************************/

// Read Temperature
void read_t(){
  digitalWrite(SHT_dataPin, HIGH);
  digitalWrite(SHT_clockPin, HIGH);
  for(int i = 0; i < 10; i++) {
    t_ar[i] = sht1x.readTemperatureC();
    delay(100);
  }
  t = (t_ar[0]+t_ar[1]+t_ar[2]+t_ar[3]+t_ar[4]+t_ar[5]+
    t_ar[6]+t_ar[7]+t_ar[8]+t_ar[9])/10;
  digitalWrite(SHT_dataPin, LOW);
  digitalWrite(SHT_clockPin, LOW);
  delay(500);
}

// Read Relative Humidity
void read_rh(){
  digitalWrite(SHT_dataPin, HIGH);
  digitalWrite(SHT_clockPin, HIGH);
  for(int i = 0; i < 10; i++) {
    rh_ar[i] = sht1x.readHumidity();
    delay(100);
  }
  rh = (rh_ar[0]+rh_ar[1]+rh_ar[2]+rh_ar[3]+rh_ar[4]+rh_ar[5]+
    rh_ar[6]+rh_ar[7]+rh_ar[8]+rh_ar[9])/10;
  digitalWrite(SHT_dataPin, LOW);
  digitalWrite(SHT_clockPin, LOW);
  delay(500);
}

// Read distance to snow surface
void read_z(){
  // Wake sensor
  digitalWrite(MB_sleepPin, HIGH);
  delay(100);
  // Get z reading in cm
  for(int i = 0; i < 10; i++) {
    mb_ar[i] = pulseIn(MB_pwPin, HIGH);
    delay(100);
  }
  z = (mb_ar[0]+mb_ar[1]+mb_ar[2]+mb_ar[3]+mb_ar[4]+mb_ar[5]+
    mb_ar[6]+mb_ar[7]+mb_ar[8]+mb_ar[9])/10;
  // Sleep sensor
  digitalWrite(MB_sleepPin, LOW);
  delay(500);
}

// Read the battery voltage
void read_vbat(){
  vbat = analogRead(VBATPIN);
  vbat *= 2;    // we divided by 2, so multiply back
  vbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  vbat /= 1024; // convert to voltage
  delay(500);
}

// Get the date/time in ISO yyyy-mm-ddThh:MM:ss format
void get_time(){
    // Get the time objects
    DateTime now = rtc.now();
    yyyy = now.year();
    mm = now.month();
    dd = now.day();
    hh = now.hour();
    MM = now.minute();
    ss = now.second();

    // Put into a char array in iso format
    sprintf(iso_dt, "%4d-%02d-%02dT%d:%02d:%02d", 
      yyyy, mm, dd, hh, MM, ss);

    // Put into a char array in short format
    sprintf(short_dt, "%4d%02d%02dT%d%02d%02d", 
      yyyy, mm, dd, hh, MM, ss);

}

// SD card line
void make_sd_line(){
  sprintf(sd_line, "%d %s %4d-%02d-%02dT%02d:%02d:%02d %f %f %d %f", 
  id, node.c_str(), yyyy, mm, dd, hh, MM, ss, t, rh, z, vbat);
}

// LoRa line
void make_lora_line(){
  int lora_t = t*100;
  int lora_rh = rh*100;
  int lora_vbat = vbat*100;
  sprintf(lora_line, "%d %s %4d%02d%02dT%02d%02d%02dt%drh%dz%db%d", 
  id, node.c_str(), yyyy, mm, dd, hh, MM, ss, lora_t, lora_rh, z, lora_vbat);
}

// Write to SD file
void SD_write(){
  // Open file
  talkToSD();
  File dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile) {
    Serial.println("Writing to SD card");
    dataFile.println(sd_line);
    Serial.println(sd_line);  
    dataFile.close();
    Serial.println("Closing Datafile");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening data.txt");
  }
}

// Set-up to talk to LoRA
void talkToRadio(){
  digitalWrite(SD_Pin, HIGH);
  delay(1);
}

// Set-up to talk to SD
void talkToSD(){
  digitalWrite(RFM95_CS, HIGH);
  delay(1);
}
