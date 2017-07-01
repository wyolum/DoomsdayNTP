/*

 Udp NTP Client

 Get the time from a Network Time Protocol (NTP) time server
 Demonstrates use of UDP sendPacket and ReceivePacket
 For more on NTP time servers and the messages needed to communicate with them,
 see http://en.wikipedia.org/wiki/Network_Time_Protocol

 created 4 Sep 2010
 by Michael Margolis
 modified 9 Apr 2012
 by Tom Igoe
 updated for the ESP8266 12 Apr 2015 
 by Ivan Grokhotkov

 This code is in the public domain.

 */

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>
#include "splash.h"
#include "credentials.h"
/* ---- credentials.h ----
char ssid[] = "XXXXXXXXXX";  // your network SSID (name)
char pass[] = "YYYYYYYYYY";  // your network password
*/


RTC_DS3231 rtc;
const uint8_t SQW_PIN = 15;
DateTime now;
uint32_t last_set_epoch = 0;
uint32_t current_time;
uint32_t last_update_time;
Adafruit_SSD1306 display = Adafruit_SSD1306();

byte digits[] = {
0x0f, 0xf0, 0x1f, 0xf8, 0x3f, 0xfc, 0x7f, 0xfe, 0xfc, 0x3f, 0xf8, 0x1f, 0xf0, 0x0f, 0xf0, 0x0f, 
0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 
0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 
0xf0, 0x0f, 0xf0, 0x0f, 0xf8, 0x1f, 0xfc, 0x3f, 0x7f, 0xfe, 0x3f, 0xfc, 0x1f, 0xf8, 0x0f, 0xf0,
0x03, 0xc0, 0x07, 0xc0, 0x0f, 0xc0, 0x1f, 0xc0, 0x3f, 0xc0, 0x3f, 0xc0, 0x03, 0xc0, 0x03, 0xc0,
0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 
0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 
0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x7f, 0xfe, 0x7f, 0xfe, 0x7f, 0xfe, 0x7f, 0xfe,
0x0f, 0xf0, 0x1f, 0xf8, 0x3f, 0xfc, 0x7f, 0xfe, 0xfc, 0x3f, 0xf8, 0x1f, 0xf0, 0x0f, 0xf0, 0x0f, 
0x00, 0x0f, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x1f, 0x00, 0x3f, 0x00, 0x7e, 0x00, 0xfc, 
0x01, 0xf8, 0x03, 0xf0, 0x07, 0xe0, 0x0f, 0xc0, 0x1f, 0x80, 0x3f, 0x00, 0x7e, 0x00, 0xfc, 0x00, 
0xf8, 0x00, 0xf0, 0x00, 0xf0, 0x00, 0xf0, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0x0f, 0xf0, 0x1f, 0xf8, 0x3f, 0xfc, 0x7f, 0xfe, 0xfc, 0x3f, 0xf8, 0x1f, 0xf0, 0x0f, 0xe0, 0x0f, 
0x00, 0x0f, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x1f, 0x00, 0x3f, 0x00, 0xfc, 0x03, 0xf8, 
0x03, 0xf8, 0x00, 0xfc, 0x00, 0x3f, 0x00, 0x1f, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x0f, 
0xe0, 0x0f, 0xf0, 0x0f, 0xf8, 0x1f, 0xfc, 0x3f, 0x7f, 0xfe, 0x3f, 0xfc, 0x1f, 0xf8, 0x0f, 0xf0,
0x00, 0x3c, 0x00, 0x3c, 0xf0, 0x3c, 0xf0, 0x3c, 0xf0, 0x3c, 0xf0, 0x3c, 0xf0, 0x3c, 0xf0, 0x3c, 
0xf0, 0x3c, 0xf0, 0x3c, 0xf0, 0x3c, 0xf0, 0x3c, 0xf0, 0x3c, 0xf0, 0x3c, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 
0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0xf0, 0x00, 0xf0, 0x00, 0xf0, 0x00, 
0xf0, 0x00, 0xf0, 0x00, 0xf0, 0x00, 0xf0, 0x00, 0xff, 0xf0, 0xff, 0xf8, 0xff, 0xfc, 0xff, 0xfe, 
0x00, 0x3f, 0x00, 0x1f, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x0f, 
0xe0, 0x0f, 0xf0, 0x0f, 0xf8, 0x1f, 0xfc, 0x3f, 0x7f, 0xfe, 0x3f, 0xfc, 0x1f, 0xf8, 0x0f, 0xf0,
0x0f, 0xf0, 0x1f, 0xf8, 0x3f, 0xfc, 0x7f, 0xfe, 0xfc, 0x3f, 0xf8, 0x1f, 0xf0, 0x0f, 0xf0, 0x07, 
0xf0, 0x00, 0xf0, 0x00, 0xf0, 0x00, 0xf0, 0x00, 0xf0, 0x00, 0xf0, 0x00, 0xff, 0xf0, 0xff, 0xf8, 
0xff, 0xfc, 0xff, 0xfe, 0xf0, 0x3f, 0xf0, 0x1f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 
0xf0, 0x0f, 0xf0, 0x0f, 0xf8, 0x1f, 0xfc, 0x3f, 0x7f, 0xfe, 0x3f, 0xfc, 0x1f, 0xf8, 0x0f, 0xf0,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x0f, 
0x00, 0x1f, 0x00, 0x3f, 0x00, 0x7e, 0x00, 0xfc, 0x01, 0xf8, 0x03, 0xf0, 0x07, 0xe0, 0x0f, 0xc0, 
0x1f, 0x80, 0x3f, 0x00, 0x3e, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 
0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c, 0x00,
0x0f, 0xf0, 0x1f, 0xf8, 0x3f, 0xfc, 0x7f, 0xfe, 0xfc, 0x3f, 0xf8, 0x1f, 0xf0, 0x0f, 0xf0, 0x0f, 
0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf8, 0x1f, 0xfc, 0x3f, 0x7f, 0xfe, 0x3f, 0xfc, 
0x3f, 0xfc, 0x7f, 0xfe, 0xfc, 0x3f, 0xf8, 0x1f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 
0xf0, 0x0f, 0xf0, 0x0f, 0xf8, 0x1f, 0xfc, 0x3f, 0x7f, 0xfe, 0x3f, 0xfc, 0x1f, 0xf8, 0x0f, 0xf0,
0x0f, 0xf0, 0x1f, 0xf8, 0x3f, 0xfc, 0x7f, 0xfe, 0xfc, 0x3f, 0xf8, 0x1f, 0xf0, 0x0f, 0xf0, 0x0f, 
0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf8, 0x1f, 0xfc, 0x3f, 0x7f, 0xff, 0x3f, 0xff, 
0x1f, 0xff, 0x0f, 0xef, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x0f, 
0xe0, 0x0f, 0xf0, 0x0f, 0xf8, 0x1f, 0xfc, 0x3f, 0x7f, 0xfe, 0x3f, 0xfc, 0x1f, 0xf8, 0x0f, 0xf0,
};
const double LSB = 1./4294967296.;
const unsigned long seventyYears = 2208988800UL;
const uint32_t RTC_UPDATE_INTERVAL = 1000; // seconds
const uint32_t RTC_SET_DURATION_ms = 2;    // 1770 microseconds
const uint32_t MEASURED_BIAS_ms = +400;

const uint32_t TOLLERANCE_MS = 100;
const unsigned int localPort = 123;      // local port to listen for UDP packets
bool ntp_pending = false;
uint32_t ntp_request_sent_ms = 0;
uint16_t ms_per_second = 1000;   // may change to correct clock drift
const int16_t timezone = -4 * 3600;
uint32_t local_hack = 0;
uint32_t local_hack_ms = 0;
uint32_t local_hack_us = 0;
int32_t last_lag_ms = 0;


/* Don't hardwire the IP address or we won't get the benefits of the pool.
 *  Lookup the IP address for the host name instead */
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
const char* ntpServerName = "time.nist.gov";

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

uint32_t bytes2long(byte *bytes){
  uint32_t out = 0;
  out = 0;
  for(int ii=0; ii<4; ii++){
    out |= (bytes[ii] << ((3 - ii) * 8));
  }
  return out;
}

void long2bytes(uint32_t l, byte *bytes){
  for(int ii=0; ii<4; ii++){
    bytes[ii] = (byte)(l >> (3 - ii) * 8);
  }  
}

void hexPrint(byte v){
  byte hi, lo;
  hi = v >> 4;
  lo = v & 0x0F;
  Serial.print(hi, HEX);
  Serial.print(lo, HEX);
}
bool sq_interrupted;
uint32_t last_sqw_ms = 0;
void SqwInterrupt(){
  current_time++;
  last_sqw_ms = millis();
}

uint16_t getTimeMS(){
  uint16_t ms = (millis() - last_sqw_ms) % 1000; // in case we miss a interrupt or two
  return ms;
}

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  delay(500);
  Serial.println("Hello World");
  Serial.println();
  while(false){ // print amount of time it takes to set rtc
    uint32_t now_us = micros();
    rtc.adjust(0);
    Serial.print(micros() - now_us);
    Serial.println(" uS");
    Serial.print("Set RTC_SET_DURATION_ms to ");
    Serial.println(int(((micros() - now_us) + 500) / 1000.));
    delay(1234);
  }

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1){
      for(int i=0; i<3; i++){
	digitalWrite(13, HIGH);
	delay(250);
	digitalWrite(13, LOW);
	delay(250);
      }
      for(int i=0; i<3; i++){
	digitalWrite(13, HIGH);
	delay(500);
	digitalWrite(13, LOW);
	delay(500);
      }
      for(int i=0; i<3; i++){
	digitalWrite(13, HIGH);
	delay(250);
	digitalWrite(13, LOW);
	delay(250);
      }
    }
  }
  Serial.println("RTC Found!");
  pinMode(SQW_PIN, INPUT);
  rtc.writeSqwPinMode(DS3231_SquareWave1Hz); // start 1 Hz Sq Wave

  while(digitalRead(SQW_PIN) == HIGH){
    // wait for next LOW
  }
  current_time = rtc.now().unixtime() + seventyYears;
  // delay(100); // make sure interrupt does not get called again
  attachInterrupt(SQW_PIN, SqwInterrupt, RISING); 

  // We start by connecting to a WiFi network 
  Serial.print("Connecting to "); 
  Serial.println(ssid); 
  WiFi.begin(ssid, pass); 
  
  while (WiFi.status() != WL_CONNECTED) { 
    delay(500); 
    Serial.print("."); 
  } 
  Serial.println(""); 
  
  Serial.println("WiFi connected"); 
  Serial.println("IP address: "); 
  Serial.println(WiFi.localIP()); 
  
  Serial.println("Starting UDP"); 
  udp.begin(localPort); 
  Serial.print("Local port: "); 
  Serial.println(udp.localPort());

  // set up the display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)

  // Clear the buffer.
  //display.clearDisplay();
  //display.fillRect(0, 0, 128, 32, 1);
  //display.drawBitmap(0, 0, IMG, IMG_WIDTH, IMG_HEIGHT, 0);
  //display.display();
  //delay(1000);
  //displayTime();
} 

void ntp_timetag(byte *bytes8){
    long2bytes(current_time, bytes8);
    long2bytes((getTimeMS() / 1000.) / LSB, bytes8 + 4);
}

uint32_t packets_served = 0;
void sendNTP(){
    now = rtc.now();
    //unsynced=0b11, version#=0b100, mode=0b111, stratum=0b0001000, poll=0b00000000, precision0b00000001
    //0b11111111, 0b00001000, 0b00000000, 0b00000001
    packetBuffer[0] = 0b11100111; //unsynced, v4, reservered
    packetBuffer[1] = 0b00001000; //unsynced
    packetBuffer[2] = 0b00000000; // poll interval
    packetBuffer[3] = 0b00000001; // precision
    long2bytes(last_update_time, packetBuffer + 16); // reference timestamp
    ntp_timetag(packetBuffer + 40);
    udp.beginPacket(udp.remoteIP(), udp.remotePort()); 
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
    Serial.println("Replied");
    packets_served++;
}

void requestNTP(){
  //get a random server from the pool
  IPAddress nist_timeServerIP; // time.nist.gov NTP server address
  WiFi.hostByName(ntpServerName, nist_timeServerIP); 

  if(!ntp_pending){ // request
    Serial.println(nist_timeServerIP);
    sendNTPpacket(nist_timeServerIP); // send an NTP packet to a time server
    Serial.println("NTP request sent");
    ntp_request_sent_ms = millis();
    ntp_pending = true;
  }
  if(millis() - ntp_request_sent_ms > 1){ // receive and parse
    ntp_pending = false;
    Serial.println("check for NTP data");
    int n_byte = udp.parsePacket();
    if(n_byte >= NTP_PACKET_SIZE) { // we have received ntp packet back!
      uint32_t receive_ms = millis();
      int32_t lag_ms = receive_ms - ntp_request_sent_ms;
      Serial.print("Lag (ms):");
      Serial.println(lag_ms);

      // We've received a packet, read the data from it
      udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

      //the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, esxtract the two words:
      // inspect header
      for(int ii=0; ii<4; ii++){
	Serial.println(packetBuffer[ii], BIN);
      }
      
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = bytes2long(packetBuffer + 40);
      unsigned long hack_us = bytes2long(packetBuffer + 44) * 1e6 * LSB;
      unsigned long origin_stamp = bytes2long(packetBuffer + 24);
      unsigned long receipt_stamp = bytes2long(packetBuffer + 32);
      unsigned long transmit_stamp = bytes2long(packetBuffer + 40);

      // now convert NTP time into everyday time:
      // subtract seventy years:
      unsigned long epoch = secsSince1900 - seventyYears;
      uint32_t hack = epoch + timezone;
      double expect = local_hack + (local_hack_us / 1000. + receive_ms - local_hack_ms) / 1000.;
      //correction =  -lag_ms + last_lag_ms - dLag
      // expect += lag_ms/2000.;
      double got = hack + hack_us / 1e6;
      int diff_ms = (int)((got - expect) * 1000);                        /// say diff_ms = 300 ==> refernce is 300 ms ahead of local
      Serial.print("got - expect(ms): ");
      Serial.println((int)(diff_ms));
      if(abs(diff_ms) > TOLLERANCE_MS){
	detachInterrupt(2);                                              /// and local ms = 600, reference is 900, so wait 100 ms and set
	delay((ms_per_second - getTimeMS() - diff_ms - RTC_SET_DURATION_ms - MEASURED_BIAS_ms) % ms_per_second); 
	current_time = epoch + 1 + seventyYears;
	rtc.adjust(epoch + 1);
	attachInterrupt(2, SqwInterrupt, RISING); 
	Serial.println("Clock reset.");
      }
      last_update_time = current_time;
      
      uint32_t hack_ms = millis();

      local_hack = hack;
      local_hack_ms = hack_ms;
      local_hack_us = hack_us;
      last_lag_ms = lag_ms;
    }
    else{
      if(millis() - ntp_request_sent_ms > 1000){
	ntp_pending = false; // give up on this packet and request a new one
      }
      else{
	ntp_pending = true; // check back later
      }
    }
  }
}

uint32_t last_display_time;
void displayTimeOfDay(uint8_t x, uint8_t y, char* tag, uint8_t hh, uint8_t mm, uint8_t ss){
  display.setCursor(x, y);
  display.print(tag);
  display.print(hh / 10);
  display.print(hh % 10);
  display.print(":");
  display.print(mm / 10);
  display.print(mm % 10);
  display.print(":");
  display.print(ss / 10);
  display.print(ss % 10);
}
void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color){
  uint16_t i, j;
  
  for(i=x; i < x + w; i++){
    for(j=y; j < y + h; j++){
      display.drawPixel(i, j, color);
    }
  }
}
void drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color){
  display.drawFastHLine(x, y, w, color);
  display.drawFastVLine(x, y, h, color);
  display.drawFastHLine(x, y + h, w, color);
  display.drawFastVLine(x+w, y, h+1, color);
}

void tod_screen1(){
  uint8_t hh, mm, ss;

  display.print(WiFi.localIP());
  //display.print("192.168.001.013:123");
  display.print(":");
  display.print(localPort);
  
  hh = (last_update_time / 3600) % 24;
  mm = (last_update_time / 60) % 60;
  ss = (last_update_time / 1) % 60;
  displayTimeOfDay(0, 11, "REF: ", hh, mm, ss);

  hh = (current_time / 3600) % 24;
  mm = (current_time / 60) % 60;
  ss = (current_time / 1) % 60;
  displayTimeOfDay(0, 22, "GMT: ", hh, mm, ss);
}

void tod_screen2(){
  display.setCursor(0, 0);
  display.print(WiFi.localIP());
  //display.print("192.168.001.013:123");
  display.print(":");
  display.print(localPort);
  display.setCursor(0, 11);
  display.print("R  ");
  display.print(last_update_time);
  display.setCursor(0, 22);
  display.print("G  ");
  display.print(current_time);  
}
void KandyTime(){
  uint8_t hh, mm, ss;
  
  hh = (current_time / 3600) % 24;
  mm = (current_time / 60) % 60;
  ss = (current_time / 1) % 60;
  display.clearDisplay();
  display.drawBitmap(5 + 0 * 17 + 0, 0, digits + 64 * (hh / 10), 16, 32, 1);
  display.drawBitmap(5 + 1 * 17 + 0, 0, digits + 64 * (hh % 10), 16, 32, 1);
  display.drawBitmap(5 + 2 * 17 + 8, 0, digits + 64 * (mm / 10), 16, 32, 1);
  display.drawBitmap(5 + 3 * 17 + 8, 0, digits + 64 * (mm % 10), 16, 32, 1);
  display.drawBitmap(5 + 4 * 17 + 18, 0, digits + 64 * (ss / 10), 16, 32, 1);
  display.drawBitmap(5 + 5 * 17 + 18, 0, digits + 64 * (ss % 10), 16, 32, 1);
  display.fillCircle(5 + 2 * 17 + 3, 10, 2, 1);
  display.fillCircle(5 + 2 * 17 + 3, 20, 2, 1);
  display.fillCircle(5 + 4 * 17 + 12, 10, 2, 1);
  display.fillCircle(5 + 4 * 17 + 12, 20, 2, 1);
  display.display();
}

void displayTime(){
  uint16_t fill;
  
  display.clearDisplay();

  display.setTextColor(WHITE);
  display.setCursor(0,0);
  if(current_time % 20 < 10){
    tod_screen1();
  }
  else{
    tod_screen2();
  }
  // (int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  drawRect(100, 11, 27, 7, 1);
  fill = (uint16_t)((100 + WiFi.RSSI()) * 25. / 100);
  fillRect(126 - fill, 13, fill, 4, 1);
  
  int n_space;
  if(packets_served == 0){
    n_space = 7;
  }
  else{
    n_space = 7 - (int)(log10(packets_served));
  }
  for(int ii=0; ii< n_space; ii++){
    display.print(" "); // right justify packets served
  }
  display.print(packets_served % (100000000));

  display.display();
  last_display_time = current_time;
  
}

void loop()
{
if(current_time != last_display_time){
  //displayTime();
    KandyTime();
  }
  int packetSize = udp.parsePacket();
  if (packetSize) {
    ntp_timetag(packetBuffer + 32);
    Serial.print("received ");
    Serial.print(packetSize);
    Serial.print(" bytes from ");
    IPAddress remote = udp.remoteIP();
    for (int i = 0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(":");
    Serial.print(udp.remotePort());    
    Serial.print(" at ");
    Serial.println(current_time);
    // read the packet into packetBufffer
    udp.read(packetBuffer, packetSize);

    // return the NTP packet
    sendNTP();
  }
  if(current_time - last_update_time > RTC_UPDATE_INTERVAL){
    requestNTP();
  }
  return;
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
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
  // 8 bytes of origin timestamp
  packetBuffer[24] = 1;
  packetBuffer[25] = 2;
  packetBuffer[26] = 3;
  packetBuffer[27] = 4;
  packetBuffer[28] = 5;
  packetBuffer[29] = 6;
  packetBuffer[30] = 7;
  packetBuffer[31] = 8;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

