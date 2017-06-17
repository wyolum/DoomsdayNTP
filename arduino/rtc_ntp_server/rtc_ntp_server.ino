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
#include "credentials.h"
/* ---- credentials.h ----
char ssid[] = "XXXXXXXXXX";  //  your network SSID (name)
char pass[] = "YYYYYYYYYY";  // your network password
*/

RTC_DS3231 rtc;
DateTime now;
uint32_t last_set_epoch = 0;
uint32_t current_time;
uint32_t last_update_time;
Adafruit_SSD1306 display = Adafruit_SSD1306();

const double LSB = 1./4294967296.;
const unsigned long seventyYears = 2208988800UL;
const uint32_t RTC_UPDATE_INTERVAL = 1000; // seconds
const uint32_t RTC_SET_DURATION_ms = 2;    // 1770 microseconds

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
  Serial.println();
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
  pinMode(2, INPUT);
  rtc.writeSqwPinMode(DS3231_SquareWave1Hz); // start 1 Hz Sq Wave

  while(digitalRead(2) == HIGH){
    // wait for next LOW
  }
  current_time = rtc.now().unixtime() + seventyYears;
  // delay(100); // make sure interrupt does not get called again
  attachInterrupt(2, SqwInterrupt, RISING); 

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
  display.clearDisplay();
  display.display();
  delay(10000);

  // Clear the buffer.
  
  displayTime();
} 

void ntp_timetag(byte *bytes8){
    long2bytes(current_time, bytes8);
    long2bytes((getTimeMS() / 1000.) / LSB, bytes8 + 4);
}

uint32_t packets_served = 0;
void sendNTP(){
    now = rtc.now();
    /*
    Serial.print("now.year():");
    Serial.println(now.year());
    Serial.print("now.unixtime():");
    Serial.println(now.unixtime());
    */
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
      if(diff_ms > TOLLERANCE_MS){
	detachInterrupt(2);                                              /// and local ms = 600, reference is 900, so wait 100 ms and set
	delay((ms_per_second - getTimeMS() - diff_ms - RTC_SET_DURATION_ms) % ms_per_second); 
	current_time = epoch + 1 + seventyYears;
	rtc.adjust(epoch + 1);
	last_update_time = current_time;
	attachInterrupt(2, SqwInterrupt, RISING); 
	Serial.println("Clock reset.");
      }
      
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
void displayTime(){
  display.clearDisplay();

  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("NTP Server Running on");
  display.setCursor(0,11);
  display.print("IP:");
  display.println(WiFi.localIP());
  display.setCursor(0,22);
  uint8_t hh, mm, ss;
  hh = (current_time / 3600) % 24;
  mm = (current_time / 60) % 60;
  ss = (current_time / 1) % 60;
  display.print("GMT: ");
  display.print(hh / 10);
  display.print(hh % 10);
  display.print(":");
  display.print(mm / 10);
  display.print(mm % 10);
  display.print(":");
  display.print(ss / 10);
  display.print(ss % 10);
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
    displayTime();
  }
  int packetSize = udp.parsePacket();
  if (packetSize) {
    ntp_timetag(packetBuffer + 32);
    Serial.print("recieved ");
    Serial.print(packetSize);
    Serial.print(" bytes from ");
    IPAddress remote = udp.remoteIP();
    for (int i = 0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(udp.remotePort());    
    // read the packet into packetBufffer
    udp.read(packetBuffer, packetSize);
    Serial.println("Contents:");
    Serial.println((char*)packetBuffer);

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

