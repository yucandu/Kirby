#include "esp_wifi.h"
//#include "driver/adc.h"
#include <esp_now.h>

#include <WiFi.h>
#include "nvs_flash.h"
//#include <SimplePgSQL.h>
#include "time.h"
#include "SPI.h"
//#include <TFT_eSPI.h> 

#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <ADS1115_WE.h> 
#include <Wire.h>
#define I2C_ADDRESS 0x48
bool isSetNtp = false;     
ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);

 Adafruit_PCD8544 display = Adafruit_PCD8544(0, 1, 2);


Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
#define BATCH_SIZE 12
volatile bool gotBatchAck = false;
volatile uint8_t lastAckBatch = 0;
static uint8_t requestSeq = 0;
volatile uint8_t lastReadySeq = 0;
int16_t adc0, adc1, adc2, adc3;
float volts0, volts1, volts2, volts3;
float abshum;
volatile bool relayComplete = false;
volatile uint8_t lastCompleteSeq = 0;
//TFT_eSPI tft = TFT_eSPI(); 

#include <Preferences.h>
Preferences prefs;
  struct tm timeinfo;
//#include "Adafruit_SHT31.h"

//Adafruit_SHT31 sht31 = Adafruit_SHT31();

RTC_DATA_ATTR int readingCnt = -1;
RTC_DATA_ATTR int arrayCnt = 0;
RTC_DATA_ATTR bool firstRun = true;
volatile bool relayReady = false;
int i;

typedef struct {
  float temp1;
  float temp2;
  unsigned long   time;
  float volts;
  float pres;
} sensorReadings;

#define maximumReadings 360 // The maximum number of readings that can be stored in the available space
#define sleeptimeSecs   30
#define WIFI_TIMEOUT 20000
#define TIME_TIMEOUT 20000
RTC_DATA_ATTR sensorReadings Readings[maximumReadings];
RTC_DATA_ATTR bool hasTimeBeenSet = false;

int hours, mins, secs;
float tempC;
bool sent = false;



uint8_t relayMAC[] = {0xC0, 0x49, 0xEF, 0x93, 0xA9, 0xFC}; // MAC address of the relay device

volatile bool gotAck = false;
volatile bool awaitingAck = false;


// Packet structure for ESP-NOW
typedef struct {
  uint8_t msgType;    // 0=data, 1=start, 2=end, 3=ack, 4=time_request, 5=time_response
  uint16_t packetId;  // Packet sequence number
  uint16_t totalPackets;
  uint16_t dataSize;  // Actual data bytes in this packet
  uint8_t data[240]; // Max ESP-NOW payload is ~250 bytes
} espnow_packet_t;

// Global variables
 bool needTimeSync = true;
RTC_DATA_ATTR unsigned long lastTimeSync = 0;
volatile bool ackReceived = false;
volatile bool timeReceived = false;
volatile unsigned long receivedTime = 0;
unsigned long ackTimeout = 1000; // 1 second timeout
unsigned long timeTimeout = 3000; // 3 second timeout for time sync

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Called when packet is sent
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  espnow_packet_t packet;
  memcpy(&packet, incomingData, sizeof(packet));
  
  if (packet.msgType == 3) { // ACK
    ackReceived = true;
  } else if (packet.msgType == 5) { // Time response
    memcpy((void*)&receivedTime, packet.data, sizeof(unsigned long));
    timeReceived = true;
  }
}

bool sendPacketWithRetry(espnow_packet_t* packet, int maxRetries = 3) {
  for (int retry = 0; retry < maxRetries; retry++) {
    ackReceived = false;
    
    esp_err_t result = esp_now_send(relayMAC, (uint8_t*)packet, sizeof(espnow_packet_t));
    if (result != ESP_OK) {
      delay(100);
      continue;
    }
    
    // Wait for ACK
    unsigned long startTime = millis();
    while (!ackReceived && (millis() - startTime) < ackTimeout) {
      delay(10);
    }
    
    if (ackReceived) {
      return true;
    }
    
    delay(100 * (retry + 1)); // Exponential backoff
  }
  return false;
}

bool requestTimeSync() {
  espnow_packet_t timeRequest;
  timeRequest.msgType = 4; // Time request
  timeRequest.packetId = 0;
  timeRequest.totalPackets = 0;
  timeRequest.dataSize = 0;
  
  timeReceived = false;
  
  for (int retry = 0; retry < 3; retry++) {
    esp_err_t result = esp_now_send(relayMAC, (uint8_t*)&timeRequest, sizeof(espnow_packet_t));
    if (result != ESP_OK) {
      delay(200);
      continue;
    }
    
    unsigned long startTime = millis();
    while (!timeReceived && (millis() - startTime) < timeTimeout) {
      delay(50);
    }
    
    if (timeReceived) {
      // Update RTC or system time here if needed
      lastTimeSync = millis();
      needTimeSync = false;

      struct timeval now = {
        .tv_sec = receivedTime,
        .tv_usec = 0
      };
      settimeofday(&now, nullptr);

      Serial.print("Time set from relay: ");
      Serial.println(receivedTime);

      return true;
    }
    
    delay(500);
  }
  
  Serial.println("Time sync failed");
  return false;
}

bool transmitReadingsArray() {
  const int maxDataPerPacket = 240;
  const int totalDataSize = sizeof(Readings);
  const int totalPackets = (totalDataSize + maxDataPerPacket - 1) / maxDataPerPacket;
  
  Serial.println("Starting transmission of " + String(totalDataSize) + " bytes in " + String(totalPackets) + " packets");
  
  // Send start packet
  espnow_packet_t startPacket;
  startPacket.msgType = 1; // Start
  startPacket.packetId = 0;
  startPacket.totalPackets = totalPackets;
  startPacket.dataSize = totalDataSize;
  
  if (!sendPacketWithRetry(&startPacket)) {
    Serial.println("Failed to send start packet");
    return false;
  }
  
  // Send data packets
  uint8_t* dataPtr = (uint8_t*)&Readings;
  for (int i = 0; i < totalPackets; i++) {
    espnow_packet_t dataPacket;
    dataPacket.msgType = 0; // Data
    dataPacket.packetId = i + 1;
    dataPacket.totalPackets = totalPackets;
    
    int remainingBytes = totalDataSize - (i * maxDataPerPacket);
    int bytesToSend = min(remainingBytes, maxDataPerPacket);
    dataPacket.dataSize = bytesToSend;
    
    memcpy(dataPacket.data, dataPtr + (i * maxDataPerPacket), bytesToSend);
    
    if (!sendPacketWithRetry(&dataPacket)) {
      Serial.println("Failed to send data packet " + String(i + 1));
      return false;
    }
    
    Serial.println("Sent packet " + String(i + 1) + "/" + String(totalPackets));
    delay(50); // Small delay between packets
  }
  
  // Send end packet
  espnow_packet_t endPacket;
  endPacket.msgType = 2; // End
  endPacket.packetId = totalPackets + 1;
  endPacket.totalPackets = totalPackets;
  endPacket.dataSize = 0;
  
  if (!sendPacketWithRetry(&endPacket)) {
    Serial.println("Failed to send end packet");
    return false;
  }
  
  Serial.println("Transmission completed successfully");
  return true;
}

void initESPNOW() {
  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer (relay MAC must be known ahead of time)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, relayMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}


void gotosleep() {
      //WiFi.disconnect();
      delay(1);
      esp_sleep_enable_timer_wakeup(sleeptimeSecs * 1000000ULL);
      delay(1);
      esp_deep_sleep_start();
      delay(1000);
}

void gotosleepfast() {
      //WiFi.disconnect();
          esp_sleep_enable_timer_wakeup(1 * 10000);
          esp_deep_sleep_start();
          delay(1000);
}

void killwifi() {
            WiFi.disconnect(); 
         // WiFi.mode(WIFI_OFF);
          //esp_wifi_stop();
         // adc_power_off();
}




float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  unsigned long start = millis();
  while(adc.isBusy()) {
    if (millis() - start > 200) { // 200ms timeout
      Serial.println("ADC read timeout!");
      return NAN; // or a safe default value
    }
    delay(1);
  }
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}

#include "esp_sntp.h"
void cbSyncTime(struct timeval *tv) { // callback function to show when NTP was synchronized
  Serial.println("NTP time synched");
  isSetNtp = true;
}

void initTime(String timezone){
  configTzTime(timezone.c_str(), "192.168.50.197");

  while ((!isSetNtp) && (millis() < TIME_TIMEOUT)) {
        delay(250);
        display.print(".");
        display.display();
        }

}

bool waitForTime(uint32_t timeout_ms = 3000) {
  unsigned long start = millis();
  while (!hasTimeBeenSet && (millis() - start < timeout_ms)) {
    delay(10);  // Give time for OnDataRecv to handle incoming data
  }
  return hasTimeBeenSet;
}


void setup(void)
{

  Serial.begin(115200);

  /*WiFi.begin();
  delay(100); // Give time for WiFi stack to initialize

  Serial.print("Relay MAC address: ");
  Serial.println(WiFi.macAddress());*/
  sntp_set_time_sync_notification_cb(cbSyncTime);
  //setCpuFrequencyMhz(80);
   // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
 

  display.begin(20, 7);


  display.display(); // show splashscreen
  //delay(1000);
  display.clearDisplay();   // clears the screen and buffer
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0,0);
  display.setTextWrap(true);
  if ((readingCnt == -1)) {
      initESPNOW();
      display.print("Connecting to get time...");
      display.display();



    Serial.println("Requesting time sync...");
    requestTimeSync();

          //initTime("EST5EDT,M3.2.0,M11.1.0");

          readingCnt = 0;
          delay(1);
          readingCnt = 0;
          delay(1);

          esp_sleep_enable_timer_wakeup(1 * 10000);
          esp_deep_sleep_start();
          delay(1000);
  }

  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(0,0);
  display.print("Init wire...");
  display.display();
  Wire.begin();
  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(0,0);
  display.print("Init ADC...");
  display.display();
  adc.init();
  adc.setVoltageRange_mV(ADS1115_RANGE_4096);
  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(0,0);
  display.print("Read ADC...");
  display.display();
  float volts0 = 2.0 * readChannel(ADS1115_COMP_3_GND);
  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(0,0);
  display.print("Init BMP...");
  display.display();

  bmp.begin();
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);
  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(0,0);
  display.print("Read BMP...");
  display.display();
  bmp.takeForcedMeasurement();
  float presread = bmp.readPressure() / 100.0;
  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(0,0);
  display.print("Init AHT...");
  display.display();
  aht.begin();
  sensors_event_t humidity, temp;
  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(0,0);
  display.print("Read AHT...");
  display.display();
  aht.getEvent(&humidity, &temp);
    display.clearDisplay();   // clears the screen and buffer
  display.setCursor(0,0);
  abshum = (6.112 * pow(2.71828, ((17.67 * temp.temperature)/(temp.temperature + 243.5))) * humidity.relative_humidity * 2.1674)/(273.15 + temp.temperature); //calculate absolute humidity
  display.print("Time: ");
  setenv("TZ","EST5EDT,M3.2.0,M11.1.0",1);  //  Now adjust the TZ.  Clock settings are adjusted to show the new local time
  tzset();
  getLocalTime(&timeinfo);
  int hr12 = timeinfo.tm_hour;
  String AMPM;
  if (hr12 > 12) {
    hr12 -= 12;
    AMPM = "PM";
  }
  else if (hr12 == 12) {AMPM = "PM";}
  else {AMPM = "AM";}
  //if (hours == 12) {AMPM = "PM";}
  if (hr12 == 0) {hr12 = 12;}


  display.print(hr12);
  if (timeinfo.tm_min < 10) {display.print(":0");}
  else {display.print(":");}
  
  display.print(timeinfo.tm_min);
  display.println(AMPM);
  display.print(temp.temperature, 2);
  display.print("C ");
  display.print(humidity.relative_humidity, 1);
  display.println("%");

  display.print("Abs: ");
  display.print(abshum, 2);
  display.println("g");

  display.print("Batt: ");
  display.print(volts0, 4);
  display.println("v");

  display.print("Pres: ");
  display.print(presread, 2);
  display.println("m");

  display.print("R");
  display.print(readingCnt); 
  display.print("/");
  display.print(maximumReadings); 
  display.print(" A");
  display.print(arrayCnt);
  display.display();

  Readings[readingCnt].temp1 = temp.temperature;    // Units °C
  Readings[readingCnt].temp2 = abshum; //humidity is temp2
  Readings[readingCnt].time = mktime(&timeinfo); 
  Readings[readingCnt].volts = volts0;
  Readings[readingCnt].pres = presread;



  ++readingCnt; 
  delay(1);

  if (readingCnt >= maximumReadings) {

      prefs.begin("stuff", false, "nvs2");
      //WiFi.setAutoReconnect(false);
      //WiFi.persistent(false);
      //WiFi.disconnect(false,true); 
      initESPNOW();  

      display.clearDisplay();   // clears the screen and buffer
      display.setCursor(0,0);
      display.print("Connecting to transmit...");
      Serial.println("Connecting to transmit...");
      display.display();

      if (transmitReadingsArray()) {
        Serial.println("Data transmission successful");
      } else {
        Serial.println("Data transmission failed");
        delay(1);
        ++arrayCnt;
        delay(1);
        prefs.putBytes(String(arrayCnt).c_str(), &Readings, sizeof(Readings));
        readingCnt = 0;
        killwifi();
        display.clearDisplay();   // clears the screen and buffer
        display.setCursor(0,0);
        display.print("saving to NVS...");
        Serial.println("saving to NVS...");
        display.display();
        delay(1000);
        esp_sleep_enable_timer_wakeup(1 * 1000000);
        esp_deep_sleep_start();
      }
      
      while (arrayCnt > 0) {
        delay(10);
        display.clearDisplay();
        display.setCursor(0,0);
        display.print("TXing #");
        Serial.print("TXing #");
        display.println(arrayCnt - 1);
        Serial.println(arrayCnt - 1);
        display.display();
        //delay(50);
        prefs.getBytes(String(arrayCnt).c_str(), &Readings, sizeof(Readings));
      if (transmitReadingsArray()) {
        Serial.println("Data transmission of array " + String(arrayCnt - 1) + " successful");
      } else {
        Serial.println("Data transmission of array " + String(arrayCnt - 1) + " failed");
        return;
      }
        arrayCnt--;
        
      }
      delay(500);
      requestTimeSync();
      arrayCnt = 0;
      readingCnt = 0;
      delay(1);
      arrayCnt = 0;
      readingCnt = 0;
      delay(1);
      lastReadySeq = 0;
      lastCompleteSeq = 0;
      display.clearDisplay();   // clears the screen and buffer
      display.setCursor(0,0);
      display.print("Done.  Closing connection...");
      Serial.println("Done.  Closing connection...");
      display.display();
      //conn.close();



      
          esp_sleep_enable_timer_wakeup(1 * 10000);
          esp_deep_sleep_start();
          delay(1000);
  } 


        gotosleep();

}


void loop()
{
gotosleep();
}
