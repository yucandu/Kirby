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
#define sleeptimeSecs   1
#define WIFI_TIMEOUT 20000
#define TIME_TIMEOUT 20000
RTC_DATA_ATTR sensorReadings Readings[maximumReadings];
RTC_DATA_ATTR bool hasTimeBeenSet = false;

int hours, mins, secs;
float tempC;
bool sent = false;

//IPAddress PGIP(192,168,50,197);        // your PostgreSQL server IP 





uint8_t relayMAC[] = {0xC0, 0x49, 0xEF, 0x93, 0xA9, 0xFC}; // MAC address of the relay device

volatile bool gotAck = false;
volatile bool awaitingAck = false;

void initESPNOW() {
  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer (relay MAC must be known ahead of time)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, relayMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

bool sendDataBatched(sensorReadings* data, int count) {
  uint8_t packetBuffer[BATCH_SIZE * sizeof(sensorReadings)];
  uint8_t batchNum = 0;

  for (int i = 0; i < count; i += BATCH_SIZE) {
    int batchLen = min(BATCH_SIZE, count - i);
    memcpy(packetBuffer, &data[i], batchLen * sizeof(sensorReadings));

    int retries = 0;
    bool acked = false;
    while (retries < 3 && !acked) {
      gotBatchAck = false;

      // Send batch with batch number as the last byte
      esp_err_t result = esp_now_send(relayMAC, packetBuffer, batchLen * sizeof(sensorReadings));
      if (result != ESP_OK) {
        Serial.println("esp_now_send failed!");
        retries++;
        delay(10);
        continue;
      }

      // Wait for ACK for this batch
      unsigned long start = millis();
      while ((!gotBatchAck || lastAckBatch != batchNum) && (millis() - start < 1000)) {
        delay(1);
      }
      if (gotBatchAck && lastAckBatch == batchNum) {
        acked = true;
      } else {
        Serial.print("Timeout waiting for batch ACK ");
        Serial.println(batchNum);
        retries++;
        delay(10);
      }
    }
    if (!acked) {
      Serial.print("Failed to send batch ");
      Serial.println(batchNum);
      return false;
    }
    batchNum++;
  }
  return true;
}

// Handshake response callback
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  Serial.print("Received something! Length: ");
  Serial.println(len);
  for (int i = 0; i < len; i++) {
    Serial.print(incomingData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  if (len == sizeof(uint32_t)) {
    uint32_t localTimeUnix;
    memcpy(&localTimeUnix, incomingData, sizeof(localTimeUnix));

    struct timeval now = {
      .tv_sec = localTimeUnix,
      .tv_usec = 0
    };
    settimeofday(&now, nullptr);

    Serial.print("Time set from relay: ");
    Serial.println(localTimeUnix);

    hasTimeBeenSet = true;
  }
  if (len == 3 && memcmp(incomingData, "ACK", 3) == 0) {
    gotAck = true;
    Serial.println("Received ACK from relay!");
    return;
  }
  // Per-batch ACK: "ACKB" + batch number (1 byte)
  if (len == 5 && memcmp(incomingData, "ACKB", 4) == 0) {
    gotBatchAck = true;
    lastAckBatch = incomingData[4];
    Serial.print("Received batch ACK for batch ");
    Serial.println(lastAckBatch);
    return;
  }
  if (len == 6 && memcmp(incomingData, "READY", 5) == 0) {
    relayReady = true;
    lastReadySeq = incomingData[5];
    Serial.println("Received READY signal from relay!");
    return;
  }
  if (len == 9 && memcmp(incomingData, "COMPLETE", 8) == 0) {
    relayComplete = true;
    lastCompleteSeq = incomingData[8];
    Serial.println("Received COMPLETE signal from relay!");
    return;
  }
}

bool waitForComplete(uint8_t seq, uint32_t timeout_ms = 8000) {
  relayComplete = false;
  lastCompleteSeq = 0;
  unsigned long start = millis();
  while ((!relayComplete || lastCompleteSeq != lastReadySeq) && (millis() - start < timeout_ms)) {
    delay(10);
  }
  return relayComplete && (lastCompleteSeq == lastReadySeq);
}

// Send data over ESP-NOW
bool sendData(sensorReadings* data, int count) {
  for (int i = 0; i < count; i++) {
    esp_err_t result = esp_now_send(relayMAC, (uint8_t*)&data[i], sizeof(sensorReadings));
    if (result != ESP_OK) {
      return false;
    }
    delay(2);  // Delay helps reduce chance of drops
  }
  return true;
}

// Send handshake request
bool sendHandshake() {
  gotAck = false;
  const char* helloMsg = "HELLO";
  esp_now_send(relayMAC, (uint8_t*)helloMsg, strlen(helloMsg));
  
  unsigned long start = millis();
  while (millis() - start < 200) {
    if (gotAck) return true;
    delay(10);
  }
  return false;
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
          esp_sleep_enable_timer_wakeup(1 * 1000000);
          esp_deep_sleep_start();
          delay(1000);
}

void killwifi() {
            WiFi.disconnect(); 
         // WiFi.mode(WIFI_OFF);
          //esp_wifi_stop();
         // adc_power_off();
}



bool waitForReady(uint32_t timeout_ms = 5000) {
  relayReady = false;
  requestSeq++;
  // send REQUEST with requestSeq
  uint8_t msg[8] = {'R','E','Q','U','E','S','T', requestSeq};
  esp_now_send(relayMAC, msg, 8);

  unsigned long start = millis();
  while ((!relayReady || lastReadySeq != requestSeq) && (millis() - start < timeout_ms)) {
    delay(10);
  }
  return relayReady && (lastReadySeq == requestSeq);
}



void transmitReadings() {
  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(0,0);

  if (readingCnt <= 0) return;

  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Waiting for relay ready...");
  Serial.print("Waiting for relay ready...");
  display.display();
  
  if (!waitForReady(8000)) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Relay not ready, saving to NVS...");
    Serial.print("Relay not ready, saving to NVS...");
    display.display();
    delay(1000);
    return; // Don't send data if relay isn't ready
  }
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Relay ready!");
  Serial.println("Relay ready!");

  display.print("TXing #");
  Serial.print("TXing #");
  display.println(arrayCnt);
  Serial.println(arrayCnt);
  display.display();

  sendDataBatched(Readings, readingCnt);
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
      uint8_t dummy = 0;
      esp_now_send(relayMAC, &dummy, sizeof(dummy));

      if (!hasTimeBeenSet) {
        display.println("Waiting for time from relay...");
        display.display();
        if (waitForTime(3000)) {
          display.println("Time successfully set from relay");
          display.display();
        } else {
          display.println("Timeout: did not receive time");
          display.display();
          // Optional: go to sleep or fail safe
        }
      }

          //initTime("EST5EDT,M3.2.0,M11.1.0");

          readingCnt = 0;
          delay(1);
          readingCnt = 0;
          delay(1);

          esp_sleep_enable_timer_wakeup(1 * 1000000);
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


      if (!sendHandshake()) {

        delay(1);
        ++arrayCnt;
        delay(1);
        prefs.putBytes(String(arrayCnt).c_str(), &Readings, sizeof(Readings));
        readingCnt = 0;
        killwifi();
        display.clearDisplay();   // clears the screen and buffer
        display.setCursor(0,0);
        display.print("Timed out, saving to NVS...");
        Serial.println("Timed out, saving to NVS...");
        display.display();
        delay(3000);
        esp_sleep_enable_timer_wakeup(1 * 1000000);
        esp_deep_sleep_start();
      }
      display.clearDisplay();   // clears the screen and buffer
      display.setCursor(0,0);
      display.print("Connected. Transmitting #0");
      Serial.println("Connected. Transmitting #0");
      display.display();
      transmitReadings();
        if (!waitForComplete(requestSeq, 8000)) {
          Serial.println("Did not receive COMPLETE from relay, aborting or retrying...");
          // Handle error: retry, save to NVS, etc.
          return;
        }
      delay(10);
      while (arrayCnt > 0) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.print("Waiting for relay ready...");
        Serial.println("Waiting for relay ready...");
        display.display();
        
        if (!waitForReady(8000)) {
          display.clearDisplay();
          display.setCursor(0,0);
          Serial.println("Relay not ready, saving to NVS...");
          display.print("Relay not ready, saving to NVS...");
          display.display();
          delay(3000);
          break; // Don't send data if relay isn't ready
        }
        delay(10);
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Relay ready!");
        Serial.println("Relay ready!");
        display.print("TXing #");
        Serial.print("TXing #");
        display.println(arrayCnt - 1);
        Serial.println(arrayCnt - 1);
        display.display();
        //delay(50);
        prefs.getBytes(String(arrayCnt).c_str(), &Readings, sizeof(Readings));
        sendDataBatched(Readings, maximumReadings);
        if (!waitForComplete(requestSeq, 8000)) {
          Serial.println("Did not receive COMPLETE from relay, aborting or retrying...");
          // Handle error: retry, save to NVS, etc.
          return;
        }  
        arrayCnt--;
        
      }
      arrayCnt = 0;
      readingCnt = -1;
      delay(1);
      arrayCnt = 0;
      readingCnt = -1;
      delay(1);
      lastReadySeq = 0;
      lastCompleteSeq = 0;
      display.clearDisplay();   // clears the screen and buffer
      display.setCursor(0,0);
      display.print("Done.  Closing connection...");
      Serial.println("Done.  Closing connection...");
      display.display();
      //conn.close();



      ESP.restart();
  } 


        gotosleep();

}


void loop()
{
gotosleep();
}
