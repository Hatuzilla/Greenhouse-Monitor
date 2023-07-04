#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "time.h"
#include "Wire.h"
#include "Adafruit_SHT31.h"
#include <SPI.h>
#include <WebSerial.h>

int count = 0; // Loop counter

#define WIFI_RETRY 5 // How many times should the WiFi try to connect?
#define NTP_RETRY 5  // How many times should the NTP server try to sync?

bool DEBUG = true; // Used for dubug messages

// Fan stuff
const int circulationFanPWMPin = 32; // PWM Control pin

const int fanRelaySignalPin = 19; // Relay control pin

int fanSpeed = 0;

String Circulation_fanStatus = "Setting up";

// Setting PWM properties
const int freq = 5000;
const int circulationFanChannel = 0;
const int intakeFanChannel = 1;
const int resolution = 8;

// Temp & Humidity Sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();
float currentTemp = 0;
float currentHumid = 0;
float averageDailyTemp = 0;
float averageDailyHumidity = 0;
float tempRunningTotal = 0;
float humidRunningTotal = 0;

String shtConnected;
bool enableHeater = false;
unsigned long heaterTimer = 0;
unsigned long heaterTimeStart = 0;
unsigned long heaterTimeFinish = 0;

// Wifi stuff
const char *ssid = "The Internet";
const char *password = "Password";

AsyncWebServer server(80); // OTA Async webserver

// Time from NTP server
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

// Date & Time variables
int yr = 0;
int mt = 0;
int dy = 0;
int hr = 0;
int mi = 0;
int se = 0;

char dateTime[24];
String dateTimeStamp = "null";
String espStartTimeStamp = "null";
uint8_t miCount = 1;
uint8_t nowMi = 0;
uint8_t nowDy = 0;

// Millis varariables
unsigned long millisNow = 0;
unsigned long millisPrev = 0;
unsigned long minuteCheck = 60000; // check time every 1 min

// Process bools
bool greenhouseProcess = true;
bool circulationFanStatus = false;

// Measure PV voltage
double voltage = 0;

//**fwd declerations**//

void setupTempHumid();
void pmwFanSetup();
void wifiSetup();
void fanStatus();
void setDateTime();
void checkTime();
void checkTempHumid();
void setupOtaUpdateandServer();
void requestTimeStamp();
void recvMsg(uint8_t *data, size_t len);
void setupWebSerial();
void sensorHeater();
void readPvVoltage();
void restart();

//---------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);

  wifiSetup(); // Initiate wifi

  delay(500);

  setDateTime(); // Set date and time from NTP server

  delay(500);

  pmwFanSetup(); // Set up fan control stuff

  delay(500);

  setupTempHumid(); // Set up temp and humidity sensor

  delay(500);

  setupOtaUpdateandServer(); // Set up OTA updates

  delay(500);

  setupWebSerial();

  delay(10);



}

//---------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------

void loop()
{
  checkTime(); // Check if 1 min has passed and start & check greenhouse processes
  // sensorHeater();

  if (greenhouseProcess == true)
  {
    count++;
    checkTempHumid();
    fanStatus();
    readPvVoltage();
    greenhouseProcess = false; // reset greenHouse Processes
  }
}

//---------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------

void wifiSetup()
{
  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi..");
  delay(10000);

  int wifi_retry_cnt = 0;
  char wifi_retry_message[50];

  if ((WiFi.status() == WL_CONNECTED))
  {
    Serial.println("...Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    delay(500);
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("...Can't connect to WiFi..trying again"); // Try again
    while ((WiFi.status() != WL_CONNECTED) && (wifi_retry_cnt < WIFI_RETRY))
    {
      delay(500);
      WiFi.begin(ssid, password);
      sprintf(wifi_retry_message, "Attempt %i of %i", wifi_retry_cnt, WIFI_RETRY);
      Serial.println(wifi_retry_message);
      wifi_retry_cnt++;
      delay(2000);
    }
  }
}

//---------------------------------------------

void setupTempHumid() // Start Temp & Humidity Sensor
{
  if (!sht31.begin(0x44))
  {
    shtConnected = "not connected";
  }

  else
  {
    shtConnected = "connected";
  }

  currentTemp = (sht31.readTemperature());
  currentHumid = (sht31.readHumidity());

  enableHeater = false;
  sht31.heater(enableHeater);
}

//---------------------------------------------

void setDateTime()
{
  if ((WiFi.status() == WL_CONNECTED))
  {

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    struct tm timeinfo;
    getLocalTime(&timeinfo);

    yr = timeinfo.tm_year + 1900;

    int ntp_retry_cnt = 0;
    char ntp_retry_message[50];

    if (yr == 1970)
    {
      Serial.println("Failed to retrieve NTP server time"); // Try again
      while ((yr == 1970) && (ntp_retry_cnt < NTP_RETRY))
      {
        Serial.print(".");
        delay(500);
        sprintf(ntp_retry_message, "Attempt %i of %i", ntp_retry_cnt, NTP_RETRY);
        Serial.println(ntp_retry_message);

        //struct tm timeinfo;
        getLocalTime(&timeinfo);

        yr = timeinfo.tm_year + 1900;
        ntp_retry_cnt++;
      }
    }

    if (yr != 1970)
    {

      //struct tm timeinfo;
      getLocalTime(&timeinfo);

      yr = timeinfo.tm_year + 1900;
      mt = timeinfo.tm_mon + 1;
      dy = timeinfo.tm_mday;
      hr = timeinfo.tm_hour;
      mi = timeinfo.tm_min;
      se = timeinfo.tm_sec;

      sprintf(dateTime, "%02i/%02i/%02i, %02i:%02i:%02i", dy, mt, yr, hr, mi, se);
      espStartTimeStamp = dateTime;
      Serial.println("NTP server sync successful");
    }
  }
}

//---------------------------------------------

void setupOtaUpdateandServer()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { requestTimeStamp(); 
             request->send(200, "text/plain", 
            ("Esp32 start date & time: " + espStartTimeStamp + 
            "\nEsp32 date & time now: " + dateTimeStamp +

            "\n\nThe circulations fans are: " + Circulation_fanStatus + 
            "\nFan speed set at :" + fanSpeed +

            "\n\nThe temp & humidity sensor is: " + shtConnected + 
            "\nThe temperature is: " + currentTemp + " C"+ 
            "\nThe humidity is: " + currentHumid + " RH"+ 
            "\nHumidity sensor heater is : " + enableHeater +

            "\n\nThe average daily temperature is: " + String(averageDailyTemp,2) + " C (Updated every minute)"+ //updated houly & reset at midnight
            "\nThe average daily Humidity is: " + String(averageDailyHumidity,2) + " %RH (Updated every minute)"+ //updated houly & reset at midnight

            "\nThe PV charging voltage is currently: " + String(voltage,2) + "v" +

            "\n\nLoop count: " + count +
            "\nFree heap: " + ESP.getFreeHeap()

            )); });

  AsyncElegantOTA.begin(&server); // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");
}

//---------------------------------------------

void checkTime()
{

  millisNow = millis();

  if ((millisNow - millisPrev) >= 200)
  { // check to see if it's a new minute every 200ms

    millisPrev = millis();

    struct tm timeinfo;
    getLocalTime(&timeinfo);
    mi = timeinfo.tm_min;

    if (mi != nowMi)
    {

      nowMi = mi;

      greenhouseProcess = true;
    }
  }
}

//---------------------------------------------

void pmwFanSetup()
{

  pinMode(fanRelaySignalPin, OUTPUT); // Pin to control circulation fan relay
  digitalWrite(fanRelaySignalPin, LOW);

  // configure circulation PWM functionalitites
  ledcSetup(circulationFanChannel, freq, resolution);

  // attach the channel to the GPIO to be circulation fan
  ledcAttachPin(circulationFanPWMPin, circulationFanChannel);
}

//---------------------------------------------

void fanStatus()
{

  // Check time and turn air circulation fans on or off

  if (hr >= 10 && hr <= 19)
  {
    digitalWrite(fanRelaySignalPin, HIGH);
    circulationFanStatus = true;
    Circulation_fanStatus = "ON";
    // Check temp and adjust intake fan

    if (currentTemp > 25 && currentTemp < 27)
    {
      digitalWrite(fanRelaySignalPin, HIGH);
      circulationFanStatus = true;
      Circulation_fanStatus = "ON";
      ledcWrite(circulationFanChannel, 50);
      fanSpeed = 50;
    }

    else if (currentTemp > 27 && currentTemp < 29)
    {
      ledcWrite(circulationFanChannel, 100);
      fanSpeed = 100;
    }

    else if (currentTemp >= 30)
    {
      ledcWrite(circulationFanChannel, 190);
      fanSpeed = 190;
    }

    else if (currentTemp <= 25)
    {
      digitalWrite(fanRelaySignalPin, LOW);
      circulationFanStatus = false;
      Circulation_fanStatus = "OFF";
    }
  }

  if ((hr > 19) || (hr < 10))

  {
    digitalWrite(fanRelaySignalPin, LOW);
    circulationFanStatus = false;
    Circulation_fanStatus = "OFF";
  }
}
//---------------------------------------------

void checkTempHumid() // Get Temp & Humidity Data
{

  currentTemp = (sht31.readTemperature());
  currentHumid = (sht31.readHumidity());

  tempRunningTotal += currentTemp;
  humidRunningTotal += currentHumid;

  averageDailyTemp = (tempRunningTotal / count);      // update average daily temp every min
  averageDailyHumidity = (humidRunningTotal / count); // update average daily humidity every min

  if (dy != nowDy) // midinght reset hour counter
  {
    miCount = 1;
    count = 1;
    nowDy = dy;

    tempRunningTotal = currentTemp;   // reset temp running total
    humidRunningTotal = currentHumid; // reset humid running total
  }
}

//---------------------------------------------

void sensorHeater() // turn on humidity evaporation heater every 10 min for 30sec (built innto sensor)
{

  heaterTimer = millis();

  if (((mi % 10 == 0) && (enableHeater == false)) || ((currentHumid >= 85) && (enableHeater == false))) // turn on every 10 mins
  {
    enableHeater = true;
    heaterTimeStart = millis();

    if (DEBUG)
    {
      Serial.println("Heater on");
    }
  }

  if (((heaterTimer - heaterTimeStart) >= 30000) && (enableHeater == true)) // turn off after 30 seconds
  {

    sht31.heater(enableHeater);

    heaterTimeFinish = millis();
  }

  if ((heaterTimer - heaterTimeFinish >= 30000) && (enableHeater == true))
  {
    enableHeater = false;
    sht31.heater(enableHeater);

    if (DEBUG)
    {
      Serial.println("Heater off");
    }
  }
}

//---------------------------------------------

void readPvVoltage()
{

  voltage = analogRead(33); // Voltage at pin as analoge value

  voltage = (-0.000000000000016 * pow(voltage, 4) + 0.000000000118171 * pow(voltage, 3) - 0.000000301211691 * pow(voltage, 2) + 0.001109019271794 * voltage + 0.034143524634089); // Voltage at pin using polynomial equation to adjust ESP32 ADC linearity for better accuracy

  voltage = (4095 / 3.3) * voltage; // Convert adjusted voltage back to analoge value

  voltage = (voltage) / 4095 * 30 * 247.191 / 220; // Convert to PV voltage
}

//---------------------------------------------

void setupWebSerial()
{
  WebSerial.begin(&server);
  WebSerial.msgCallback(recvMsg);
}

//---------------------------------------------

void requestTimeStamp()
{

  struct tm timeinfo;
  getLocalTime(&timeinfo);

  // Assign vaules to variables used to set RTC below
  yr = timeinfo.tm_year + 1900;
  mt = timeinfo.tm_mon + 1;
  dy = timeinfo.tm_mday;
  hr = timeinfo.tm_hour;
  mi = timeinfo.tm_min;
  se = timeinfo.tm_sec;

  sprintf(dateTime, "%02i/%02i/%02i, %02i:%02i:%02i", dy, mt, yr, hr, mi, se);
  dateTimeStamp = dateTime;
}

//---------------------------------------------

void recvMsg(uint8_t *data, size_t len)
{
  WebSerial.println("Received Data...");
  String inputString = "";
  for (int i = 0; i < len; i++)
  {
    inputString += char(data[i]);
  }
  WebSerial.println(inputString);
  if (inputString == "restart")
  {
    WebSerial.println("ESP Restarting");

    delay(1000);

    restart();
  }

  else
  {
    WebSerial.println("Unknown command");
  }
}

void restart()
{
  ESP.restart();
}
