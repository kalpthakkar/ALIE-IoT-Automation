#include <Arduino.h>

#ifdef ARDUINO_ARCH_ESP32
#include "WiFi.h"
#include "AsyncTCP.h"
#else
#include "ESP8266WiFi.h"
#include "ESPAsyncTCP.h"
#endif
#include "ESPAsyncWebServer.h"
#include "Adafruit_Sensor.h"
#include "DHT.h"
#include "WiFiClient.h"
#include "BlynkSimpleEsp32.h"
#include "HTTPClient.h"
#include "EEPROM.h"
#include "esp_task_wdt.h"

#define ESPALEXA_ASYNC
/* 
*  Set Maximum Devices that ESPAlexa could handle 
*  
*  Each device "slot" occupies memory, even if no device is initialized.
*  You can change the maximum number of devices by adding (#define ESPALEXA_MAXDEVICES 20)
*  This value can also be decreased to save memory if we are not using 10 devices at all.
*  
*  Each Group will also be considered as 1 uniquw devices (no matters multiple devices are associated inside or not)
*  Hint: For 8 channel relay, Max-Devices are set to 18 (8 individual channel + 10 groups)  
*/ 
#define ESPALEXA_MAXDEVICES  18
// #define ESPALEXA_MAXDEVICES  10      /* For 4 channel relay */
// #define ESPALEXA_MAXDEVICES  3       /* For 2 channel relay */
// #define ESPALEXA_MAXDEVICES  1       /* For 1 channel relay */

// #define ESPALEXA_DEBUG  false

#include "Espalexa.h"

RTC_NOINIT_ATTR int rtc_reset_cause = 0;

/* Handler for Light indicator status updating FreeRTOS Task */
TaskHandle_t LightControlTask;
/* Set GPOI_number of the LED pin (Light_indicator) */
const int ledPin = 2;  // 2 corresponds to GPIO2 (D2 in ESP32)
/* setting PWM properties */
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;
/*
*  0 => Reseting state (Waited Blink)
*  1 => WiFi Connected & Working (ON)
*  2 => WiFi Not Connected (Glow-Low)
*  3 => Sleep (OFF)
*  Else => (Fast Blink)
*/
int LightMode;

// Defining touch threshold to toggle deep sleep
#define TOUCH_THRESHOLD 20
// Set touch pin for touch-sensing to enable deep sleep
#define TOUCHPIN  4

// Set to true to define Relay as Normally Open (NO) : ( set false for Normally Closed (NC) )
#define RELAY_NO    true

// Set number of relays
#define NUM_RELAYS  8

// Assign each GPIO to a relay
int relayGPIOs[NUM_RELAYS] = {23, 22, 21, 19, 18, 5, 25, 26};

/*
*  Note that we are storing our network credentials and device_account related credentials 
*  inside EEPROM (Flash memory) for getting access to it even after device powered off (non-volatile)
*
*  This credentials will setup wirelessly without hard-coding. 
*/
/* network_credentials */
String ssid = "";
String password = "";
String ssid_list;
/* device_account credentials */
int authorization_portal; 
String esp_username = "";
String esp_password = "";
String esp_location = "";

// Naming the Access Point created by ESP during setup (Hotspot name)
#define Soft_AP_SSID  "Smart Module Setup"

/********************* Do not change *********************/
// For <input>:name="username" field validation in localIP/auth
// For <input>:name="password" field validation in localIP/auth
const char* USERNAME_PARAM = "username"; 
const char* PASSWORD_PARAM = "password";
/*********************************************************/

/********************* Do not change *********************/
/* 
*  For sending update request for passed RELAY_PARAM (1 <-range-> 8) & STATE_PARAM (0/1)
*  (1-8) : For 8 relay program (same applicable to others)
*  (0/1) : ON_state(0) & OFF_state(1)  --> For Normally_Open(NO) circuit (opposite for NC)
*/
const char* RELAY_PARAM = "relay";  
const char* STATE_PARAM = "state";
/*********************************************************/

/**************** WiFi Configuration : Static/Fixed IP ****************/
// Set the Static IP address
IPAddress local_IP(192, 168, 0, 100);
// Set the Gateway IP address
IPAddress gateway(192, 168, 0, 1);

IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional
/**********************************************************************/


/********************* WiFi Connectivity Controler *********************/
/* 
*  Will contineously try to re-connect wifi in loop as soon as WiFi gets 
*  disconnected or in counter loop after disconnected to WiFi.
*  1 unit = 1 second
*
*  To be very specific, there is a delay of 500ms (inside loop), so:
*  Actual $(reconnect_waiting_time) = $(1.5 * reconnect_waiting_time)
*/
#define reconnect_waiting_time  10    // (1 unit = 1 second)
/*
*  Define waiting time to re-try reconnecting to WiFi after failure from 
*  'contineously looped' $(reconnect_waiting_time) to save energy and create effeciency
*  
*  $(reconnection_timeout) is designed to not go to sleep mode before 
*  attempting for few mins (generally 3-4) and to avoid contineous looping.
*
*  This $(reconnection_timeout) will again try to re-connect to WiFi after 
*  specified period of time  ===> (1 unit = 1 min)
*  
*  To be very specific, adding $(1.5 * reconnect_waiting_time) to this which is 
*  actual waiting time of $(reconnect_waiting_time) whic gives the end result as:
*
*  Actual $(reconnection_timeout) = $(reconnection_timeout) + $(1.5 * reconnect_waiting_time)
*/
#define reconnection_timeout  1   // (1 unit = 1 min)
/* 
*  $(WiFi_maxConnectionTrigger) is the max number of time we want 
*  to allow $(reconnection_timeout) before entering to deep_sleep
*
*  Set maximum attempt limit to connect to WiFi
*  If crossed the limit, then go for timer based deep_sleep
*  
*  This will result in '''total time before going into deep sleep'''
*  Total time = 
*         ($(reconnect_waiting_time) unit [default: sec] 
*         + 
*         $(reconnection_timeout) unit [default: min]) [unit time]
*         *
*         $(WiFi_maxConnectionTrigger)
*
*  Actual Total time = 
*         ( ( 1.5 * $(reconnect_waiting_time) ) unit [default: sec] 
*           + 
*           $(reconnection_timeout) unit [default: min] ) [unit time]
*         *
*         $(WiFi_maxConnectionTrigger)
*/ 
#define WiFi_maxConnectionTrigger  3    // (iterations)
/* 
*  WiFi caused duration of deep sleep 
*  "duration_deep_sleep" unit depends on what you multiply with in below code.
*  Incase (duration_deep_sleep * 3600000000) => (1 unit = 1 hr)
*  Incase (duration_deep_sleep * 60000000) => (1 unit = 1 min)
*  Incase (duration_deep_sleep * 1000000) => (1 unit = 1 sec)
*  Default set to (1 unit = 1 min)
*/
#define duration_deep_sleep  25     // (1 unit = 1 min)
/*
*  Incase if the WiFi is again not recovered in next $(duration_deep_sleep) time period,
*  then it will again wait for 3-4 mins [3.75 mins using default settings] and will go 
*  back to bed for long run if still failed to connect in that short duration.
************************************************************************/

/***********************************************************************/
/* 
*  Reseting ESP session will be open for $(setup_session_timeout) unit of time
*  Once reached timeout, ESP will go into deep_sleep mode with TOUCHPIN abled to wake it up.
*/
#define setup_session_timeout  5    // (1 unit = 1 min)
/***********************************************************************/

// Set data pin to which DHT sensor is connected
#define DHTPIN 15     // Digital pin connected to the DHT sensor

/********* Uncomment the type of sensor in use: *********/
#define DHTTYPE    DHT11     // DHT 11
// #define DHTTYPE    DHT22     // DHT 22 (AM2302)
// #define DHTTYPE    DHT21     // DHT 21 (AM2301)
/*******************************************************/

/* 
*  Custom bottom-right icons for custom service
*  Suggested Icon Names: 
*  leaf (for gardening), 
*  paw (for pet food/drink automation), 
*  notifications (on door bell communication services), 
*  videocam (for monitoring), 
*  water, person, settings, home (...for other services...) 
*/
String feature_icon1 = "home";
String feature_icon2 = "leaf";
String feature_icon3 = "exit";

/* Comment this out to disable prints and save space */
// #define BLYNK_PRINT Serial
/* Fill-in your Template ID (only if using Blynk.Cloud) */
// #define BLYNK_TEMPLATE_ID   "YourTemplateID"

// Set your Blynk Auth Code
char authCode[] = "zi5Xs2bVaSp8JtZGy2lP9DhleuR0djFK";

/*
*  Alexa controling Device callback functions
*  
*  Instructions: device#Changed must be limited upto NUM_RELAYS
*  Example: For (NUM_RELAYS => 8), keep range of # from (1-8)
*  Do appropriate addition/deletion of this functions declaration & its definition part too... 
*/
void device1Changed(uint8_t brightness);
void device2Changed(uint8_t brightness);
void device3Changed(uint8_t brightness);
void device4Changed(uint8_t brightness);
void device5Changed(uint8_t brightness);
void device6Changed(uint8_t brightness);
void device7Changed(uint8_t brightness);
void device8Changed(uint8_t brightness);

/*
*  Alexa controling Group callback functions
*  
*  "group#Changed" must be declared as per below instructions:
*  NUM_RELAY 8 => (# range from 1-to-10)
*  NUM_RELAY 4 => (# range from 1-to-6)
*  NUM_RELAY 2 => (# be only 1 i.e. only group1Changed) 
*  Example: For (NUM_RELAYS => 8), keep range of # from (1-8), h 
*  Do appropriate addition/deletion of this functions declaration & its definition part too... 
*/
void group1Changed(uint8_t brightness);
void group2Changed(uint8_t brightness);
void group3Changed(uint8_t brightness);
void group4Changed(uint8_t brightness);
void group5Changed(uint8_t brightness);
void group6Changed(uint8_t brightness);
void group7Changed(uint8_t brightness);
void group8Changed(uint8_t brightness);
void group9Changed(uint8_t brightness);
void group10Changed(uint8_t brightness);

String Device_Name[NUM_RELAYS] = {"Light 1", "Light 2", "Light 3", "Light 4", "Light 5", "Light 6", "Light 7", "Light 8"};
void (*deviceChanged[NUM_RELAYS])(uint8_t) = {device1Changed, device2Changed, device3Changed, device4Changed, device5Changed, device6Changed, device7Changed, device8Changed};


/* 
*  We can not use if statement here, because #define is interpret by 
*  the preprocessor, and the output would be a wrong syntax. 
*/
#define NUM_GROUPS ((NUM_RELAYS == 8) ? 10 : (NUM_RELAYS == 4) ? 6 : (NUM_RELAYS == 2) ? 1 : 0)

String Group_Name[NUM_GROUPS] = {"Group 1", "Group 2", "Group 3", "Group 4", "Group 5", "Group 6", "Group 7", "Group 8", "Group 9", "Group 10"};
void (*groupChanged[NUM_GROUPS])(uint8_t) = {group1Changed, group2Changed, group3Changed, group4Changed, group5Changed, group6Changed, group7Changed, group8Changed, group9Changed, group10Changed};


Espalexa espalexa;
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

void callback(){}  // Function retuning as a callback for touchAttachInterrupt function

void goToDeepSleep(){  // Enable deep sleep mode using touch sensing
  if(touchRead(TOUCHPIN) < TOUCH_THRESHOLD){
    Serial.println("Touch Detect #1sec");
    delay(1000);
    if(touchRead(TOUCHPIN) < TOUCH_THRESHOLD){
      Serial.println("Touch Detect #2sec");
      delay(1000);
      if(touchRead(TOUCHPIN) < TOUCH_THRESHOLD){
        Serial.println("Touch Detect #3sec");
        delay(1000);
        if(touchRead(TOUCHPIN) < TOUCH_THRESHOLD){
          Serial.println("Touch Detect #4sec");
          delay(1000);
          if(touchRead(TOUCHPIN) < TOUCH_THRESHOLD){
            Serial.println("Factory Resetting ESP..."); // Factory reset on contineously_touching_pin for 5 sec 
            /*
            *  Address 161 (Stores value: 0/1) 
            *  For ESP_reset_status:disallow/allow for maintaining control flow into setup()
            *  Below line will disallow setup() for starting server and will 
            *  start setupAP() function to re-configure this ESP thing
            */
            EEPROM.write(161, 0);
            delay(500);
            EEPROM.commit();
            delay(500);
            ESP.restart();

            if(touchRead(TOUCHPIN) < TOUCH_THRESHOLD){
              // Same reseting part for precision
              EEPROM.write(161, 0);
              delay(500);
              EEPROM.commit();
              delay(500);
              ESP.restart();
            }
            else
            {
              EEPROM.write(161, 0);
              delay(500);
              EEPROM.commit();
              delay(500);
              ESP.restart();
            }
          }
          else
          {
            Serial.println("4th sec close-out");
          }
        }
        else
        {
          Serial.println("3rd sec close-out");
        }
      }
      else
      {
        Serial.println("2nd sec close-out");
      }
    }
    else
    {
      touchAttachInterrupt(TOUCHPIN, callback, TOUCH_THRESHOLD);
      esp_sleep_enable_touchpad_wakeup();
      LightMode = 3;
      Serial.println("Going to sleep...Zzz");
      esp_deep_sleep_start();
    }
  }
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void print_wifi_error(){
  switch(WiFi.status()) 
  {
    case WL_IDLE_STATUS : Serial.println("WL_IDLE_STATUS"); break;
    case WL_NO_SSID_AVAIL : Serial.println("WL_NO_SSID_AVAIL"); break;
    case WL_CONNECT_FAILED : Serial.println("WL_CONNECT_FAILED"); break;
    case WL_DISCONNECTED : Serial.println("WL_DISCONNECTED"); break;
    default : Serial.printf("Unknown WiFi issue"); break;
  }
}

void LightController( void * pvParameters ){      // Light_Indicator handler using FreeRTOS 
  Serial.print("Light Controller Task is running on core ");
  Serial.println(xPortGetCoreID());
  /*
  *  0 => Reseting state (Waited Blink)
  *  1 => WiFi Connected & Working (ON)
  *  2 => WiFi Not Connected (Glow-Low)
  *  3 => Sleep (OFF)
  *  Else => (Fast Blink)
  */
  for(;;){
    if(LightMode == 0)
    {
      // Serial.println("LightMode(0): Waited Bling");
      while(LightMode == 0)
      {
        ledcWrite(ledChannel, 255);
        delay(150);
        ledcWrite(ledChannel, 0);
        delay(1200);
      }
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    else if(LightMode == 1)
    {
      // Serial.print("LightMode(1): ON");
      ledcWrite(ledChannel, 255);
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    else if(LightMode == 2)
    {
      // Serial.println("LightMode(2): Glow");
      // increase the LED brightness

      for(int dutyCycle = 0; dutyCycle <= 255; dutyCycle+=3){   
        // changing the LED brightness with PWM
        ledcWrite(ledChannel, dutyCycle);
        delay(15);
      }
    
      // decrease the LED brightness
      for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle-=3){
        // changing the LED brightness with PWM
        ledcWrite(ledChannel, dutyCycle);   
        delay(15);
      }
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    else if(LightMode == 3)
    {
      // Serial.println("LightMode(3): OFF"); 
      ledcWrite(ledChannel, 0);
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    else
    {
      // Serial.println("LightMode(Cofiguring/Unknown state): Fast Bling");
      while((LightMode != 0) && (LightMode != 1) && (LightMode != 2) && (LightMode != 3))
      {
        ledcWrite(ledChannel, 255);
        delay(250);
        ledcWrite(ledChannel, 0);
        delay(250);
      }
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    /**************************************
    *   Normal Blinking (Light Mode)
    *   Can be added incase to implement new indication functionality 
    ************ Code Below ***************/
    // Serial.println("Normal Bling");
    // while(true)
    // {
    //   ledcWrite(ledChannel, 255);
    //   delay(1000);
    //   ledcWrite(ledChannel, 0);
    //   delay(1000);
    // }
    // vTaskDelay(2000 / portTICK_PERIOD_MS);
    /**************************************/
  }
}

void autoConnectToWiFi( void * parameters ) {  // WiFi connection handler using FreeRTOS
  int wifi_failure_counter = 0;
  for(;;) {
    vTaskDelay(10);
    if(WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi is connected");
      if(LightMode != 1)
      {
        LightMode = 1;
      }
      vTaskDelay(25000 / portTICK_PERIOD_MS);  // connection check every 25 sec
      continue;
    }

    // If disconnected, then we start re_connection to a WiFi network
    Serial.println("WiFi not connected");
    LightMode = 2;
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());

    unsigned long startAttemptTime = millis();

    /* 
    *  Keeps looping while we're not connected & 
    *  have not reached timeout of $(reconnect_waiting_time)sec [default:10 sec]
    *  Note: with a delay of 500ms (inside loop), actual time is (1.5 * reconnect_waiting_time)sec [default:15 sec]
    */
    while(WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < (reconnect_waiting_time * 1000)) {
      vTaskDelay(500);  // (500 ms)
      Serial.print(".");
    }

    // When we failed to make a WiFi connection within $(reconnect_waiting_time) [default:10sec | actual: 15sec]
    if(WiFi.status() != WL_CONNECTED) {

      Serial.println("Failed to connect WiFi");
      if(LightMode != 2) { LightMode = 2; }
      
      /* Trigger Deep sleep if counter exceeds the provided limit in $(WiFi_maxConnectionTrigger) [default: 3 iterations] */
      wifi_failure_counter += 1;
      if(WiFi_maxConnectionTrigger < wifi_failure_counter) {
        /* 
        *  Ultimate Guide for handling deep_sleep & wake_up_cause :
        *  https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/  
        */
        Serial.print("print_wifi_error() says: ");
        print_wifi_error();
        Serial.println("Sleep a little and retry later, bye");
        /*
        *  Next we decide what all peripherals to shut down/keep on
        *  By default, ESP32 will automatically power down the peripherals
        *  not needed by the wakeup source, but if you want to be a poweruser
        *  this is for you.
        *  https://readthedocs.org/projects/esp-idf-zh/downloads/pdf/latest/
        *  OR
        *  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/sleep_modes.html
        *  Left the line commented as an example of how to configure peripherals.
        *  In ESP32, RTC fast memory will always be kept enabled in deep sleep.
        *  The line below turns off all RTC peripherals in deep sleep.
        *  (Arg1 -> domain): power domain to configure
        *  (Arg2 -> option): power down option (ESP_PD_OPTION_OFF, ESP_PD_OPTION_ON, or ESP_PD_OPTION_AUTO))
        *  Returns:
        *    ESP_OK on success
        *    ESP_ERR_INVALID_ARG if either of the arguments is out of range
        */
        // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
        // Serial.println("Configured all RTC Peripherals to be powered down in sleep");
        /* 
        *  Set the wakeup (Alarm like)
        *  Below function accepts sleep time in microseconds.
        *  1000000 value can be used for us_to_s (microsec_to_sec) conversion 
        *  Note: (u in us stands for the notation mue)
        *  60000000 value can be used for us_to_min (microsec_to_min) conversion
        */
        esp_sleep_enable_timer_wakeup(duration_deep_sleep * 60000000);
        /* 
        *  Ask processor to sleep now...Zzz
        *  Below is the link of power consumption data in each of 
        *  (active/modem sleep/light sleep/deep sleep/hibernation) state
        *  Link: https://lastminuteengineers.com/esp32-sleep-modes-power-consumption/
        */
        delay(1000);
        Serial.flush();

        /* Enabling touch_to_wakeup feature */
        touchAttachInterrupt(TOUCHPIN, callback, TOUCH_THRESHOLD);
        esp_sleep_enable_touchpad_wakeup();
        LightMode = 3;
        Serial.println("Touch WakeUp is enabled. You can always awake ESP anytime");
        esp_deep_sleep_start();
      }

      // re_Run the loop by providing recovery time to WiFi for 1 mins
      vTaskDelay(reconnection_timeout * 60000 / portTICK_PERIOD_MS);
      continue;
    }
    else
    {
      LightMode = 1;
    }

    // Got WiFi connection back within 20 sec (if:not failed)
    Serial.println("");
    Serial.println("WiFi Connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  } 
}

/* Creating instance by entering valid (DHT_GPIO_PIN) & (DHT_SensorName/DHT_Sensor_Type) */
DHT dht(DHTPIN, DHTTYPE);

/* Returns DHT sensor Temperature reading */
String readDHTTemperature() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float t = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(t)) {    
    Serial.println("Failed to read from DHT sensor!");
    return "--";
  }
  else {
    Serial.println(t);
    return String(t);
  }
}

/* Returns DHT sensor Humidity reading */
String readDHTHumidity() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  if (isnan(h)) {
    Serial.println("Failed to read from DHT sensor!");
    return "--";
  }
  else {
    Serial.println(h);
    return String(h);
  }
}

/* Function for ESPalexa to change device state */
void changeDeviceState(int RelayPin, uint8_t brightness) {
  //Control the device
  digitalWrite(RelayPin, (brightness == 255)? ((RELAY_NO)?LOW:HIGH) : ((RELAY_NO)?HIGH:LOW));
}

/* Function for ESPalexa to change group state */
void changeGroupState(String groupName, uint8_t brightness) {
  // relayGPIOs are set on 0 based indexing 
  int* groupGPIOs;
  int devices_in_group = 0;
  if(NUM_RELAYS == 8)   // Associating "# Group GPIO Pins" for 8 channel relay
  {
    // Assigning relay GPIO_Pins to Group
    if(groupName == Group_Name[0]) {                // Group 1 | 8 Channel
      Serial.println("Group 1 | 8 Channel");
      groupGPIOs = (int*)calloc(2, sizeof(int));
      groupGPIOs[0] = relayGPIOs[0];
      groupGPIOs[1] = relayGPIOs[1];
      devices_in_group = 2;
    }
    else if(groupName == Group_Name[1]) {           // Group 2 | 8 Channel
      Serial.println("Group 2 | 8 Channel");
      groupGPIOs = (int*)calloc(2, sizeof(int));
      groupGPIOs[0] = relayGPIOs[2];
      groupGPIOs[1] = relayGPIOs[3];
      devices_in_group = 2;
    }
    else if(groupName == Group_Name[2]) {           // Group 3 | 8 Channel
      Serial.println("Group 3 | 8 Channel");
      groupGPIOs = (int*)calloc(2, sizeof(int));
      groupGPIOs[0] = relayGPIOs[4];
      groupGPIOs[1] = relayGPIOs[5];
      devices_in_group = 2;
    }
    else if(groupName == Group_Name[3]) {           // Group 4 | 8 Channel
      Serial.println("Group 4 | 8 Channel");
      groupGPIOs = (int*)calloc(2, sizeof(int));
      groupGPIOs[0] = relayGPIOs[6];
      groupGPIOs[1] = relayGPIOs[7];
      devices_in_group = 2;
    }
    else if(groupName == Group_Name[4]) {           // Group 5 | 8 Channel
      Serial.println("Group 5 | 8 Channel");
      groupGPIOs = (int*)calloc(4, sizeof(int));
      groupGPIOs[0] = relayGPIOs[0];
      groupGPIOs[1] = relayGPIOs[1];
      groupGPIOs[2] = relayGPIOs[2];
      groupGPIOs[3] = relayGPIOs[3];
      devices_in_group = 4;
    }
    else if(groupName == Group_Name[5]) {           // Group 6 | 8 Channel
      Serial.println("Group 6 | 8 Channel");
      groupGPIOs = (int*)calloc(4, sizeof(int));
      groupGPIOs[0] = relayGPIOs[2];
      groupGPIOs[1] = relayGPIOs[3];
      groupGPIOs[2] = relayGPIOs[4];
      groupGPIOs[3] = relayGPIOs[5];
      devices_in_group = 4;
    }
    else if(groupName == Group_Name[6]) {           // Group 7 | 8 Channel
      Serial.println("Group 7 | 8 Channel");
      groupGPIOs = (int*)calloc(4, sizeof(int));
      groupGPIOs[0] = relayGPIOs[4];
      groupGPIOs[1] = relayGPIOs[5];
      groupGPIOs[2] = relayGPIOs[6];
      groupGPIOs[3] = relayGPIOs[7];
      devices_in_group = 4;
    }
    else if(groupName == Group_Name[7]) {           // Group 8 | 8 Channel
      Serial.println("Group 8 | 8 Channel");
      groupGPIOs = (int*)calloc(6, sizeof(int));
      groupGPIOs[0] = relayGPIOs[0];
      groupGPIOs[1] = relayGPIOs[1];
      groupGPIOs[2] = relayGPIOs[2];
      groupGPIOs[3] = relayGPIOs[3];
      groupGPIOs[4] = relayGPIOs[4];
      groupGPIOs[5] = relayGPIOs[5];
      devices_in_group = 6;
    }
    else if(groupName == Group_Name[8]) {           // Group 9 | 8 Channel
      Serial.println("Group 9 | 8 Channel");
      groupGPIOs = (int*)calloc(6, sizeof(int));
      groupGPIOs[0] = relayGPIOs[2];
      groupGPIOs[1] = relayGPIOs[3];
      groupGPIOs[2] = relayGPIOs[4];
      groupGPIOs[3] = relayGPIOs[5];
      groupGPIOs[4] = relayGPIOs[6];
      groupGPIOs[5] = relayGPIOs[7];
      devices_in_group = 6;
    }
    else {  // Group_Name[9] ==> All Devices        // Group 10 | 8 Channel
      Serial.println("Group 10 | 8 Channel");
      groupGPIOs = (int*)calloc(8, sizeof(int));
      groupGPIOs[0] = relayGPIOs[0];
      groupGPIOs[1] = relayGPIOs[1];
      groupGPIOs[2] = relayGPIOs[2];
      groupGPIOs[3] = relayGPIOs[3];
      groupGPIOs[4] = relayGPIOs[4];
      groupGPIOs[5] = relayGPIOs[5];
      groupGPIOs[4] = relayGPIOs[6];
      groupGPIOs[5] = relayGPIOs[7];
      devices_in_group = 8;
    }
  }
  else if(NUM_RELAYS == 4)
  {
    // Assigning relay GPIO_Pins to Group
    if(groupName == Group_Name[0]) {                // Group 1 | 4 Channel
      Serial.println("Group 1 | 4 Channel");
      groupGPIOs = (int*)calloc(2, sizeof(int));
      groupGPIOs[0] = relayGPIOs[0];
      groupGPIOs[1] = relayGPIOs[1];
      devices_in_group = 2;
    }
    else if(groupName == Group_Name[1]) {           // Group 2 | 4 Channel
      Serial.println("Group 2 | 4 Channel");
      groupGPIOs = (int*)calloc(2, sizeof(int));
      groupGPIOs[0] = relayGPIOs[1];
      groupGPIOs[1] = relayGPIOs[2];
      devices_in_group = 2;
    }
    else if(groupName == Group_Name[2]) {           // Group 3 | 4 Channel
      Serial.println("Group 3 | 4 Channel");
      groupGPIOs = (int*)calloc(2, sizeof(int));
      groupGPIOs[0] = relayGPIOs[2];
      groupGPIOs[1] = relayGPIOs[3];
      devices_in_group = 2;
    }
    else if(groupName == Group_Name[3]) {           // Group 4 | 4 Channel
      Serial.println("Group 4 | 4 Channel");
      groupGPIOs = (int*)calloc(3, sizeof(int));
      groupGPIOs[0] = relayGPIOs[0];
      groupGPIOs[1] = relayGPIOs[1];
      groupGPIOs[2] = relayGPIOs[2];
      devices_in_group = 3;
    }
    else if(groupName == Group_Name[4]) {           // Group 5 | 4 Channel
      Serial.println("Group 5 | 4 Channel");
      groupGPIOs = (int*)calloc(3, sizeof(int));
      groupGPIOs[0] = relayGPIOs[1];
      groupGPIOs[1] = relayGPIOs[2];
      groupGPIOs[2] = relayGPIOs[3];
      devices_in_group = 3;
    }
    else {  // Group_Name[5] ==> All Devices        // Group 6 | 4 Channel
      Serial.println("Group 6 | 4 Channel");
      groupGPIOs = (int*)calloc(4, sizeof(int));
      groupGPIOs[0] = relayGPIOs[0];
      groupGPIOs[1] = relayGPIOs[1];
      groupGPIOs[2] = relayGPIOs[2];
      groupGPIOs[3] = relayGPIOs[3];
      devices_in_group = 4;
    }
  }
  else if(NUM_RELAYS == 2)
  {
    Serial.println("Group 1 | 2 Channel");
    groupGPIOs = (int*)calloc(2, sizeof(int));      // Group 1 | 2 Channel
    groupGPIOs[0] = relayGPIOs[0];
    groupGPIOs[1] = relayGPIOs[1];
    devices_in_group = 2;
  }
  else
  {
    Serial.println("Number of Relays neither from 8, 4, nor 2. Hence group can't be created");
  }

  // Main loop for switching state of all devices associated to group
  for(int i=0; i<devices_in_group; i++)
  {
    Serial.println(groupGPIOs[i] + " ||| " + (brightness == 255) ? ((RELAY_NO)?LOW:HIGH) : ((RELAY_NO)?HIGH:LOW));
    digitalWrite(groupGPIOs[i], (brightness == 255) ? ((RELAY_NO)?LOW:HIGH) : ((RELAY_NO)?HIGH:LOW));
  }
  free(groupGPIOs); // Freeing dynamic memory allocated using calloc
}

/* Device controling ESPalexa function */
void device1Changed(uint8_t brightness) { changeDeviceState(relayGPIOs[0], brightness); }
void device2Changed(uint8_t brightness) { changeDeviceState(relayGPIOs[1], brightness); }
void device3Changed(uint8_t brightness) { changeDeviceState(relayGPIOs[2], brightness); }
void device4Changed(uint8_t brightness) { changeDeviceState(relayGPIOs[3], brightness); }
void device5Changed(uint8_t brightness) { changeDeviceState(relayGPIOs[4], brightness); }
void device6Changed(uint8_t brightness) { changeDeviceState(relayGPIOs[5], brightness); }
void device7Changed(uint8_t brightness) { changeDeviceState(relayGPIOs[6], brightness); }
void device8Changed(uint8_t brightness) { changeDeviceState(relayGPIOs[7], brightness); }

/* Group controling ESPalexa function */
void group1Changed(uint8_t brightness)  { changeGroupState(Group_Name[0], brightness); }
void group2Changed(uint8_t brightness)  { changeGroupState(Group_Name[1], brightness); }
void group3Changed(uint8_t brightness)  { changeGroupState(Group_Name[2], brightness); }
void group4Changed(uint8_t brightness)  { changeGroupState(Group_Name[3], brightness); }
void group5Changed(uint8_t brightness)  { changeGroupState(Group_Name[4], brightness); }
void group6Changed(uint8_t brightness)  { changeGroupState(Group_Name[5], brightness); }
void group7Changed(uint8_t brightness)  { changeGroupState(Group_Name[6], brightness); }
void group8Changed(uint8_t brightness)  { changeGroupState(Group_Name[7], brightness); }
void group9Changed(uint8_t brightness)  { changeGroupState(Group_Name[8], brightness); }
void group10Changed(uint8_t brightness) { changeGroupState(Group_Name[9], brightness); }

const char configured_setup[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>Smart Home</title>  
  <style>
  *,::after,::before{margin:0;padding:0;box-sizing:inherit}html{box-sizing:border-box;font-size:40%;overflow-y:scroll}body{background-color:#eee;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,"Helvetica Neue",Arial,"Noto Sans",sans-serif,"Apple Color Emoji","Segoe UI Emoji","Segoe UI Symbol","Noto Color Emoji"}.container{min-height:30vh;margin:6%;display:flex;justify-content:center;align-items:center;font-family:Poppins,sans-serif;background:#eee}.components{width:120rem;height:99rem;border-radius:3rem;box-shadow:.8rem .8rem 1.4rem #c8d0e7,-.2rem -.2rem 1.8rem #fff;padding:4rem;align-items:center}.components span{font-size:8.4rem;display:inline-block;margin-top:8rem}.components b{background-color:red;background-image:linear-gradient(to right,#7b4397,#a1338a,#c01d73,#d40d55,#dc2430);background-size:100%;background-repeat:repeat;-webkit-background-clip:text;-webkit-text-fill-color:transparent;-moz-background-clip:text;-moz-text-fill-color:transparent}h2{color:#000}.alert{position:relative;margin-bottom:1rem;border:1px solid transparent}.alert-info{color:#0c5460;background-color:#d1ecf1;border-color:#bee5eb}.alert-info .alert-link{color:#062c33}.alert-link{font-weight:700}a{color:#007bff;text-decoration:none;background-color:transparent}a:hover{text-decoration:underline}.btn-success{color:#fff;background-color:#28a745;border:none;cursor:pointer}@media (min-width:768px) and (max-width:980px){h2{font-size:9rem;margin-top:18rem;margin-bottom:3rem}.alert-info{font-size:6rem;width:90%;text-align:left;padding:4rem 6rem;border-radius:4rem}.components b{font-size:8.4rem}#showSteps{margin-top:5rem}.btn-success{font-size:9rem;width:60rem;height:18rem;border-radius:100px;margin-top:9rem}svg{width:14rem;height:14rem;padding-top:3.5rem;margin-right:2.4rem;margin-top:-3rem}#openLink{margin-top:6rem}}@media screen and (orientation:landscape){html{font-size:65%}.components{width:52rem;height:35rem;border-radius:3rem;box-shadow:.8rem .8rem 1.4rem #c8d0e7,-.2rem -.2rem 1.8rem #fff;padding:4rem;align-items:center}.container{min-height:10vh;max-height:37vh;margin:6%}#showSteps{margin-top:.5rem}.components span{font-size:3rem;display:inline-block;margin-top:2.5rem;display:flex;justify-content:center}.components i{margin-right:2rem}.components b{font-size:3.3rem}h2{font-size:7rem;font-weight:600;margin-top:2.5rem;margin-bottom:1.5rem}.alert-info{font-size:2rem;width:50%;text-align:left;border-radius:1rem;padding:1rem 1.75rem}.btn-success{font-size:2.5rem;width:15rem;height:5rem;border-radius:100px;margin-top:2rem}svg{width:6.4rem;height:6.4rem;padding-top:2.5rem;margin-right:1rem;margin-top:-3rem}.components span{margin-top:0}#instructions span{margin-top:2.5rem;margin-right:1.5rem}}
  </style>
</head>
<body>
<center>

    <h2>All set to be by your side</h2>

    <div class="alert alert-info">
        <strong>Info!</strong> 
        Incorrect network cretentials will result in again looping-up the entire setup process.
        <br>
        <div style="margin-top: 8px;">
                You can always <a href="#" class="alert-link">re-configure this device</a> incase needed.
        </div>
    </div>

    <div class="container">
        <div class="components">
            
            <div id="showSteps">
                <b>Steps remaining</b>
            </div>
            
            <br>

            <div id="instructions">
            <span id="connectWiFi">
                <svg fill="currentColor" class="bi bi-wifi" viewBox="0 0 16 16">
                <path d="M15.384 6.115a.485.485 0 0 0-.047-.736A12.444 12.444 0 0 0 8 3C5.259 3 2.723 3.882.663 5.379a.485.485 0 0 0-.048.736.518.518 0 0 0 .668.05A11.448 11.448 0 0 1 8 4c2.507 0 4.827.802 6.716 2.164.205.148.49.13.668-.049z" stroke="black" stroke-width="0.4"/>
                <path d="M13.229 8.271a.482.482 0 0 0-.063-.745A9.455 9.455 0 0 0 8 6c-1.905 0-3.68.56-5.166 1.526a.48.48 0 0 0-.063.745.525.525 0 0 0 .652.065A8.46 8.46 0 0 1 8 7a8.46 8.46 0 0 1 4.576 1.336c.206.132.48.108.653-.065zm-2.183 2.183c.226-.226.185-.605-.1-.75A6.473 6.473 0 0 0 8 9c-1.06 0-2.062.254-2.946.704-.285.145-.326.524-.1.75l.015.015c.16.16.407.19.611.09A5.478 5.478 0 0 1 8 10c.868 0 1.69.201 2.42.56.203.1.45.07.61-.091l.016-.015zM9.06 12.44c.196-.196.198-.52-.04-.66A1.99 1.99 0 0 0 8 11.5a1.99 1.99 0 0 0-1.02.28c-.238.14-.236.464-.04.66l.706.706a.5.5 0 0 0 .707 0l.707-.707z" stroke="black" stroke-width="0.4"/>
                </svg>Connect to `TARGET_SSID`
            </span>
            
            <br>
            
            <span id="openLink">
                <svg fill="currentColor" class="bi bi-link-45deg" viewBox="0 0 16 16">
                <path d="M4.715 6.542 3.343 7.914a3 3 0 1 0 4.243 4.243l1.828-1.829A3 3 0 0 0 8.586 5.5L8 6.086a1.002 1.002 0 0 0-.154.199 2 2 0 0 1 .861 3.337L6.88 11.45a2 2 0 1 1-2.83-2.83l.793-.792a4.018 4.018 0 0 1-.128-1.287z" stroke="black" stroke-width="0.4"/>
                <path d="M6.586 4.672A3 3 0 0 0 7.414 9.5l.775-.776a2 2 0 0 1-.896-3.346L9.12 3.55a2 2 0 1 1 2.83 2.83l-.793.792c.112.42.155.855.128 1.287l1.372-1.372a3 3 0 1 0-4.243-4.243L6.586 4.672z" stroke="black" stroke-width="0.4"/>
                </svg>Open below link
            </span>
            </div>
            <br>
            `TARGET_WEBPAGE`
        </div>
    </div>    
</center>
</body>
</html>
)rawliteral";

const char auth[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <title>Smart Home</title>
  <link href="https://fonts.googleapis.com/css?family=Poppins:400,600,700&display=swap" rel="stylesheet">
  <link href="https://fonts.googleapis.com/icon?family=Material+Icons" rel="stylesheet">
  <script src="https://kit.fontawesome.com/f0601b0fb2.js" crossorigin="anonymous"></script>
  <script type='module' src='https://unpkg.com/ionicons@5.0.0/dist/ionicons/ionicons.esm.js'></script>
  <script nomodule='' src='https://unpkg.com/ionicons@5.0.0/dist/ionicons/ionicons.js'></script>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" type="text/css" href="http://combinatronics.com/kalpthakkar/041ea0cc940dd085fddc0b0975907fbd/raw/97bafb6735b79a0b3715510976ab332f4cc08976/auth-style.css">
</head>
<body>
<div class="container">
  <div class="components">
    <form method='POST' action="/">
      <div class="person">
        <input name="username" type="text" class="person__input" placeholder="Username">
        <div class="person__icon">
          <ion-icon name="person"></ion-icon>
        </div>
      </div>
      <br>
      <div class="key">
        <input name="password" type="password" class="key__input" placeholder="Password">
        <div class="key__icon">
          <ion-icon name="key"></ion-icon>
        </div>
      </div>
      <br><br>
        <input type="submit" value="Submit" class="btn btn__primary">
    </form>
  </div>
</div>
</body>
</html>
)rawliteral";

const char home_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'>
<title>Smart Home</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link href="https://fonts.googleapis.com/css?family=Poppins:400,600,700&display=swap" rel="stylesheet">
<script type='module' src='https://unpkg.com/ionicons@5.0.0/dist/ionicons/ionicons.esm.js'></script>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/normalize/5.0.0/normalize.min.css">
<!-- **************************** Main Style **************************** -->
<link rel="stylesheet" type="text/css" href="https://combinatronics.com/kalpthakkar/9566b8b6c30c93077cf495dc4a48f181/raw/a3d8112cc1e80434e2de9f1b21fdb92441314f1e/home-page-main-layout.css">
<!-- **************************** Clock Section **************************** -->
<link rel="stylesheet" type="text/css" href="https://combinatronics.com/kalpthakkar/2654d77099dca1ad9dbf2ac992d13e46/raw/d5fd3ac82fe2b8a804e98d2dbc008bb1eafdab4a/clock-section.css">
<!-- **************************** Toggle Switch Buttons **************************** -->
<link rel="stylesheet" type="text/css" href="https://combinatronics.com/kalpthakkar/cc64a4e16a33152b0c3def76fa34565e/raw/f06682be605812d65538960b6d3965af5c141409/toggleCheckbox-section.css">
<!-- **************************** BUTTON (Deep Sleep & Location buttons) **************************** -->
<link rel="stylesheet" type="text/css" href="https://combinatronics.com/kalpthakkar/48f9d7bdd71438cfee1d23715ac45539/raw/8f58aa60716aa1572a6179d3c854f6c94863a4ef/home-page-buttons.css">
<!-- **************************** Segmented Control **************************** -->
<link rel="stylesheet" type="text/css" href="https://combinatronics.com/kalpthakkar/9d18e78bd4ee3d9a06212ccf74378e46/raw/0211db7dfddda7b791be63df722f5b773712b8b0/segmented-controller.css">
<!-- **************************** Slider Section **************************** -->
<link rel="stylesheet" type="text/css" href="https://combinatronics.com/kalpthakkar/cee9175cb51e9e618c4b801d1c8ef63d/raw/a3fe5e1fb0e78ad86b1c9096facd82c050f3b752/slider-section.css">
<!-- **************************** Featured Icons Section **************************** -->
<link rel="stylesheet" type="text/css" href="https://combinatronics.com/kalpthakkar/fed4b87a2db6549624c72774b6d8e216/raw/37799b3f547f90a83037a43458e7cd14d9acb6ff/featured-icons.css">
<!-- **************************** Sense Segment (Temperature & Humidity) **************************** -->
<!--  Icons of Temperature & Humidity  -->
<link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css" integrity="sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr" crossorigin="anonymous">
<link rel="stylesheet" type="text/css" href="https://combinatronics.com/kalpthakkar/b994f213672e8f52de96604497fcff14/raw/485643804d4fdee84845078d1dd92981abaca284/dht-sensor-segment.css">
<!-- **************************** Play Segment (Play & Pause Button) **************************** -->
<link rel="stylesheet" type="text/css" href="https://combinatronics.com/kalpthakkar/74316399793aab3ff180438e85c12dda/raw/f7df5658f2f99e97d87cc208c13fdc77cb2b60bb/play-segment.css">
<!-- **************************** A/C Segment Style **************************** --> 
<link rel="stylesheet" type="text/css" href="https://combinatronics.com/kalpthakkar/d9a340430b9658ffa422bcae95d0923b/raw/92ff5e2ddf0c99d1c861065353f32cff099fea8d/ac-segment.css">
<!-- **************************** Favico.ico **************************** -->
<link rel='icon' type='image/x-icon' href='https://www.drupal.org/favicon.ico' />
<!-- <link rel="icon" type="image/png" href="#"/> -->
</head>
<body>
<br>
<div class="container">
  <div class="components">
    
    <div class="switch">
      `BUTTONPLACEHOLDER`
    </div>
    
    <form method='POST' action="/deepsleep">
      <button class="btn btn__primary" type="submit" style="border: none; margin-left: 2rem;"><p>Deep Sleep</p></button>
    </form>
    
    <div class="btn btn__secondary"><p>`LOCATION`</p></div>
    
    <div class="clock">
      <div class="hand hours"></div>
      <div class="hand minutes"></div>
      <div class="hand seconds"></div>
      <div class="point"></div>
      <div class="marker">
        <span class="marker__1"></span>
        <span class="marker__2"></span>
        <span class="marker__3"></span>
        <span class="marker__4"></span>
      </div>
    </div>

    <div class="segmented-control">
      <input type="radio" name="radio2" onclick="currentSlide(1)" value="3" id="tab-1" checked/>
      <label for="tab-1" class= "segmented-control__1">
        <p>Sense</p>
      </label>
      
      <input type="radio" name="radio2" onclick="currentSlide(2)" value="4" id="tab-2" />
      <label for="tab-2" class= "segmented-control__2">
        <p>Play</p></label>
      
      <input type="radio" name="radio2" onclick="currentSlide(3)" value="5" id="tab-3" />
      <label for="tab-3" class= "segmented-control__3">
        <p>A/C</p></label>
      
      <div class="segmented-control__color"></div>
    </div>
    
    
    <div class="slider-content">
        
      <div class="mySlides fade">
        <div class="sense">
          <p>
            <span class="dht-labels">Temperature</span>
            <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
            
            <span id="temperature">`TEMPERATURE`</span>
            <sup class="units">&deg;C</sup>
          </p>
          <br>
          <p>
            <span class="dht-labels" id="humidity-text">Humidity</span>
            <i class="fas fa-tint" style="color:#00add6;"></i> 
            <span id="humidity">`HUMIDITY`</span>
            <sup class="units">&percnt;</sup>
          </p>
        </div>
      </div>
    
      <div class="mySlides fade">
        <div class="circle">
          <span class="circle__btn">
            <ion-icon class="pause" name="pause"></ion-icon>
            <ion-icon class="play" name="play"></ion-icon>
          </span>
          <span class="circle__back-1"></span>
          <span class="circle__back-2"></span>
        </div>
      </div>

      <div class="mySlides fade">
        <div class="temp">
          <div class="temp__dial">
            <div class="temp__drag"></div>
            <div class="temp__dial-shades">
              <div class="temp__shade-cold"></div>
              <div class="temp__shade-hot"></div>
            </div>
            <div class="temp__dial-core"></div>
            <div class="temp__value">
              <span class="temp__digit">1</span><span class="temp__digit">8</span>°
            </div>
          </div>
          
          <div class="temp__outdoors">
            <div class="temp__outdoors-col">
              <small class="temp__heading">Outside</small>
              <br>
              <span class="temp__o-value">0°</span>
            </div>
            <div class="temp__outdoors-col">
              <small class="temp__heading">Humidity</small>
              <br>
              <span class="temp__o-value">0%</span>
            </div>
          </div>
        </div>
      </div>
      
    </div>
    
    <!-- Slider bar -->
    <div class="slider">
      <div class="slider__box">
        <span class="slider__btn" id="find"></span>
        <span class="slider__color"></span>
        <span class="slider__tooltip">50%</span>
      </div>
    </div>

    <!-- 3 Icons (bottom-right-corner) -->
    <div class="icon">
      `ICONSPLACEHOLDER`
    </div>
    
  </div>
</div>
<!-- Clock Section JS -->
<script type="text/javascript" src="https://combinatronics.com/kalpthakkar/1814421df25c1d0b03fed01071bfd34f/raw/3d8c573d36b7744742dd1da701c946a32535fe5f/clock-section.js"></script>
<!-- Toggle Checkbox -->
<script type="text/javascript" src="https://combinatronics.com/kalpthakkar/f496ba08b11bbacf37af379a08595ceb/raw/b642cfd41cbb29e76c60778bfe5568fe7be8692e/toggleCheckbox-section.js"></script>
<!-- Slider Section JS -->
<script type="text/javascript" src="https://combinatronics.com/kalpthakkar/7a0c290407fa5a98475ed10f5bf8ecb1/raw/f69f53e017cae78f291a9f9f76c03367cb46cf8f/slider-section.js"></script>
<!-- Segmented controller JS -->
<script type="text/javascript" src="https://combinatronics.com/kalpthakkar/7764cdd7492df69827c4045156ef1998/raw/0fecc1809ea6bb60ad146f13a569a8a98a432b2b/segmented-controller.js"></script>
<!-- DHT Sensor Segment JS -->
<script type="text/javascript" src="https://combinatronics.com/kalpthakkar/6acaff836b493a8dcdb4f82906f79528/raw/026e4790e026835b24f16fd6b6d689c1cd931add/dht-sensor-segment.js"></script>
<!-- Play Segment JS -->
<script type="text/javascript" src="https://combinatronics.com/kalpthakkar/515e9f96ac894ff582cc6317fc0764cb/raw/b6b5114caad2020fbe0a61ecf54f3a1c3265e08f/play-segment.js"></script>
<!-- AC Segment JS -->
<script src='https://unpkg.co/gsap@3/dist/gsap.min.js'></script>
<script src='https://unpkg.com/gsap@3/dist/Draggable.min.js'></script>
<script type="text/javascript" src="https://combinatronics.com/kalpthakkar/69ebbbf3682fd8e06638ae45ca81b8d2/raw/2d6926a97097acd0b022695c198dca1dc3d0fdcc/ac-segment.js"></script>
</body>
</html>
)rawliteral";

const char deep_sleep[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>Smart Home</title>
  <link rel="stylesheet" href="https://stackpath.bootstrapcdn.com/bootstrap/4.3.1/css/bootstrap.min.css" integrity="sha384-ggOyR0iXCbMQv3Xipma34MD+dH/1fQ784/j6cY/iJTQUOhcWr7x9JvoRxT2MZw1T" crossorigin="anonymous">
  <link rel="stylesheet" type="text/css" href="http://combinatronics.com/kalpthakkar/773fec892e16af792e5798d68e338a35/raw/bc0111f0be9eaa263b442eb08b5f2057c3c3d543/deep-sleep-style.css">
  <script src="https://ajax.googleapis.com/ajax/libs/jquery/3.5.1/jquery.min.js"></script>
  <script>
  $.get("https://gist.githubusercontent.com/kalpthakkar/1291610d5134fe272de0752e74fd4fb2/raw/867ce4382b9bedfbb4498599a063f0ce5f0c1906/deep-sleep-struct1.html", function(response) { 
  document.getElementById('mainFrame1').innerHTML= response;
  });
  </script>
</head>
<body>
<div id="mainFrame1"></div>
<!-- How to wakeUp -->
<div class="modal fade" id="ModalLong-2" tabindex="-1" role="dialog" aria-labelledby="ModalLong-2Title" aria-hidden="true" style="margin:0; z-index:1050;">
  <div class="modal-dialog" role="document" style="margin:0; justify-self: center; z-index:1050;">
    <div class="modal-content">
      <div class="modal-header">
        <h5 class="modal-title" id="ModalLong-2Title">How to wakeUp?</h5>
        <button type="button" class="close" data-dismiss="modal" aria-label="Close">
          <span aria-hidden="true">&times;</span>
        </button>
      </div>
      <div class="modal-body">
          
      <i class="fas fa-microchip"></i><span>Go to controller</span><br>
      <i class="fas fa-fingerprint"></i><span>Tap on touch sensor</span><br>
      <i class="fas fa-paper-plane"></i> `WAKEUP_WEBPAGE`<br>
      </div>
      <div class="modal-footer">
        <button type="button" class="btn btn-secondary" data-dismiss="modal">Close</button>
      </div>
    </div>
  </div>
</div>
<script src="https://cdnjs.cloudflare.com/ajax/libs/popper.js/1.14.7/umd/popper.min.js" integrity="sha384-UO2eT0CpHqdSJQ6hJty5KVphtPhzWj9WO1clHTMGa3JDZwrnQq4sF86dIHNDz0W1" crossorigin="anonymous"></script>
<script src="https://stackpath.bootstrapcdn.com/bootstrap/4.3.1/js/bootstrap.min.js" integrity="sha384-JjSmVgyd0p3pXB1rRibZUAYoIIy6OrQ6VrjIEaFf/nJGzIxFDsf4x0xIM+B07jRM" crossorigin="anonymous"></script>
<script src="https://kit.fontawesome.com/e706aa2dd0.js" crossorigin="anonymous"></script>
</body>
</html>
)rawliteral";

void eeprom_writeString(char address,String data)
{
  int data_size = data.length();
  for(int i=0; i<data_size; i++)
  {
    EEPROM.write(address+i,data[i]);
  }
  EEPROM.write(address+data_size,'\0');   // Adding termination null character for String Data
  /* 
  *  Data can be read from flash as many times as we want.
  *  But most devices are designed for about 100,000 to 1,000,000 write/erase operations
  * 
  *  Here EEPROM.commit(); is commented to avoid commiting each operation
  *  Must be self-declared at the end of every group of operations where this function is implemented
  */
  // EEPROM.commit();  // Save changes
}

String eeprom_readString(char address)
{
  char data[100]; //Max 100 Bytes
  int len=0;
  unsigned char temp;
  temp=EEPROM.read(address);
  while(temp != '\0' && len<65)   // Read until null character
  {    
    temp=EEPROM.read(address+len);
    data[len]=temp;
    len++;
  }
  data[len]='\0';
  return String(data);
}

// returns "checked" or "" style to html toggle_switch buttons when called by processor()
String relayState(int numRelay){
  if(RELAY_NO){  // For Normally Open circuit
    if(digitalRead(relayGPIOs[numRelay-1])){
      return "";
    }
    else {
      return "checked";
    }
  }
  else {  // For Normally Closed circuit
    if(digitalRead(relayGPIOs[numRelay-1])){
      return "checked";
    }
    else {
      return "";
    }
  }
  return "";
}

// Replaces all placeholder with buttons/sensor_reading/icons/etc in your web page
String processor(const String& var){
  //Serial.println(var);
  if(var == "BUTTONPLACEHOLDER"){
    String buttons ="";
    for(int i=1; i<=NUM_RELAYS; i++){
      String relayStateValue = relayState(i);
      buttons+= "<div class=\"toggle-switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"" + String(i) + "\" "+ relayStateValue + "><label for=\"" + String(i) + "\" "+ relayStateValue + "></label></div>";
    }
    return buttons;
  }
  else if(var == "LOCATION"){
    return esp_location;
  }
  else if(var == "TEMPERATURE"){
    return readDHTTemperature();
  }
  else if(var == "HUMIDITY"){
    return readDHTHumidity();
  }
  else if(var == "ICONSPLACEHOLDER"){
    String icons_template = "";
    icons_template+= "<div class=\"icon__one\"><ion-icon name=\"" + feature_icon1 + "\"></ion-icon></div><div class=\"icon__two\"><ion-icon name=\"" + feature_icon2 + "\"></ion-icon></div><a href=\"/auth\"><div class=\"icon__three\"><ion-icon name=\"" + feature_icon3 + "\"></ion-icon></div></a>";
    return icons_template;
  }
  else if(var == "WAKEUP_WEBPAGE"){
    String url = "http://" + String(local_IP[0]) + '.' + String(local_IP[1]) + '.' + String(local_IP[2]) + '.' + String(local_IP[3]);
    if(EEPROM.read(160) == 1){url += "/auth";}
    return "<a href=\"" + url + "\"><button type=\"button\" class=\"btn btn-light\">Happy Controlling</button></a>";
  }
  else if(var == "TARGET_SSID"){
    return ('"' + eeprom_readString(0) + '"');
  }
  else if(var == "TARGET_WEBPAGE"){
    String url = "http://" + String(local_IP[0]) + '.' + String(local_IP[1]) + '.' + String(local_IP[2]) + '.' + String(local_IP[3]);
    if(EEPROM.read(160) == 1){url += "/auth";}
    return "<a href=\"" + url + "\"><button type=\"button\" class=\"btn btn-success\">Explore</button></a>";
  }
  return String();
}

bool testWifi(void)
{
  int c = 0;
  Serial.println("Waiting for Wifi to connect");
  while ( c < 20 ) 
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      return true;
    }
    delay(500);
    Serial.print("*");
    c++;
  }
  Serial.println("");
  Serial.println("Connecting period timed out, opening Access-Point");
  return false;
}

void createWebServer()
{
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {

      String wifi_configuration_html ="<!DOCTYPE html> <html> <head> <meta charset='UTF-8'> <title>Smart Home</title> <link href=\"https://fonts.googleapis.com/css?family=Poppins:400,600,700&display=swap\" rel=\"stylesheet\"> <link href=\"https://fonts.googleapis.com/icon?family=Material+Icons\" rel=\"stylesheet\"> <script src=\"https://kit.fontawesome.com/f0601b0fb2.js\" crossorigin=\"anonymous\"></script> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"> <style type=\"text/css\"> :root{--primary-light:#8abdff;--primary:#6d5dfc;--primary-dark:#5b0eeb;--white:#FFFFFF;--greyLight-1:#E4EBF5;--greyLight-2:#c8d0e7;--greyLight-3:#bec8e4;--greyDark:#9baacf}*,::after,::before{margin:0;padding:0;box-sizing:inherit}html{box-sizing:border-box;font-size:62.5%;overflow-y:scroll;background:var(--greyLight-1)}.container{min-height:76vh;display:flex;justify-content:center;align-items:center;font-family:Poppins,sans-serif;background:var(--greyLight-1)}.alert{width:80%;padding:20px;margin-top:2.5rem;margin-bottom:0;background-image:linear-gradient(to right,#7b4397,#a1338a,#c01d73,#d40d55,#dc2430);color:#fff;border-radius:9px}.closebtn{margin-left:15px;color:#fff;font-weight:700;float:right;font-size:22px;line-height:20px;cursor:pointer;transition:.3s}.closebtn:hover{color:#000}#error_list p{margin-top:.6rem;font-family:Verdana,sans-serif}";
      wifi_configuration_html +="media screen and (min-width:600px){html{font-size:85%;margin-top:4rem;margin-bottom:4rem}.container{min-height:90vh}.alert{width:50%;margin-top:0;margin-bottom:2.5rem}}.components{width:28.5rem;height:50.6rem;border-radius:3rem;box-shadow:.8rem .8rem 1.4rem var(--greyLight-2),-.2rem -.2rem 1.8rem var(--white);padding:4rem;display:grid;grid-template-columns:17.6rem 19rem 20.4rem;grid-template-rows:repeat(autofit,-webkit-min-content);grid-template-rows:repeat(autofit,min-content);grid-column-gap:0;grid-row-gap:2.5rem;align-items:center}form{grid-column:1/1;grid-row:2/3;margin-top:-3rem}.inputIcon{position:relative;display:flex;align-items:center;width:25.4rem}.inputIcon__input{width:20.4rem;height:4rem;border:none;border-radius:1rem;font-size:1.4rem;padding-left:3.8rem;box-shadow:inset .2rem .2rem .5rem var(--greyLight-2),inset -.2rem -.2rem .5rem var(--white);background:0 0;font-family:inherit;color:var(--greyDark)}.inputIcon__input::-moz-placeholder{color:var(--greyLight-3)}.inputIcon__input:-ms-input-placeholder{color:var(--greyLight-3)}.inputIcon__input::placeholder{color:var(--greyLight-3)}.inputIcon__input:focus{outline:0;box-shadow:.3rem .3rem .6rem var(--greyLight-2),-.2rem -.2rem .5rem var(--white)}.inputIcon__input:focus+.inputIcon__icon{color:var(--primary)}.inputIcon__icon{height:2rem;position:absolute;font-size:2rem;padding:0 1rem;display:flex;color:var(--greyDark);transition:.3s ease}.radio{display:grid;grid-template-columns:repeat(2,10.4em);justify-items:center}.radio input{display:none}.radio__1 input:checked~label,.radio__2 input:checked~label{box-shadow:inset .2rem .2rem .5rem var(--greyLight-2),inset -.2rem -.2rem .5rem var(--white)}.radio__1 input:checked~label::after,.radio__2 input:checked~label::after{background:var(--primary)}.radio__1 label,.radio__2 label{box-shadow:.3rem .3rem .6rem var(--greyLight-2),-.2rem -.2rem .5rem var(--white);position:relative;display:flex;justify-content:center;align-items:center;cursor:pointer;width:2.8rem;height:2.8rem;border-radius:50%}.radio__1 label:hover::after,.radio__2 label:hover::after{background:var(--primary)}.radio__1 label::after,.radio__2 label::after{content:\"\";position:absolute;width:1.4rem;height:1.4rem;background:var(--greyDark);border-radius:50%;transition:.3s ease}.btn{width:15rem;height:4rem;font-size:1.6rem;border-radius:9rem;box-shadow:.3rem .3rem .6rem var(--greyLight-2),-.2rem -.2rem .5rem var(--white);position:relative;justify-self:center;display:flex;align-items:center;justify-content:center;cursor:pointer;transition:.3s ease;border:none;margin-left:3rem}.btn__primary{background:var(--primary);box-shadow:inset .2rem .2rem 1rem var(--primary-light),inset -.2rem -.2rem 1rem var(--primary-dark),.3rem .3rem .6rem var(--greyLight-2),-.2rem -.2rem .5rem var(--white);color:var(--greyLight-1)}.btn__primary:hover{color:var(--white)}.btn__primary:active{box-shadow:inset .2rem .2rem 1rem var(--primary-dark),inset -.2rem -.2rem 1rem var(--primary-light)}.btn__info{margin:1rem 0 0 .3rem;padding:0 1rem;height:3.3rem;width:20rem;pointer-events:none;background:#24f024;box-shadow:inset .2rem .2rem 1rem #94ff94,inset -.2rem -.2rem 1rem #00ad00,.3rem .3rem .6rem var(--greyLight-2),-.2rem -.2rem .5rem var(--white);color:#fff}.btn__info:hover{color:#fff}.btn__info:active{box-shadow:inset .2rem .2rem 1rem #008011,inset -.2rem -.2rem 1rem #a3ffaf}.btn__info p{font-weight:600;font-size:1.2rem;padding-left:1rem}select{-webkit-appearance:none;-moz-appearance:none;appearance:none}.inputIcon__icon svg{margin-top:.15rem;width:1.8rem;height:1.8rem} </style> <script type=\"text/javascript\"> var create_portal=!0;function toggleField(e,t){e.disabled=!0,e.style.display=\"none\",t.disabled=!1,t.style.display=\"inline\",t.focus(),t.setAttribute(\"style\",\"width: 20.4rem;height: 4rem;border: none;border-radius: 1rem;font-size: 1.4rem;padding-left: 3.8rem;box-shadow: inset 0.2rem 0.2rem 0.5rem var(--greyLight-2), inset -0.2rem -0.2rem 0.5rem var(--white);background: none;font-family: inherit;color: var(--greyDark);outline: none;\")}function showPortal(){document.getElementById(\"portal\").style.display=\"block\",document.getElementById(\"component\").setAttribute(\"style\",\"height: 50.6rem;\"),document.getElementById(\"input_username\").attributes.required=\"required\",document.getElementById(\"input_password\").attributes.required=\"required\",create_portal=!0}function hidePortal(){document.getElementById(\"portal\").style.display=\"none\",document.getElementById(\"component\").setAttribute(\"style\",\"height: 40.6rem;\"),document.getElementById(\"input_username\").attributes.required=\"\",document.getElementById(\"input_password\").attributes.required=\"\",create_portal=!1}function validateForm(){var e=0,t=document.getElementById(\"error_list\");if(t.innerHTML=\"\",document.getElementById(\"select_custom_ssid\").value.length>32&&(t.innerHTML+=\"<p>Invalid Network Name</p>\",e+=1),(n=document.forms.setup.networkPassword.value).length>64&&(t.innerHTML+=\"<p>Invalid Network Password</p>\",e+=1),create_portal&&\"none\"!=document.getElementById(\"portal\").style.display){var n,r=document.forms.setup.deviceUsername.value;r.length<3&&(t.innerHTML+=\"<p>Username must be greater that 2 characters</p>\",e+=1),r.length>20&&(t.innerHTML+=\"<p>Username must be less than 20 letters</p>\",e+=1),r.includes(\" \")&&(t.innerHTML+=\"<p>Username cannot contain spaces</p>\",e+=1),(n=document.forms.setup.devicePassword.value).length<4&&(t.innerHTML+=\"<p>Account password must be at least 4 characters long</p>\",e+=1),n.length>30&&(t.innerHTML+=\"<p>Account password must max contain 30 letters</p>\",e+=1),n.includes(\" \")&&(t.innerHTML+=\"<p>Account password cannot contain spaces</p>\",e+=1)}return document.getElementById(\"select_custom_location\").value.length>13&&(t.innerHTML+=\"<p>Location must be named within 13 characters</p>\",e+=1),document.getElementById(\"select_location\").value.includes(\"default\")&&document.getElementById(\"select_custom_location\").disabled&&(t.innerHTML+=\"<p>Select this device location where it has been setup</p>\",e+=1),0==e||(document.getElementById(\"display_errors\").style.display=\"block\",!1)}window.onload=function(){document.getElementById(\"display_errors\").style.display=\"none\"}; </script> </head> <body> <center> <div class=\"alert\" id=\"display_errors\"> <span class=\"closebtn\" onclick=\"this.parentElement.style.display='none'; \">&times;</span> <h3><strong>Errors!</strong></h3> <div id=\"error_list\"> </div> </div> </center> <div class=\"container\"> <div class=\"components\" id=\"component\"> <form name=\"setup\" method='POST' action=\"/setup\" onsubmit=\"return validateForm()\"> <div class=\"inputIcon\"> <select class=\"inputIcon__input\" name=\"networkName\" onchange=\"if(this.options[this.selectedIndex].value=='customSSID'){toggleField(this,this.nextSibling); this.selectedIndex='0';}\" required> \"" + ssid_list + "\" <option value=\"customSSID\">Custom</option> </select><input name=\"networkName\" style=\"display:none;\" id=\"select_custom_ssid\" disabled=\"disabled\" onblur=\"if(this.value==''){toggleField(this,this.previousSibling);}\"> <div class=\"inputIcon__icon\" > <svg xmlns=\"http://www.w3.org/2000/svg\" width=\"16\" height=\"16\" fill=\"currentColor\" class=\"bi bi-rss-fill\" viewBox=\"0 0 16 16\"> <path d=\"M2 0a2 2 0 0 0-2 2v12a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V2a2 2 0 0 0-2-2H2zm1.5 2.5c5.523 0 10 4.477 10 10a1 1 0 1 1-2 0 8 8 0 0 0-8-8 1 1 0 0 1 0-2zm0 4a6 6 0 0 1 6 6 1 1 0 1 1-2 0 4 4 0 0 0-4-4 1 1 0 0 1 0-2zm.5 7a1.5 1.5 0 1 1 0-3 1.5 1.5 0 0 1 0 3z\"/> </svg> </div> </div> <br> <div class=\"inputIcon\"> <input name=\"networkPassword\" type=\"password\" class=\"inputIcon__input\" placeholder=\"WiFi password\" required> <div class=\"inputIcon__icon\"> <svg xmlns=\"http://www.w3.org/2000/svg\" width=\"16\" height=\"16\" fill=\"currentColor\" class=\"bi bi-shield-lock-fill\" viewBox=\"0 0 16 16\"> <path fill-rule=\"evenodd\" d=\"M8 0c-.69 0-1.843.265-2.928.56-1.11.3-2.229.655-2.887.87a1.54 1.54 0 0 0-1.044 1.262c-.596 4.477.787 7.795 2.465 9.99a11.777 11.777 0 0 0 2.517 2.453c.386.273.744.482 1.048.625.28.132.581.24.829.24s.548-.108.829-.24a7.159 7.159 0 0 0 1.048-.625 11.775 11.775 0 0 0 2.517-2.453c1.678-2.195 3.061-5.513 2.465-9.99a1.541 1.541 0 0 0-1.044-1.263 62.467 62.467 0 0 0-2.887-.87C9.843.266 8.69 0 8 0zm0 5a1.5 1.5 0 0 1 .5 2.915l.385 1.99a.5.5 0 0 1-.491.595h-.788a.5.5 0 0 1-.49-.595l.384-1.99A1.5 1.5 0 0 1 8 5z\"/> </svg> </div> </div> <br> <div class=\"btn btn__info\"> <svg xmlns=\"http://www.w3.org/2000/svg\" width=\"23\" height=\"23\" fill=\"currentColor\" class=\"bi bi-layers-fill\" viewBox=\"0 0 16 16\"> <path d=\"M7.765 1.559a.5.5 0 0 1 .47 0l7.5 4a.5.5 0 0 1 0 .882l-7.5 4a.5.5 0 0 1-.47 0l-7.5-4a.5.5 0 0 1 0-.882l7.5-4z\"/> <path d=\"m2.125 8.567-1.86.992a.5.5 0 0 0 0 .882l7.5 4a.5.5 0 0 0 .47 0l7.5-4a.5.5 0 0 0 0-.882l-1.86-.992-5.17 2.756a1.5 1.5 0 0 1-1.41 0l-5.17-2.756z\"/> </svg> <p>Create Login Portal ?</p> </div> <br> <div class=\"radio\"> <div class=\"radio__1\"> <input id=\"radio-1\" type=\"radio\" name=\"radio\" value=\"1\" checked=\"checked\" onchange=\"showPortal()\"> <label for=\"radio-1\"></label> </div> <div class=\"radio__2\"> <input id=\"radio-2\" type=\"radio\" name=\"radio\" value=\"0\" onchange=\"hidePortal(this.value)\"> <label for=\"radio-2\"></label> </div> </div> <br> <div id=\"portal\"> <div class=\"inputIcon\"> <input name=\"deviceUsername\" type=\"text\" id=\"input_username\" class=\"inputIcon__input\" placeholder=\"Set Username\"> <div class=\"inputIcon__icon\"> <svg xmlns=\"http://www.w3.org/2000/svg\" style=\"width: 2rem; height: 2rem; margin-top: 0.10rem; margin-left: 0.05rem;\" fill=\"currentColor\" class=\"bi bi-person-fill\" viewBox=\"0 0 16 16\"> <path d=\"M3 14s-1 0-1-1 1-4 6-4 6 3 6 4-1 1-1 1H3zm5-6a3 3 0 1 0 0-6 3 3 0 0 0 0 6z\"/> </svg> </div> </div> <br> <div class=\"inputIcon\"> <input name=\"devicePassword\" type=\"password\" id=\"input_password\" class=\"inputIcon__input\" placeholder=\"Set Password\"> <div class=\"inputIcon__icon\"> <svg xmlns=\"http://www.w3.org/2000/svg\" style=\"width: 2rem; margin-left: 0.15rem;\" fill=\"currentColor\" class=\"bi bi-unlock-fill\" viewBox=\"0 0 16 16\"> <path d=\"M11 1a2 2 0 0 0-2 2v4a2 2 0 0 1 2 2v5a2 2 0 0 1-2 2H3a2 2 0 0 1-2-2V9a2 2 0 0 1 2-2h5V3a3 3 0 0 1 6 0v4a.5.5 0 0 1-1 0V3a2 2 0 0 0-2-2z\"/> </svg> </div> </div> </div> <br> <div class=\"inputIcon\"> <select class=\"inputIcon__input\" name=\"deviceLocation\" onchange=\"if(this.options[this.selectedIndex].value=='customOption'){toggleField(this,this.nextSibling); this.selectedIndex='0';}\" id=\"select_location\" required> <option value=\"default\" disabled=\"disabled\" selected>Select Room</option> <option value=\"Bedroom\">Bedroom</option> <option value=\"Dining\">Dining</option> <option value=\"Drawing Room\">Drawing Room</option> <option value=\"Garden\">Garden</option> <option value=\"Garrage\">Garrage</option> <option value=\"Hall\">Hall</option> <option value=\"Kitchen\">Kitchen</option> <option value=\"Living Room\">Living Room</option> <option value=\"Lobby\">Lobby</option> <option value=\"Outdoor\">Outdoor</option> <option value=\"Parking\">Parking</option> <option value=\"customOption\">Custom Name</option> </select><input name=\"deviceLocation\" style=\"display:none;\" id=\"select_custom_location\" disabled=\"disabled\" onblur=\"if(this.value==''){toggleField(this,this.previousSibling);}\"> <div class=\"inputIcon__icon\" > <svg xmlns=\"http://www.w3.org/2000/svg\" style=\"margin-top: 0.10rem; margin-left: 0.05rem;\" fill=\"currentColor\" class=\"bi bi-geo-alt-fill\" viewBox=\"0 0 16 16\"> <path d=\"M8 16s6-5.686 6-10A6 6 0 0 0 2 6c0 4.314 6 10 6 10zm0-7a3 3 0 1 1 0-6 3 3 0 0 1 0 6z\"/> </svg> </div> </div> <br> <br> <input type=\"submit\" value=\"Confirm\" class=\"btn btn__primary\"> </form> </div> </div> </body> </html>";
      request->send(200, "text/html", wifi_configuration_html);
    });

    server.on("/setup", HTTP_POST, [](AsyncWebServerRequest *request) {

      String ssid_query;
      String pass_query;
      int create_portal = 0;
      String username_query;
      String passphrase_query;
      String location_query;

      if(request->hasParam("networkName", true) && request->hasParam("networkPassword", true) && request->hasParam("deviceLocation", true)) 
      {
        ssid_query = request->getParam("networkName", true)->value();
        pass_query = request->getParam("networkPassword", true)->value();
        create_portal = atoi(request->getParam("radio", true)->value().c_str());
        /* Assigning username and password query only if user has agreed to create authorization protal */
        if(create_portal == 1)
        {
          username_query = request->getParam("deviceUsername", true)->value();
          passphrase_query = request->getParam("devicePassword", true)->value();
        }
        location_query = request->getParam("deviceLocation", true)->value();

        Serial.println("====================== PRINTING QUERIES: ======================");
        Serial.print("ssid_query: ");
        Serial.println(ssid_query);
        Serial.print("pass_query: ");
        Serial.println(pass_query);
        Serial.print("create_portal: ");
        Serial.println(create_portal);
        Serial.print("username_query: ");
        Serial.println(username_query);
        Serial.print("passphrase_query: ");
        Serial.println(passphrase_query);
        Serial.print("location_query: ");
        Serial.println(location_query);

        /* Cleaning EEPROM from (0<-to->161 address from total of 512) for fresh input */
        Serial.println("Clearing eeprom");
        for (int i = 0; i < 162; ++i) 
        {
          EEPROM.write(i, 0);
        }
        Serial.println("Caught SSID_QUERY: " + ssid_query);
        Serial.println("Caught PASS_QUERY: " + pass_query);
        Serial.println("");
        
        /* 
        *  Function takes only the base address to write 
        *  and will add it to memory as per length_of_data. 
        * 
        *  If length_of_data is found to be less_than max_char_allowed, 
        *  then remaingin memory address in boundation will be unused.
        *  
        *  This boundation/max_char_error_handling will be done on web_page itself.
        */
        Serial.println("Writing form values to EEPROM:");
        eeprom_writeString(0, ssid_query);            // Address 0 <-> 32 (Max len: 32 char for network_ssid)
        eeprom_writeString(33, pass_query);           // Address 33 <-> 96 (Max len: 64 char for network_passowrd)
        if(create_portal == 1)
        {
          eeprom_writeString(97, username_query);     // Address 97 <-> 116 (Max len: 20 char for esp_username)
          eeprom_writeString(117, passphrase_query);  // Address 117 <-> 146 (Max len: 30 char for esp_password)
        }
        eeprom_writeString(147, location_query);      // Address 147 <-> 166 (Max len: 13 char for esp_location)
        EEPROM.write(160, create_portal);             // Address 160 (Stores value: 0/1) (Max len: 1 int for portal_status:off/on)
        EEPROM.write(161, 1);                         // Address 161 (Stores value: 0/1) (Max len: 1 int for ESP_reset_status:disallow/allow for maintaining control flow into setup())
        delay(2000);
        EEPROM.commit();  // Save changes
        Serial.println("Commited EEPROM changes");

        Serial.println("Sending request to server...");
        /* 
        *  Processor display SSID & Target_webpage according to portal opened/closed 
        *  Thus there need commit of networkName & radio param before sending request to display webpage accordingly
        */
        request->send_P(200, "text/html", configured_setup, processor);
        Serial.println("Request Sent.");

        delay(500);
        server.end();
        Serial.println("Server ended");

        Serial.println("ESP restart");
        delay(1000);
        ESP.restart();
      } 
      else 
      {
        Serial.println("Sending 404: Params are not set properly");
        request->send(404, "text/html", "{\"Error\":\"404 not found it\"}");
      }
      DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    });
   
}

void launchWeb()
{
  Serial.println("");
  if (WiFi.status() == WL_CONNECTED){
    Serial.println("Status: WiFi Connected");
  } else {
    Serial.println("Status: WiFi not Connected");
  }
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("SoftAP IP: ");
  Serial.println(WiFi.softAPIP());
  createWebServer();

  // Start the server
  server.begin();
  Serial.println("Server started");
}

void setupAP(void)
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("Scanning for network...");
  int n = WiFi.scanNetworks();
  if (n == 0) 
  {
    Serial.println("No networks discovered");
  }
  else
  {
    Serial.print(n);
    Serial.println(" networks found");

    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.println(")");
      delay(10);
    }
  }
  Serial.println("");
  for (int i = 0; i < n; ++i)
  {
    ssid_list += "<option value=\"" + WiFi.SSID(i) + "\" > \""+ WiFi.SSID(i) +"\" </option>";
  }
  delay(100);

  /* Can be used for making static/fixed IP address of softAP (optional) */
  // IPAddress IP = {10, 10, 10, 10};
  // IPAddress NMask = (255, 255, 255, 0);
  // WiFi.softAPConfig(IP, IP, NMask);

  WiFi.softAP(Soft_AP_SSID, "");
  Serial.println("Soft_AP has started");

  launchWeb();
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(9600);
  delay(1000);
  Serial.println("ESP32 has Started");

  /*
  *  Theoritically assigned issues and respectedvalues
  *  value 1: associate with critical deep_sleep operation
  */
  if(rtc_reset_cause == 1)
  {
    Serial.println("ESP Reset Cause: Webpage DeepSleep request triggered Task Watchdog Timer (TWDT) issue");
    Serial.println("Handling Issue...");
    /* Reseting variable before entering deep_sleep */
    rtc_reset_cause = 0;
    touchAttachInterrupt(TOUCHPIN, callback, TOUCH_THRESHOLD);
    esp_sleep_enable_touchpad_wakeup();
    LightMode = 3;
    Serial.println("successfully Going to sleep... Zzz");
    esp_deep_sleep_start();
  }
  else
  {
    Serial.println("No critical crash/reset issues discovered");
  }

  // Prints reason responsible to cause wakeup from deep_sleep state 
  Serial.print("Wakeup Reason: "); 
  print_wakeup_reason();

  Serial.println("Disconnecting previously connected WiFi");
  WiFi.disconnect();
  EEPROM.begin(512); //Initialasing EEPROM
  delay(10);

  ssid = eeprom_readString(0);
  password = eeprom_readString(33);
  authorization_portal = EEPROM.read(160);
  Serial.print("authorization_portal: ");
  Serial.println(authorization_portal);
  if(authorization_portal == 1)
  {
    esp_username = eeprom_readString(97);
    esp_password = eeprom_readString(117);
  }
  esp_location = eeprom_readString(147);

  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid.c_str(), password.c_str());

  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);

  xTaskCreatePinnedToCore(
      LightController,      /* Function to implement the task */
      "Light control",      /* Name of the task */
      1000,                 /* Stack size in words */
      NULL,                 /* Task input parameter */
      0,                    /* Priority of the task */
      &LightControlTask,    /* Task handle. */
      0);                   /* Core where the task should run */

  if(EEPROM.read(161) == 1)
  {
    // Created FreeRTOS Task to maintain WiFi connection "Asynchronously".
    xTaskCreatePinnedToCore(
      autoConnectToWiFi,
      "auto connect to WiFi",   // Task name
      3000,                     // Stack size (bytes)
      NULL,                     // Parameter
      1,                        // Task priority
      NULL,                     // Task handle
      CONFIG_ARDUINO_RUNNING_CORE
    );

    Serial.println(WiFi.localIP());

    if(WiFi.status() == WL_CONNECTED) {
      Serial.println("Succesfully Connected!!!");
    }
    else {
      Serial.println("WiFi is'nt connected");
    }

    /* Starting DHT reference object for getting temperature/humidity readings */
    dht.begin();

    // Blynk.config(authCode, “blynk-cloud.com”, 8442);
    Blynk.begin(authCode, ssid.c_str(), password.c_str(), IPAddress(139,59,206,133), 8442);

    /*
    *  Set all relays to off when the program starts - 
    *  if set to Normally Open (NO), the relay is off when you set the relay to HIGH
    */
    for(int i=0; i<NUM_RELAYS; i++)
    {
      pinMode(relayGPIOs[i], OUTPUT);
      digitalWrite(relayGPIOs[i], (RELAY_NO)?HIGH:LOW);
    }

    /*
    *  Enabling touch_to_wakeup & touch_to_wakeup feature 
    *  
    *  Note: Here we could also assign TOUCHPIN value as T0/T1/T2 as per diagram
    *  Example: TOUCHPIN = 4 is associated with T0 (Touch 0) in ESP32 circuit diagram
    */
    touchAttachInterrupt(TOUCHPIN, callback, TOUCH_THRESHOLD);
    esp_sleep_enable_touchpad_wakeup();
    Serial.println("0");
    if(authorization_portal == 1)
    {
      Serial.println("1");
      server.on("/auth", HTTP_GET, [](AsyncWebServerRequest *request){
        Serial.println("2");
        request->send_P(200, "text/html", auth);
        Serial.println("3");
      });
    }
    else if(authorization_portal == 0)
    {
      Serial.println("Inside 1");
      server.on("/", HTTP_GET, [] (AsyncWebServerRequest *request) {
        Serial.println("Inside 2");
        request->send_P(200, "text/html", home_page, processor);
        Serial.println("Inside 3");
        server.on("/deepsleep", HTTP_POST, [] (AsyncWebServerRequest *request) {
          Serial.println("Inside 4");
          request->send_P(200, "text/html", deep_sleep, processor);
          Serial.println("Inside 5");
          /* 
          *  Initializing critical deep_sleep operation to RTC_NOINIT_ATTR variable 
          *  for error handling incase ESP gets crashed/restart 
          *  
          *  Theoritically assigned value 1 to this issue. Will be handeled at beginning 
          *  of setup() (startUp_error_handler) to validate between normal restart or critically caused restart.
          *  
          *  Incase of rtc_reset_cause of 1, ESP will go_to_sleep immediately when 
          *  startUp_error_handler will see value 1 if this operation was incomplete.
          * 
          *  Similarly each critical rtc_reset_cause values will be associated with incomplete_purpose
          *  which will be accomplished by startUp_error_handler.
          */
          rtc_reset_cause = 1;

          int c = 0;
          while(c < 5)
          {
            delay(500);
            c++;
          }
          Serial.println("Enabling touch sensing");
          touchAttachInterrupt(TOUCHPIN, callback, TOUCH_THRESHOLD);
          esp_sleep_enable_touchpad_wakeup();
          Serial.println("Enabled touch sensing to wakeUp");
          LightMode = 3;
          delay(3000);
          Serial.println("Going to sleep... Zzz");
          /* 
          *  Successfully reached critical deep_sleep operation hence 
          *  closing the startUp_error_handler issue by assigning 0 value in RTC_Memory 
          */
          rtc_reset_cause = 0;
          esp_deep_sleep_start();
        });
      });
    }
    else
    {
      Serial.println("Something Something...");
    }

    // Route for root / web page
    Serial.println("4");
    server.on("/", HTTP_POST, [] (AsyncWebServerRequest *request) {
      Serial.println("5");
      String inputUsername;
      String inputUsernameParam;
      String inputPassword;
      String inputPasswordParam;
      // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
      if(request->hasParam(USERNAME_PARAM, true) && request->hasParam(PASSWORD_PARAM, true)) {
        Serial.println("6");
        inputUsername = request->getParam(USERNAME_PARAM, true)->value();
        inputUsernameParam = USERNAME_PARAM;
        inputPassword = request->getParam(PASSWORD_PARAM, true)->value();
        inputPasswordParam = PASSWORD_PARAM;
      } else {
        Serial.println("7");
        inputUsername = "null";
        inputUsernameParam = "null";
        inputPassword = "null";
        inputPasswordParam = "null";
      }
      Serial.println("8");
      if(((inputUsername == esp_username) && (inputPassword == esp_password)) || authorization_portal == 0)
      {
        Serial.println("9");
        request->send_P(200, "text/html", home_page, processor);
        Serial.println("10");
        server.on("/deepsleep", HTTP_POST, [] (AsyncWebServerRequest *request) {
          Serial.println("11");
          request->send_P(200, "text/html", deep_sleep, processor);
          Serial.println("12");
          /* 
          *  Initializing critical deep_sleep operation to RTC_NOINIT_ATTR variable 
          *  for error handling incase ESP gets crashed/restart 
          *  
          *  Theoritically assigned value 1 to this issue. Will be handeled at beginning 
          *  of setup() (startUp_error_handler) to validate between normal restart or critically caused restart.
          *  
          *  Incase of rtc_reset_cause of 1, ESP will go_to_sleep immediately when 
          *  startUp_error_handler will see value 1 if this operation was incomplete.
          * 
          *  Similarly each critical rtc_reset_cause values will be associated with incomplete_purpose
          *  which will be accomplished by startUp_error_handler.
          */
          rtc_reset_cause = 1;

          int c = 0;
          while(c < 5)
          {
            delay(500);
            c++;
          }
          Serial.println("Enabling touch sensing");
          touchAttachInterrupt(TOUCHPIN, callback, TOUCH_THRESHOLD);
          esp_sleep_enable_touchpad_wakeup();
          Serial.println("Enabled touch sensing to wakeUp");
          LightMode = 3;
          delay(3000);
          Serial.println("Going to sleep... Zzz");
          /* 
          *  Successfully reached critical deep_sleep operation hence 
          *  closing the startUp_error_handler issue by assigning 0 value in RTC_Memory 
          */
          rtc_reset_cause = 0;
          esp_deep_sleep_start();
        });
      } else {
        Serial.println("13");
        request->send(404, "text/html", "<h2> Invalid Username or Password </h2><br><a href=\"/auth\">Return to Login Page</a>");
        Serial.println("14");
      }
    });
    Serial.println("15");
    server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
      Serial.println("16");
      request->send_P(200, "text/plain", readDHTTemperature().c_str());
      Serial.println("17");
    });
    Serial.println("18");
    server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
      Serial.println("19");
      request->send_P(200, "text/plain", readDHTHumidity().c_str());
      Serial.println("20");
    });
    Serial.println("21");
    // Send a GET request to <ESP_IP>/update?relay=<inputMessage>&state=<inputMessage2>
    server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
      Serial.println("22");
      String inputMessage;
      String inputMessage2;
      // GET input1 value on <ESP_IP>/update?relay=<inputMessage>
      if(request->hasParam(RELAY_PARAM) & request->hasParam(STATE_PARAM)) {
        inputMessage = request->getParam(RELAY_PARAM)->value();
        inputMessage2 = request->getParam(STATE_PARAM)->value();
        if(RELAY_NO){
          Serial.print("Connection: NO ");
          digitalWrite(relayGPIOs[inputMessage.toInt()-1], !inputMessage2.toInt());
        }
        else{
          Serial.print("Connection: NC ");
          digitalWrite(relayGPIOs[inputMessage.toInt()-1], inputMessage2.toInt());
        }
      }
      else {
        inputMessage = "null";
      }
      Serial.println("Relay: " + inputMessage + " State: " + inputMessage2);
      request->send(200, "text/plain", "OK");
    });
    Serial.println("23");
    server.onNotFound([](AsyncWebServerRequest *request){
      Serial.println("24");
      if (!espalexa.handleAlexaApiCall(request)) //if you don't know the URI, ask espalexa whether it is an Alexa control request
      {
        Serial.println("25");
        //whatever you want to do with 404s
        request->send(404, "text/plain", "Not found");
        Serial.println("26");
      }
    });

    // Defining Devices to ESPalexa
    for(int i=0; i<NUM_RELAYS; i++)
    {
      espalexa.addDevice(Device_Name[i], deviceChanged[i], 0);
    }
    // Defining Groups to ESPalexa
    for(int i=0; i<NUM_GROUPS; i++)
    {
      espalexa.addDevice(Group_Name[i], groupChanged[i]);
    }

    // Start server
    espalexa.begin(&server); //give espalexa a pointer to your server object so it can use your server instead of creating its own
    //server.begin(); //omit this since it will be done by espalexa.begin(&server)
  }
  else    // Incase of EEPROM.read(161) is '0' or 'Not specified'.
  {
    Serial.println("Turning the HotSpot On");
    LightMode = 0;
    setupAP();  // Setup HotSpot

    unsigned long startAttemptTime = millis();
    /* 
    *  Keeps looping while until setup not done with time limit of $(setup_session_timeout) [default: 5 mins]
    */
    while (millis() - startAttemptTime < (setup_session_timeout * 60 * 1000))
    {
      Serial.print(".");
      delay(100);
      // server.handleClient();  /* Not required in ESP32 */
    }
    Serial.println("ESP Resetting Session Timeout...");
    /* Execute below code incase Resetting session is not completed in $(setup_session_timeout) mins */
    touchAttachInterrupt(TOUCHPIN, callback, TOUCH_THRESHOLD);
    esp_sleep_enable_touchpad_wakeup();
    LightMode = 3;
    Serial.println("Going to sleep...Zzz");
    Serial.println("Touch WakeUp is enabled. You can always awake ESP anytime");
    esp_deep_sleep_start();
  }
}

void loop() {
  goToDeepSleep();
  espalexa.loop();
  Blynk.run();
  delay(1000);
}