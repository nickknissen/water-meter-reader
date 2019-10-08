#include <../config.h>
#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"

#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"

#define DEGUB_ESP

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    15
#define XCLK_GPIO_NUM     27
#define SIOD_GPIO_NUM     25
#define SIOC_GPIO_NUM     23

#define Y9_GPIO_NUM       19
#define Y8_GPIO_NUM       36
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       39
#define Y5_GPIO_NUM        5
#define Y4_GPIO_NUM       34
#define Y3_GPIO_NUM       35
#define Y2_GPIO_NUM       17
#define VSYNC_GPIO_NUM    22
#define HREF_GPIO_NUM     26
#define PCLK_GPIO_NUM     21

// Connection timeout;
#define CON_TIMEOUT   10*1000 // milliseconds

// Not using Deep Sleep on PCB because TPL5110 timer takes over.
#define TIME_TO_SLEEP (uint64_t)10*60*1000*1000 // microseconds

#ifdef DEGUB_ESP
  #define DBG(x) Serial.println(x)
#else 
  #define DBG(...)
#endif

// Camera buffer, URL and picture name
camera_fb_t *fb = NULL;

AsyncMqttClient mqttClient;
TimerHandle_t   mqttReconnectTimer;
TimerHandle_t   wifiReconnectTimer;

// Create functions prior to calling them as .cpp files are differnt from Arduino .ino
void connectWiFi(void);
void connectMQTT(void);
void deep_sleep (void);
bool camera_init(void);
bool take_picture(void);

void onMqttConnect(bool sessionPresent) {
  // Take picture
  take_picture();

  // Publish picture
  const char* pic_buf = (const char*)(fb->buf);
  size_t length = fb->len;
  uint16_t packetIdPubTemp = mqttClient.publish( TOPIC_PIC, 0, false, pic_buf, length );
  
  DBG("buffer is " + String(length) + " bytes");

  // No delay result in no message sent.
  delay(200);

  if(!packetIdPubTemp) {
    DBG("Sending Failed! err: " + String( packetIdPubTemp ));
  } else {
    DBG("MQTT Publish succesful");
  }
  
  deep_sleep();
}

bool take_picture() {
  DBG("Taking picture now");

  fb = esp_camera_fb_get();  
  if(!fb) {
    DBG("Camera capture failed");
    return false;
  }
  
  DBG("Camera capture success");

  return true;
}

void deep_sleep() {
  DBG("Going to sleep after: " + String( millis() ) + "ms");
  Serial.flush();

  esp_deep_sleep_start();
}

void setup() {
  #ifdef DEGUB_ESP
    Serial.begin(115200);
    Serial.setDebugOutput(true);
  #endif

  // COnfigure MQTT Broker and callback
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectMQTT));
  mqttClient.setCredentials(USERNAME, PASSWORD);
  mqttClient.onConnect (onMqttConnect);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  camera_init();
  
  esp_sleep_enable_timer_wakeup( TIME_TO_SLEEP );

  connectWiFi();
  connectMQTT();
}

void loop() {
  // put your main code here, to run repeatedly:
}

bool camera_init() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 

  config.frame_size   = FRAMESIZE_HQVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.print("Camera init failed with error 0x%x");
    DBG(err);
    return false;
  }
  return true;
}

void connectWiFi() {
  DBG("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED && millis() < CON_TIMEOUT) {
    delay(500);
    Serial.print(".");
  }

  if( WiFi.status() != WL_CONNECTED ) {
    DBG("Failed to connect to WiFi");
    delay( 600 );
    deep_sleep();
  }

  DBG();
  DBG("IP address: ");
  DBG(WiFi.localIP());
}

void connectMQTT() {
  DBG("Connecting to MQTT...");
  mqttClient.connect();

  while(!mqttClient.connected() && millis() < CON_TIMEOUT) {
    delay(250);
    Serial.print(".");
  }

  if(!mqttClient.connected()) {
    DBG("Failed to connect to MQTT Broker");
    deep_sleep();
  }
}