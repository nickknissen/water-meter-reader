#include <../config.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <math.h>
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

#include <sstream>
#include <string>

// Connection timeout;
#define CON_TIMEOUT   10*1000 // milliseconds

// Not using Deep Sleep on PCB because TPL5110 timer takes over.
#define TIME_TO_SLEEP (uint64_t)10*60*1000*1000 // microseconds

#define PACKET_SIZE 3000;

#ifdef DEGUB_ESP
  #define DBG(x) Serial.println(x)
#else
  #define DBG(...)
#endif

AsyncMqttClient mqttClient;
TimerHandle_t   mqttReconnectTimer;
TimerHandle_t   wifiReconnectTimer;

camera_fb_t *fb = NULL;


Adafruit_NeoPixel strip(NEOPIXEL_AMOUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Create functions prior to calling them as .cpp files are differnt from Arduino .ino
void colorWipe(void);
void connectWiFi(void);
void connectMQTT(void);
void deep_sleep (void);
void sendPackage(int start, int end, int pos, int noOfPackets, const char * buffer, long imageId);
bool camera_init(void);
bool take_picture(void);


void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
    strip.show();
    delay(wait);
  }
}

void onMqttConnect(bool sessionPresent) {
  // Take picture
  take_picture();
  // Publish picture
  const char* data = (const char*)(fb->buf);
  char buffer[3000] = {0};
  size_t length = fb->len;
  int start = 0;
  int pos = 0;
  int end = PACKET_SIZE;
  size_t buffer_len = sizeof(buffer)-1;

  while (start <= length) {
    memcpy(buffer, data + start, end);
    DBG(buffer);

    uint16_t packetIdPubTemp = mqttClient.publish(MQTT_TOPIC, 0, false, buffer, buffer_len);

    if(!packetIdPubTemp) {
      DBG("Sending Failed! err: " + String( packetIdPubTemp ));
    } else {
      DBG("MQTT Publish succesful");
    }

    end += PACKET_SIZE;
    start += PACKET_SIZE;
    pos++;
  }

  deep_sleep();
}

void sendPackage(int start, int end, int pos, int noOfPackets, const char * buffer, long imageId) {
  DBG("start: " + String(start) + ", end: " + String(end) + ", pos" + String(pos));

  String message = "{\"imageId\": "+ String(imageId) +", \"data\": \""+ buffer +"\", \"pos\": "+pos+", \"size\": "+ String(noOfPackets)+"}";

  String message2 = ""+String(imageId)+":" + buffer;

  uint16_t packetIdPubTemp = mqttClient.publish(MQTT_TOPIC, 0, false, message.c_str(), message.length());

  if(!packetIdPubTemp) {
    colorWipe(strip.Color(255,0,0), 20);
    DBG("Sending Failed! err: " + String( packetIdPubTemp ));
    colorWipe(strip.Color(0,0,0), 20);
  } else {
    DBG("MQTT Publish succesful");
  }
  colorWipe(strip.Color(0,0,0), 20);

}

bool take_picture() {
  DBG("Taking picture now");
  colorWipe(strip.Color(127, 127, 127), 0);
  delay(500);

  fb = esp_camera_fb_get();  
  if(!fb) {
    DBG("Camera capture failed");
    return false;
  }
  
  DBG("Camera capture success");
  delay(200);
  colorWipe(strip.Color(0, 0, 0), 0);
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
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'

  // COnfigure MQTT Broker and callback
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectMQTT));
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  mqttClient.onConnect (onMqttConnect);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  camera_init();
  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP);

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

  config.frame_size   = FRAMESIZE_QQVGA2;
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