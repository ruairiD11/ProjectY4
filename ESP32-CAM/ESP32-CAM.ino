/*  Remote-Robot ESP32 Camera code
 *  Ruairi Doherty
 *  References:
 *    AWS_IOT Library for ESP32 - ExploreEmbedded / HornBill: https://github.com/ExploreEmbedded/Hornbill-Examples/tree/master/arduino-esp32/AWS_IOT
 *    Arduino ESP32 Library CameraWebServer Example Code 
 */

#include <AWS_IOT.h>
#include <WiFi.h>
#include "esp_now.h"
#include "esp_camera.h"

// Specific model of my ESP32 CAM
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

AWS_IOT hornbill;

char ssid[] = "OnePlus3";
char password[] = "12345678";
char AWS_HOST[] = "a2ot8lh5ttyzzl-ats.iot.us-east-1.amazonaws.com";
char CLIENT_ID[] = "espClient";
char TOPIC[] = "myESP32/esp32topic";

int status = WL_IDLE_STATUS;
char payload[8]; 

// Glove module ESP32 MAC address
uint8_t gloveModuleAddress[] = { 0x84, 0x0D, 0x8E, 0xE6, 0x7C, 0x44 };

uint8_t sensorDataRx[4];

void sub_callback_handler(char *topicName, int payloadLen, char *payLoad)
{
  // do nothing
}

void on_data_receive(const uint8_t* mac_addr, const uint8_t* incomingData, int len)
{
  memcpy(&sensorDataRx, incomingData, sizeof(sensorDataRx));
}

void startCameraServer();

void setup() {
  Serial.begin(115200);
  camera_config();
  wifi_init();
  topic_subscribe();
}

void camera_config() 
{
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
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);
}

void wifi_init()
{
  while(status != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, password);
    delay(5000);
  }
  Serial.println("Connected to WiFi");

  startCameraServer();

  Serial.print("Livestream: 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("'");
}

void topic_subscribe()
{
  if(hornbill.connect(AWS_HOST, CLIENT_ID) == 0)
  {
    Serial.println("Connected to AWS");
    delay(1000);

    if(0==hornbill.subscribe(TOPIC, sub_callback_handler))
    {
      Serial.println("Subscribe Successful");
      init_esp_now();
    }
    else
    {
      Serial.println("Subscribe Failed - check Thing Name and certs");
      while(1);
    }
  }
  else
  {
    Serial.println("AWS connection failed - check HOST address");
    while(1);
  }
}

void init_esp_now() 
{
  ESP_ERROR_CHECK(esp_now_init());
  esp_now_peer_info_t glove_peerInfo;
  memcpy(glove_peerInfo.peer_addr, gloveModuleAddress, 6);
  glove_peerInfo.ifidx = WIFI_IF_STA;
  glove_peerInfo.channel = 1;
  glove_peerInfo.encrypt = false;
  ESP_ERROR_CHECK(esp_now_add_peer(&glove_peerInfo));
  esp_now_register_recv_cb(on_data_receive);

  delay(1000);
}

void topic_publish()
{
  // Convert data back to 16-bit signed ints
  sensorDataRx[0] = (int16_t) sensorDataRx[0]; 
  sensorDataRx[1] = (int16_t) sensorDataRx[1]; 
  sensorDataRx[2] = (int16_t) sensorDataRx[2]; 
  sensorDataRx[3] = (int16_t) sensorDataRx[3]; 
  
  sprintf(payload,"%d, %d, %d, %d", sensorDataRx[0], sensorDataRx[1], 
                                    sensorDataRx[2], sensorDataRx[3]);
  if(hornbill.publish(TOPIC, payload) == 0)
  {
    Serial.print("Publish Message: ");
    Serial.println(payload);
  }
  else
    Serial.println("Publish Failed");
  delay(50);
}

void loop() {
  topic_publish();
}
