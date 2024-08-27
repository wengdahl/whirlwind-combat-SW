/**
   ESPNOW - Basic communication - Slave
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and a Slave ESP32
   Description: This sketch consists of the code for the Slave module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/

   << This Device Slave >>

   Flow: Master
   Step 1 : ESPNow Init on Master and set it in STA mode
   Step 2 : Start scanning for Slave ESP32 (we have added a prefix of `slave` to the SSID of slave for an easy setup)
   Step 3 : Once found, add Slave as peer
   Step 4 : Register for send callback
   Step 5 : Start Transmitting data from Master to Slave

   Flow: Slave
   Step 1 : ESPNow Init on Slave
   Step 2 : Update the SSID of Slave with a prefix of `slave`
   Step 3 : Set Slave in AP mode
   Step 4 : Register for receive callback and wait for data
   Step 5 : Once data arrives, print it in the serial monitor

   Note: Master and Slave have been defined to easily understand the setup.
         Based on the ESPNOW API, there is no concept of Master and Slave.
         Any devices can act as master or salve.
*/

#include <esp_now.h>
#include <WiFi.h>
#include <math.h>

#include "RMT_DSHOT.h"

#define CHANNEL 1

//#define DEBUG
#define DEBUG_TIME

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    #ifdef DEBUG
    Serial.println("ESPNow Init Success");
    #endif
  }
  else {
    #ifdef DEBUG
    Serial.println("ESPNow Init Failed");
    #endif
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESPNow/Basic/Slave Example");
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);

  // Motors
  initialize_motors();
  delay(1000);
}
static uint32_t last_time = 0;

static struct{
  bool weapon;
  signed char pot; // 0 to 100
  signed char x; // -100 to 100
  signed char y; // -100 to 100
} rec_data;

uint16_t dshot_speed[2] = {0, 0};

// callback when data is recv from Master
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *data, int data_len) {
  #ifdef DEBUG_TIME
  Serial.print("Latency: "); Serial.println(millis()-last_time -100);
  #endif
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           info->src_addr[0], info->src_addr[1], info->src_addr[2], info->src_addr[3], info->src_addr[4], info->src_addr[5]);
  // Store new values
  rec_data.weapon = (data[0]!=0)? 1:0;
  rec_data.pot = (signed char)data[1];
  rec_data.x = (signed char)data[2];
  rec_data.y = (signed char)data[3];

  // Left, right
  int motor_pow[2] = {0, 0};

  motor_pow[0] = constrain(rec_data.y - rec_data.x, -100, 100);
  motor_pow[1] = constrain(rec_data.y + rec_data.x, -100, 100);

  // left throttle
  if(motor_pow[0] < -5){
    dshot_speed[0] = (-1*motor_pow[0])*10 +47;
  }else if(motor_pow[0] < 5){
     dshot_speed[0] = 0;
  }else{
     dshot_speed[0] = min(motor_pow[0]*10 +1049, 2047);
  }

  // Right throttle
  if(motor_pow[1] < -5){
    dshot_speed[1] = (-1*motor_pow[1])*10 +47;
  }else if(motor_pow[1] < 5){
     dshot_speed[1] = 0;
  }else{
     dshot_speed[1] = min(motor_pow[1]*10 +1049, 2047);
  }


  #ifdef DEBUG
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: ");   
  Serial.print(data[0]);
  Serial.print(":");
  Serial.print(data[1]);
  Serial.print(":");
  Serial.print(data[2]);
  Serial.print(":");
  Serial.print(data[3]);
  Serial.println("");

  Serial.println("Last Packet Recv Values: ");   
  Serial.print("WEAPON: "); Serial.println((data[0]!=0)? 1:0);
  Serial.print("POT");
  Serial.println(((signed char)data[1]));
  Serial.print("X");
  Serial.println(((signed char)data[2]));
  Serial.print("Y");
  Serial.println(((signed char)data[3]));

  Serial.print("X-throttle");
  Serial.println(dshot_speed[0]);
  Serial.print("Y-throttle");
  Serial.println(dshot_speed[1]);
  #endif

  update_throttle(0, rec_data.weapon ? (20*rec_data.pot)+47: 0); // pot
  update_throttle(1, dshot_speed[0]); // x
  update_throttle(2, dshot_speed[1]); // y
  last_time = millis();
}

void loop() {
  if(millis()-last_time >1000){
    update_throttle(0, 0); // pot
    update_throttle(1, 0); // x
    update_throttle(2, 0); // y
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}
