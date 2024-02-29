#define ARDUINO_ESP32S3_DEV

#include "position.h"

#include <Adafruit_NeoPixel.h>

// MCPWM
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

// BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>


#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "05790121-75a6-4039-aedd-d1cd894e7049"
#define CHARACTERISTIC_UUID_2 "abd904de-fd93-498d-90f2-64a60c6d347b"
#define CHARACTERISTIC_UUID_3 "d03E95a6-4ade-4185-8c00-533bcf9ddd5f"
#define CHARACTERISTIC_UUID_4 "029cb157-efc6-4758-87fb-cd05526f1738" // Tare


// BLE
//See file .../hardware/espressif/esp32/variants/(esp32|doitESP32devkitV1)/pins_arduino.h
#define LED_BUILTIN       2         // Pin D2 mapped to pin GPIO2/ADC12 of ESP32, control on-board LED
#define PIN_LED           2         // Pin D2 mapped to pin GPIO2/ADC12 of ESP32, control on-board LED

#define PIN_D0            1         // Pin D0 mapped to pin GPIO0/BOOT/ADC11/TOUCH1 of ESP32
#define PIN_D1            2         // Pin D1 mapped to pin GPIO1/TX0 of ESP32
#define PIN_D2            3         // Pin D2 mapped to pin GPIO2/ADC12/TOUCH2 of ESP32
#define PIN_D3            4         // Pin D3 mapped to pin GPIO3/RX0 of ESP32
#define PIN_D4            5         // Pin D4 mapped to pin GPIO4/ADC10/TOUCH0 of ESP32
#define PIN_D5            6         // Pin D5 mapped to pin GPIO5/SPISS/VSPI_SS of ESP32
#define PIN_D6            43         // Pin D6 mapped to pin GPIO6/FLASH_SCK of ESP32
#define PIN_D7            44         // Pin D7 mapped to pin GPIO7/FLASH_D0 of ESP32
#define PIN_D8            7         // Pin D8 mapped to pin GPIO8/FLASH_D1 of ESP32
#define PIN_D9            8         // Pin D9 mapped to pin GPIO9/FLASH_D2 of ESP32
#define PIN_D10           9         // Pin D9 mapped to pin GPIO9/FLASH_D2 of ESP32

// Command format:
// 00 360 100
// command type, angle (degrees), percent forward
BLECharacteristic *ble_command_characteristic;

BLECharacteristic *ble_command_angle;

BLECharacteristic *ble_command_power;

BLECharacteristic *ble_command_tare;

// How many internal neopixels do we have? some boards have more than one!
#define NUMPIXELS    3
#define PIN_NEOPIXEL PIN_D3

Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// IMU - wants a pullup resistor
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>

Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);

double xPos = 0, yPos = 0, headingVel = 0;

double offset=0;

bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
      pServer->updateConnParams(param->connect.remote_bda, 0x06, 0x06, 0, 600);
      deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer)
    {
      deviceConnected = false;
    }
};

void displayDataRate(void)
{
  Serial.print  ("Data Rate:    ");

  switch(accel.getDataRate())
  {
    case ADXL343_DATARATE_3200_HZ:
      Serial.print  ("3200 ");
      break;
    case ADXL343_DATARATE_1600_HZ:
      Serial.print  ("1600 ");
      break;
    case ADXL343_DATARATE_800_HZ:
      Serial.print  ("800 ");
      break;
    case ADXL343_DATARATE_400_HZ:
      Serial.print  ("400 ");
      break;
    case ADXL343_DATARATE_200_HZ:
      Serial.print  ("200 ");
      break;
    case ADXL343_DATARATE_100_HZ:
      Serial.print  ("100 ");
      break;
    case ADXL343_DATARATE_50_HZ:
      Serial.print  ("50 ");
      break;
    case ADXL343_DATARATE_25_HZ:
      Serial.print  ("25 ");
      break;
    case ADXL343_DATARATE_12_5_HZ:
      Serial.print  ("12.5 ");
      break;
    case ADXL343_DATARATE_6_25HZ:
      Serial.print  ("6.25 ");
      break;
    case ADXL343_DATARATE_3_13_HZ:
      Serial.print  ("3.13 ");
      break;
    case ADXL343_DATARATE_1_56_HZ:
      Serial.print  ("1.56 ");
      break;
    case ADXL343_DATARATE_0_78_HZ:
      Serial.print  ("0.78 ");
      break;
    case ADXL343_DATARATE_0_39_HZ:
      Serial.print  ("0.39 ");
      break;
    case ADXL343_DATARATE_0_20_HZ:
      Serial.print  ("0.20 ");
      break;
    case ADXL343_DATARATE_0_10_HZ:
      Serial.print  ("0.10 ");
      break;
    default:
      Serial.print  ("???? ");
      break;
  }
  Serial.println(" Hz");
}

#define GPIO_PWM0A_OUT 1   //HV
#define GPIO_PWM1A_OUT 2   //HV

#define PIN_NEOPIXEL   4 // On Trinket or Gemma, suggest changing this to 1

int freq0 = 50; // 50 nominal, 500 theoretical limit
int freq1 = 1000;


void setuppwm(){
   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
   
 mcpwm_config_t pwm_config0;
     pwm_config0.frequency = freq0;  //frequency 
     pwm_config0.cmpr_a = 0;      //duty cycle of PWMxA = 50.0%
     pwm_config0.cmpr_b = 0;      //duty cycle of PWMxB = 50.0%
     pwm_config0.counter_mode = MCPWM_UP_COUNTER; // Up-down counter (triangle wave)
     pwm_config0.duty_mode = MCPWM_DUTY_MODE_0; // Active HIGH
     
   mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config0);    //Configure PWM0A & PWM0B with above settings
   mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config0);    //Configure PWM0A & PWM0B with above settings
   
   delay(20);
   mcpwm_set_timer_sync_output(MCPWM_UNIT_0, MCPWM_TIMER_0,MCPWM_SWSYNC_SOURCE_TEZ);
   mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_0,MCPWM_SELECT_TIMER0_SYNC, 0);   
   mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1,MCPWM_SELECT_TIMER0_SYNC, 500); 

   mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 1500);
   mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 1500);

   delay(1000);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 2000);
   mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 2000);
   
   delay(1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 1500);
   mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 1500);
   delay(1000);
 }

void setup()
{
  Serial.begin(115200);

  
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(20); // not so bright

  pixels.fill(0x0000FF);
  pixels.show();

  // Wait for monitor
  //while (!Serial);

  //Initialize motors
  setuppwm();

  // Init IMU
  Wire.setPins(SDA, SCL);
  Wire.begin(SDA, SCL);
  Serial.println("ADXL375 Accelerometer Test"); Serial.println("");
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL375 ... check your connections */
    Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
    while(1);
  }

  // Range is fixed at +-200g

  /* Display some basic information on this sensor */
  displayDataRate();
  accel.printSensorDetails();
  displayDataRate();
  Serial.println("");
  
  delay(1000);
  //bno.setExtCrystalUse(true);

  // BLE

  Serial.println("Starting BLE work!");

  BLEDevice::init("Whirlwind");
  BLEServer *pServer = BLEDevice::createServer();
  // Init callbacks
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  ble_command_characteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  ble_command_angle = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_2,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  ble_command_power = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_3,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  ble_command_tare = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_4,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  ble_command_characteristic->setValue("0");
  ble_command_angle->setValue("0");
  ble_command_power->setValue("0");
  ble_command_tare->setValue("10");

  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");

  // /BLE

  pixels.setPixelColor(0, 248, 255, 132, 50);
  pixels.setBrightness(100); // not so bright
  pixels.show();

  delay(10000);

  for(int i = 0; i<100;i++){
    sensors_event_t event;
    accel.getEvent(&event);

    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

   /* imu::Vector<3> li_ac = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // VECTOR_ACCELEROMETER
    Serial.print(F("Li. Acc.: "));
    Serial.print((float)li_ac.y());
    Serial.println();*/

    offset += (float)event.acceleration.y;
    delay(5);
  }

  offset = offset/100;


  pixels.fill(0x00FF00);
  pixels.setBrightness(20); // not so bright
  pixels.show();

}

#define MOTOR_MID_PULSE 1500 // US

#define FWD_CONST 0.45 // degrees devoted to fwd/100
#define TURN_CONST 0.45 // us devoted to turn/100

unsigned long start_time_sense=0;
unsigned long start_time_loop=0;

void loop()
{
  ////////////////// Update sensor
  int time_sense =  millis()-start_time_sense; // How long since last update
  if(time_sense >10){
    /* Get a new sensor event */ 
    sensors_event_t event;
    accel.getEvent(&event);

    // Weird offset in X - thats why I own a bunch of buttons
    // TODO add a calibration procedure w/lights
    float lin_x = event.acceleration.x-7.69;
    float lin_y = event.acceleration.y;
    float lin_z = event.acceleration.z;

    Serial.print(lin_x);
    Serial.print(" ");
    Serial.print(lin_y);
    Serial.print(" ");
    Serial.println(lin_z);

    start_time_sense =  millis();
    updateAccel(lin_x, 10); // TODO implement tare
  }


  // Set motors
  std::string angle_str = ble_command_angle->getValue();
  std::string power_str = ble_command_power->getValue();
  std::string tare_str = ble_command_tare->getValue();

  int cmd = atoi(ble_command_characteristic->getValue().c_str());
  int angle = atoi(angle_str.c_str());
  int speed = atoi(power_str.c_str());

  int tare = atoi(tare_str.c_str());

  // Process command
  int left = MOTOR_MID_PULSE;
  int right = MOTOR_MID_PULSE;
  // Direct control
  if (cmd == 0){
    // Only run if outside dead zone
    if(speed > 10){
      float fwd = speed*cos(0.01745*angle);
      float turn = speed*sin(0.01745*angle);

      left += FWD_CONST*fwd - TURN_CONST*turn;
      right += FWD_CONST*fwd + TURN_CONST*turn;
    }
  }else if(cmd == 1){ // Point and shoot
  }else if(cmd == 2){ // Melty
    left -= 250;
    right += 250;
  }

  // Set motors
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, left);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, right);


  // ble_command_tare - getvalue
  int time_loop =  micros()-start_time_loop; // How long since last update
  start_time_loop = micros();
  updateHeading(time_loop);

  int theta = getHeadingAngle();


  if(theta<180){
    pixels.setPixelColor(2, 255, 0, 0, 10); // red
  }else{
    pixels.setPixelColor(2, 0, 0, 255, 10); // blue
  }

  if(theta <270 && theta > 90){
      pixels.setPixelColor(1, 128, 128, 0, 50);
  }else{
      pixels.setPixelColor(1, 0, 0, 0, 50);
  }
  pixels.show();

  delayMicroseconds(10);
}