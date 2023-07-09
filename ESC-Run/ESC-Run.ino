#define ARDUINO_ESP32C3_DEV

#define TIMER_INTERRUPT_DEBUG       0
#define ISR_SERVO_DEBUG             1

// Select different ESP32 timer number (0-1) to avoid conflict
#define USE_ESP32_TIMER_NO          1

#include "position.h"

#include <Adafruit_NeoPixel.h>

// Update ESP32_fasttimerinterrupt.h if needed
#include "ESP32_C3_ISR_Servo.h"

// BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>


#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "05790121-75a6-4039-aedd-d1cd894e7049"
#define CHARACTERISTIC_UUID_2 "abd904de-fd93-498d-90f2-64a60c6d347b"
#define CHARACTERISTIC_UUID_3 "d03E95a6-4ade-4185-8c00-533bcf9ddd5f"
#define CHARACTERISTIC_UUID_4 "029cb157-efc6-4758-87fb-cd05526f1738" // Tare


// /BLE

//See file .../hardware/espressif/esp32/variants/(esp32|doitESP32devkitV1)/pins_arduino.h
#define LED_BUILTIN       2         // Pin D2 mapped to pin GPIO2/ADC12 of ESP32, control on-board LED
#define PIN_LED           2         // Pin D2 mapped to pin GPIO2/ADC12 of ESP32, control on-board LED

#define PIN_D0            0         // Pin D0 mapped to pin GPIO0/BOOT/ADC11/TOUCH1 of ESP32
#define PIN_D1            1         // Pin D1 mapped to pin GPIO1/TX0 of ESP32
#define PIN_D2            2         // Pin D2 mapped to pin GPIO2/ADC12/TOUCH2 of ESP32
#define PIN_D3            3         // Pin D3 mapped to pin GPIO3/RX0 of ESP32
#define PIN_D4            4         // Pin D4 mapped to pin GPIO4/ADC10/TOUCH0 of ESP32
#define PIN_D5            5         // Pin D5 mapped to pin GPIO5/SPISS/VSPI_SS of ESP32
#define PIN_D6            6         // Pin D6 mapped to pin GPIO6/FLASH_SCK of ESP32
#define PIN_D7            7         // Pin D7 mapped to pin GPIO7/FLASH_D0 of ESP32
#define PIN_D8            8         // Pin D8 mapped to pin GPIO8/FLASH_D1 of ESP32
#define PIN_D9            9         // Pin D9 mapped to pin GPIO9/FLASH_D2 of ESP32

// Published values for SG90 servos; adjust if needed
#define MIN_MICROS      1000  //544
#define MAX_MICROS      2000

int left_motor  = 0;
int right_motor  = 1;

// Command format:
// 00 360 100
// command tyoe, angle (degrees), percent forward
BLECharacteristic *ble_command_characteristic;

BLECharacteristic *ble_command_angle;

BLECharacteristic *ble_command_power;

BLECharacteristic *ble_command_tare;

// How many internal neopixels do we have? some boards have more than one!
#define NUMPIXELS    3
#define PIN_NEOPIXEL 7

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

void setup()
{
  Serial.begin(115200);

  pixels.fill(0x0000FF);
  pixels.show();

  // Wait for monitor
  //while (!Serial);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(20); // not so bright


  // Init IMU
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
 // bno.setExtCrystalUse(true);

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

  Serial.print(F("\nStarting ESP32_C3_ISR_MultiServos on ")); Serial.println(ARDUINO_BOARD);
  Serial.println(ESP32_C3_ISR_SERVO_VERSION);

  //Select ESP32 timer USE_ESP32_TIMER_NO
  ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);

  left_motor = ESP32_ISR_Servos.setupServo(PIN_D0, MIN_MICROS, MAX_MICROS);
  right_motor = ESP32_ISR_Servos.setupServo(PIN_D1, MIN_MICROS, MAX_MICROS);

  if (left_motor != -1)
    Serial.println(F("Setup Servo1 OK"));
  else
    Serial.println(F("Setup Servo1 failed"));

  if (right_motor != -1)
    Serial.println(F("Setup Servo2 OK"));
  else
    Serial.println(F("Setup Servo2 failed"));

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

#define MOTOR_MID_PULSE 1500

#define FWD_CONST 1 // us devoted to fwd/100
#define TURN_CONST 1 // us devoted to turn/100

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

    float lin_x = event.acceleration.z-offset;

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
      int fwd = speed*cos(0.01745*angle);
      int turn = speed*sin(0.01745*angle);

      left += FWD_CONST*fwd - TURN_CONST*turn;
      right += FWD_CONST*fwd + TURN_CONST*turn;
    }
  }else if(cmd == 1){ // Point and shoot
  }else if(cmd == 2){ // Melty
    left -= 50;
    right += 50;
  }

  // Set motors
  ESP32_ISR_Servos.setPulseWidth(left_motor, left);    // Send the signal to the ESC
  ESP32_ISR_Servos.setPulseWidth(right_motor, right);    // Send the signal to the ESC


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