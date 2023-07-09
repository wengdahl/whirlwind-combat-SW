/**bno055
 * A BLE client example that is rich in capabilities.
 * There is a lot new capabilities implemented.
 * author unknown
 * updated by chegewara
 */

#include "BLEDevice.h"
#include <math.h>
#include "BLEScan.h"
#include <BLEUtils.h>
#include <BLEClient.h>

#define NUM_CHARACTERISTICS 4

// The remote service we wish to connect to.
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID0("05790121-75a6-4039-aedd-d1cd894e7049");
static BLEUUID    charUUID1("abd904de-fd93-498d-90f2-64a60c6d347b");
static BLEUUID    charUUID2("d03E95a6-4ade-4185-8c00-533bcf9ddd5f");

static BLEUUID    charUUID3("029cb157-efc6-4758-87fb-cd05526f1738");

static BLEUUID   charUUID[NUM_CHARACTERISTICS] = {charUUID0, charUUID1, charUUID2, charUUID3};

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic[NUM_CHARACTERISTICS];
static BLEAdvertisedDevice* myDevice;

#define BUTTON_PIN 22
#define POT_PIN 35
#define LED_INDICATE_PIN 27

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.println((char*)pData);
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    digitalWrite(LED_INDICATE_PIN,1);
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    digitalWrite(LED_INDICATE_PIN,0);
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");

    //pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise) Max 517
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    for(int i =0; i<NUM_CHARACTERISTICS ; i++){
      pRemoteCharacteristic[i] = pRemoteService->getCharacteristic(charUUID[i]);
      if (pRemoteCharacteristic[i] == nullptr) {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(charUUID[i].toString().c_str());
        pClient->disconnect();
        return false;
      }
      Serial.println(" - Found our characteristic");

      // Read the value of the characteristic.
      if(pRemoteCharacteristic[i]->canRead()) {
        std::string value = pRemoteCharacteristic[i]->readValue();
        Serial.print("The characteristic value was: ");
        Serial.println(value.c_str());
      }

      if(pRemoteCharacteristic[i]->canNotify())
        pRemoteCharacteristic[i]->registerForNotify(notifyCallback);
    }

    connected = true;
    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks


void setup() {
  Serial.begin(115200);

  // must enable USB CDC on boot to use serial
  //while (!Serial);

  Serial.println("Starting Arduino BLE Client application...");

  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(20); // todo try me 1349
  pBLEScan->setWindow(25);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N11);

  // Set up button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_INDICATE_PIN, OUTPUT);

} // End of setup.


// This is the Arduino main loop function.
void loop() {

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // Read joysticks
  //////////////////////////////////
  int y_val = analogRead(33)-1847;
  int x_val = analogRead(32)-1847;
  // print out the values you read:
  Serial.printf("ADC Y value = %d\n",y_val);
  Serial.printf("ADC X value = %d\n",x_val);
  
  delay(10);  // delay in between reads for clear read from serial
  int y_adjusted = constrain(map(y_val,-2047,2047,-100,100),-100,100);
  int x_adjusted = constrain(map(x_val,-2047,2047,-100,100),-100,100);
  int angle = ((int)(57.295779513082320876798154814105*atan2(x_adjusted, y_adjusted)+270))%360;

  long squared = constrain(x_adjusted*x_adjusted+y_adjusted*y_adjusted,0,100*100);
  int power = sqrt(squared);

    Serial.printf("%04d %04d angle: %04d power: %03d\n",x_adjusted, y_adjusted, angle, power);


  int potValue = (analogRead(POT_PIN)/45.5) +10;
  Serial.printf("Button status %d Pot reading  %d\n",digitalRead(BUTTON_PIN), potValue);

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    int command = 0;
    if(!digitalRead(BUTTON_PIN)){ // Pulled low
      command = 2; // Melty
    }

    static int lastCommand = 0;
    static int lastAngle = 0;
    static int lastPower = 0;

    // Don't DDOS the ESP32 on the robot with uneeded pings 
    if(command != lastCommand){
      String commandStr = String(command);
      Serial.println("Setting new command value to \"" + commandStr + "\"");
      // Set the characteristic's value to be the array of bytes that is actually a string.
      pRemoteCharacteristic[0]->writeValue(commandStr.c_str(), commandStr.length());

      lastCommand = command;
    }

    if(angle != lastAngle){
      String angleStr = String(angle);
      Serial.println("\tSetting new angle value to \"" + angleStr + "\"");
      // Angle
      pRemoteCharacteristic[1]->writeValue(angleStr.c_str(), angleStr.length());
      lastAngle = angle;
    }

    if(power != lastPower){
      String powerStr = String(power);
      Serial.println("Setting new power value to \"" + powerStr + "\"");
      // Power
      pRemoteCharacteristic[2]->writeValue(powerStr.c_str(), powerStr.length());
      lastPower = power;
    }
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
    printf("restart scan");
  }

  delay(10);
} // End of loop
