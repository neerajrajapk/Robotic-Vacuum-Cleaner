#include <atomic>
#include <Arduino.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include <BLEDevice.h>
#include <BLE2902.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <SparkFun_TB6612.h>
#include <WiFi.h>

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define DEVICE_MANUFACTURER "Mobiler"
#define DEVICE_NAME "Robotic Vacuum Cleaner"

// Standard UUIds for Device Information
#define DEVINFO_UUID (uint16_t)0x180a
#define DEVINFO_MANUFACTURER_UUID (uint16_t)0x2a29
#define DEVINFO_NAME_UUID (uint16_t)0x2a24
#define DEVINFO_SERIAL_UUID (uint16_t)0x2a25


// Left Motor
const int Motor_1_Pin_1 = 15;
const int Motor_1_Pin_2 = 27;
const int Enable_1_Pin = 14;

// Right Motor
const int Motor_2_Pin_1 = 25;
const int Motor_2_Pin_2 = 26;
const int Enable_2_Pin = 4;

// Motor definitions
Motor* motor1;
Motor* motor2;

// Relay 
const int Relay_1 = 18;
const int Relay_2 = 19;


// Ultra sensor pins- single pin configuration
const int triggercenter = 32;
const int echocenter = 32;
const int triggerright = 21;
const int echoright = 21;
const int triggerleft = 33;
const int echoleft = 33;

boolean leftFlag;
boolean centerFlag;
boolean rightFlag;

#define TRIGGER_DIRECTION_DISTANCE 30

#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 40 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

//boolean volatile started=false;

std::atomic<bool> started(false);

#define ONBOARD_LED  13 // only for Adafruit Huzzah32


// Setting PWM properties
#define FREQUENCY 30000
#define pwmChannel1  0
#define pwmChannel2  1
#define resolution  8

#define dutyCycle_7 100
#define dutyCycle_11 90

int speed;


void keepLow();

void deviateRight();
void deviateLeft();
void forward();
void reverse(boolean shouldDelay);

void turnOn();
void turnOff();

void handleEchoSensor();


boolean checkObstacle(float distance);

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
      deviceConnected = true;
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer *pServer) {
      deviceConnected = false;
    }
};

// Call back class to handle bluetooth data events
class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();
      //Done to remove NUL character coming from MIT App
  
      value.erase(find(value.begin(),value.end(),'\0'),value.end());
   
      // if(value.compare("start7")==0)
      // {
      //   speed=dutyCycle_7;
      //   started=true;
      //   forward();
      // } 
      
      if(value.compare("start")==0)
      {
        speed=dutyCycle_11;
        started=true;
        forward();
      }
      else if(value.compare("stop")==0)
      {
        // This should be synchronized to avoid race condition with echo
        started=false;
        keepLow();
        Serial.println("Stopped");
      }
      else if(value.compare("reverse")==0)
      {
       reverse(false);
      }else if(value.compare("right")==0){
        deviateRight();
      }else if(value.compare("left")==0){
        deviateLeft();
      }
      else if(value.compare("on")==0){
        turnOn();
      }
      else if(value.compare("off")==0){
        turnOff();
      }
      
  }
};


void setup() {
  Serial.begin(9600);
  pinMode(ONBOARD_LED,OUTPUT);
  pinMode(Relay_1, OUTPUT);
  pinMode(Relay_2, OUTPUT);
  turnOff();
  // PIN mode will be handled by the New ping library
  /* pinMode(triggerleft,OUTPUT);
  pinMode(echoleft,INPUT);

  pinMode(triggercenter,OUTPUT);
  pinMode(echocenter,INPUT);

  pinMode(triggerright,OUTPUT);
  pinMode(echoright,INPUT); */
 
  // Pin mode of Motor pins internally handled by the library
  motor1=new Motor(Motor_1_Pin_1, Motor_1_Pin_2, Enable_1_Pin, 1,pwmChannel1,FREQUENCY,resolution);
  motor2=new Motor(Motor_2_Pin_1, Motor_2_Pin_2, Enable_2_Pin, 1,pwmChannel2,FREQUENCY,resolution);

  digitalWrite(ONBOARD_LED,HIGH);
  keepLow();

  // Create the BLE Device
  BLEDevice::init(DEVICE_NAME);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Welcome to Robotic Vacuum Cleaner Controller");
  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting to connect with the Robotic Controller app...");
}

void loop() {
    // notify changed value
      if(started.load()){
   //handleEchoSensor();
    }
    if (deviceConnected) {
        pCharacteristic->setValue((uint8_t*)&value, 4);
        pCharacteristic->notify();
        value++;
        delay(10); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("Robotic Controller not Connected");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
        Serial.println("CONNECTED!");
    }
}

void keepLow(){
    switchOff(*motor1,*motor2);
} 

void forward()
{
  motor1->drive(speed);
  motor2->drive(speed);
  Serial.println("Moving forward");
}

void reverse(boolean shouldDelay)
{
  motor1->drive(-speed);
  motor2->drive(-speed);
  Serial.println("Moving backward");
  if(shouldDelay){
    delay(300);
  }
  
}

void deviateRight()
{
  reverse(true);
  right(*motor1,*motor2,speed);
  Serial.println("Turning right");
  delay(750);
  brake(*motor1,*motor2);
  delay(200);
  forward();
}

void deviateLeft()
{
  reverse(true);
  left(*motor1,*motor2,speed);
  Serial.println("Turning left");
  delay(750);
  brake(*motor1,*motor2);
  delay(200);
  forward();
}



void turnOn(){
  digitalWrite(Relay_1, LOW);
  digitalWrite(Relay_2, LOW);
  Serial.println("Current flowing");
}
void turnOff(){
  digitalWrite(Relay_1, HIGH);
  digitalWrite(Relay_2, HIGH);
  Serial.println("Current not flowing");
}
