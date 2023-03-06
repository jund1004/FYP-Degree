#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

using std::string;

MAX30105 particleSensor;

#define bleServerName "MAX30102_ESP32Sub"
#define SERVICE_UUID "7092b545-f5da-471a-b16f-1550f55ac087"

bool deviceConnected = false;

BLECharacteristic bmeHeartRateAvgCharacteristics("501368d8-3d67-4b41-bde6-28df9aa6355a", 
                                              BLECharacteristic::PROPERTY_READ |
                                              BLECharacteristic::PROPERTY_WRITE |
                                              BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeHeartRateAvgDescriptor(BLEUUID((uint16_t)0x2902));

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

#define LED_PIN 5 // ESP32 pin GIOP18 connected to LED
int ledState = LOW;   // ledState used to set the LED
unsigned long previousMillis = 0;   // will store last time LED was updated


void setup() {

  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

    // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *bmeService = pServer->createService(SERVICE_UUID);

  bmeService->addCharacteristic(&bmeHeartRateAvgCharacteristics);

  bmeHeartRateAvgDescriptor.setValue("Heart Rate Avg");
  bmeHeartRateAvgCharacteristics.addDescriptor(new BLE2902());

  // Start the service
  bmeService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true) {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  String bavgchar = (String) beatAvg;
  unsigned long currentMillis = millis();
  if (irValue < 50000) {
    Serial.print(" No finger?");
    Serial.println();
    String zero = "No finger";
    bmeHeartRateAvgCharacteristics.setValue(zero.c_str());
    bmeHeartRateAvgCharacteristics.notify(); 
    digitalWrite(LED_PIN, LOW); 
  }
  else {
        digitalWrite(LED_PIN, HIGH); 
    
  bmeHeartRateAvgCharacteristics.setValue(bavgchar.c_str());
  bmeHeartRateAvgCharacteristics.notify();
     Serial.print("IR2=");
    Serial.print(irValue);
    Serial.print(", BPM2=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM2=");
    Serial.print(bavgchar);
    Serial.println(); 
  }
    
  }