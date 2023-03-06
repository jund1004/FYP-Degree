#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

using std::string;

MAX30105 particleSensor;

#define bleServerSensorName "MAX30102_ESP32Sub"
static BLEUUID bmeServiceUUID("7092b545-f5da-471a-b16f-1550f55ac087");
static BLEUUID bmeSensorHRACharacteristicUUID("501368d8-3d67-4b41-bde6-28df9aa6355a");

BLEServer *pServer = NULL;//added
static boolean doConnect = false;
bool oldDeviceConnected = false;//added
static boolean connected = false; 
//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;
 
//Characteristicd that we want to read
static BLERemoteCharacteristic* sensorHRACharacteristic;

//Activate notify
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};

char* sensorHRAChar;

float sensorHRA;

boolean newHRA = false;

//Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToServer(BLEAddress pAddress) {
   BLEClient* pClient = BLEDevice::createClient();
 
  // Connect to the remove BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");
 
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(bmeServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(bmeServiceUUID.toString().c_str());
    return (false);
  }
   // Obtain a reference to the characteristics in the service of the remote BLE server.
  sensorHRACharacteristic = pRemoteService->getCharacteristic(bmeSensorHRACharacteristicUUID);

  if (sensorHRACharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.println(" - Found our characteristics");
 
  //Assign callback functions for the Characteristics

  sensorHRACharacteristic->registerForNotify(hraNotifyCallback);
  return true;
}

//Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerSensorName) { //Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      doConnect = true; //Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};

//When the BLE Server sends a new humidity reading with the notify property
static void hraNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                    uint8_t* pData, size_t length, bool isNotify) {
  //store humidity value
  sensorHRAChar = (char*)pData;
  newHRA = true;
}

#define bleServerName "MAX30102_ESP32"
#define SERVICE_UUID "e07845d5-d0a0-4484-973e-d11d1ce1eb66"

bool deviceConnected = false;

BLECharacteristic bmeHeartRateCharacteristics("2e1bd78c-36f5-486b-89d3-c6c0df91d781", 
                                              BLECharacteristic::PROPERTY_READ |
                                              BLECharacteristic::PROPERTY_WRITE |
                                              BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeHeartRateDescriptor(BLEUUID((uint16_t)0x2903));

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void checkToReconnect() //added
{
  // disconnected so advertise
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Disconnected: start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connected so reset boolean control
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    Serial.println("Reconnected");
    oldDeviceConnected = deviceConnected;
  }
}

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

static const unsigned long REFRESH_INTERVAL1 = 3000; // ms
static unsigned long lastRefreshTime1 = 0;

boolean firstSensor = false;
boolean secondSensor = false;
String bleavgchar;
float bleSensor=0;

#define LED_PIN 5 // ESP32 pin GIOP18 connected to LED
#define LED_PIN2 18 // ESP32 pin GIOP18 connected to LED
#define BLINK_INTERVAL 100  // interval at which to blink LED (milliseconds)
int ledState = LOW;   // ledState used to set the LED
unsigned long previousMillis = 0;   // will store last time LED was updated

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {

        // Do stuff based on the command received from the app
        if (rxValue.find("A") != -1) { 
          digitalWrite(LED_PIN2, HIGH);
        }
        else if (rxValue.find("B") != -1) {
          digitalWrite(LED_PIN2, LOW);
        }
      }
    }
};

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);

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

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
  
    // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *bmeService = pServer->createService(SERVICE_UUID);

  bmeService->addCharacteristic(&bmeHeartRateCharacteristics);
  bmeHeartRateDescriptor.setValue("Heart Rate");
  bmeHeartRateCharacteristics.addDescriptor(new BLE2902());
  bmeHeartRateCharacteristics.setCallbacks(new MyCallbacks());

  // Start the service
  bmeService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {

  checkToReconnect();
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




  //if new temperature readings are available, print in the OLED
  if (irValue < 50000){
    digitalWrite(LED_PIN, LOW); 
  }
  else {
    digitalWrite(LED_PIN, HIGH); 
  }

	if(lastBeat - lastRefreshTime1 >= REFRESH_INTERVAL1)
	{
		lastRefreshTime1 += REFRESH_INTERVAL1;
   if (irValue < 50000){
    Serial.print("First Sensor Finger?");
    Serial.println();
    firstSensor = false;
  }
  else {
    Serial.print("Avg BPM=");
    Serial.print(beatAvg);
    Serial.println();
    firstSensor = true;
  }

  if (newHRA){
    newHRA = false;
    sensorHRA = atof(sensorHRAChar);
    if (sensorHRA<=0){
      Serial.print("Second Sensor Finger?");
      Serial.println();
      secondSensor = false;
    }
    else {
      Serial.print("Avg BPM2=");
      Serial.print(sensorHRA);
      Serial.println();
      secondSensor = true;
    }
  }
  

        if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      //Activate the Notify property of each Characteristic
      sensorHRACharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      connected = true;
    } else {
      Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
    }
    doConnect = false;
  }
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  bleSensor = (beatAvg + sensorHRA)/2;
  bleavgchar = String(bleSensor, 0);

      if(firstSensor && secondSensor){
      
      bmeHeartRateCharacteristics.setValue(bleavgchar.c_str());
      bmeHeartRateCharacteristics.notify(); 	
    }
    else{
    String zero = "No Finger";
    bmeHeartRateCharacteristics.setValue(zero.c_str());
    bmeHeartRateCharacteristics.notify(); 
    }

  }



}
