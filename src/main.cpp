/*
 * From https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function/blob/master/ESP32_ADC_Read_Voltage_Accurate.ino
 */

#include <Arduino.h>
#include <float.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

// Local includes
#include "data.h"

TaskHandle_t Task1, Task2;
SemaphoreHandle_t semaphore;

double reading = 0;
double VCC = 3.14;

// Declaration for task functions
void gatherData( void * parameter );


BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t ble_data[BLE_DATA_LENGTH];
uint8_t readings[MAX_READINGS * BLE_DATA_LENGTH];
int num_readings = 0;
uint8_t readings_buffer[MAX_READINGS * BLE_DATA_LENGTH];
int num_buffer_readings = 0;
int num_return_readings = 0;
int stroke_index = 0;
int READING_INTERVAL_MS = 20; // Interval between readings
float READING_THRESHOLD_KGS = 0.1;
float NUM_RETURN_READINGS = 3; // the number of readings below threshold before we trigger a return and start sending BLE stroke data
uint32_t stroke_start;
char buffer[50];

bool data_available = false;
bool data_sending = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup() {
  Serial.begin(115200);
  /*
  analogReadResolution(12);             // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetWidth(12);                   // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
                                        //  9-bit gives an ADC range of 0-511
                                        // 10-bit gives an ADC range of 0-1023
                                        // 11-bit gives an ADC range of 0-2047
                                        // 12-bit gives an ADC range of 0-4095
  analogSetCycles(8);                   // Set number of cycles per sample, default is 8 and provides an optimal result, range is 1 - 255
  analogSetSamples(1);                  // Set number of samples in the range, default is 1, it has an effect on sensitivity has been multiplied
  analogSetClockDiv(1);                 // Set the divider for the ADC clock, default is 1, range is 1 - 255
  analogSetAttenuation(ADC_11db);       // Sets the input attenuation for ALL ADC inputs, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
  analogSetPinAttenuation(VP,ADC_11db); // Sets the input attenuation, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
                                        // ADC_0db provides no attenuation so IN/OUT = 1 / 1 an input of 3 volts remains at 3 volts before ADC measurement
                                        // ADC_2_5db provides an attenuation so that IN/OUT = 1 / 1.34 an input of 3 volts is reduced to 2.238 volts before ADC measurement
                                        // ADC_6db provides an attenuation so that IN/OUT = 1 / 2 an input of 3 volts is reduced to 1.500 volts before ADC measurement
                                        // ADC_11db provides an attenuation so that IN/OUT = 1 / 3.6 an input of 3 volts is reduced to 0.833 volts before ADC measurement
  adcAttachPin(VP);                     // Attach a pin to ADC (also clears any other analog mode that could be on), returns TRUE/FALSE result 
  adcStart(VP);                         // Starts an ADC conversion on attached pin's bus
  adcBusy(VP);                          // Check if conversion on the pin's ADC bus is currently running, returns TRUE/FALSE result 
  adcEnd(VP);                           // Get the result of the conversion (will wait if it have not finished), returns 16-bit integer result
  */

  semaphore = xSemaphoreCreateMutex();

// A viewer suggested to use :     &codeForTask1, because his ESP crashed

  // Gathering data on core 1
  xTaskCreatePinnedToCore(
    &gatherData,       /* Function to implement the task */
    "dataGathering",  /* Name of the task */
    4096,             /* Stack size in words */
    NULL,             /* Task input parameter */
    0,                /* priority */
    &Task1,           /* Task handle. */
    1);               /* Core where the task should run */
    
  delay(500);  // needed to start-up task 1

  /*
  xTaskCreatePinnedToCore(
    sendDataViaBLE,
    "BLEcomms",
    1000,
    NULL,
    1,
    &Task2,
    0);
    */
  
  // Set up the analog read pin
  //adcAttachPin(ADC1_CHANNEL_0_GPIO_NUM);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
 
  //analogSetClockDiv(255); // 1338mS  

  // Create the BLE Device
  BLEDevice::init("PowerLogger");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY ||
                      BLECharacteristic::PROPERTY_INDICATE ||
                      BLECharacteristic::PROPERTY_WRITE ||
                      BLECharacteristic::PROPERTY_READ
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor for clients to notify us that someone is listening
  BLE2902 *ble2902 = new BLE2902();
  ble2902->setNotifications(true);
  pCharacteristic->addDescriptor(ble2902);

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");  
}

double readVoltage(byte pin){
  //double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  int reading = adc1_get_raw(ADC1_CHANNEL_0);
  sprintf(buffer, "raw = %d", reading);
  Serial.println(buffer);  
  if (reading < 1 || reading > 4095) return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
} // Added an improved polynomial, use either, comment out as required

double voltageToOhms (double voltage) {
  if (voltage <= 0) return DBL_MAX;
  return 330.0 * ((VCC/voltage) - 1.0);
}

double ohmsToKgs (double ohms) {
  if (ohms == DBL_MAX) return 0;
  return 52952 * pow(ohms,-1.55);
}

void doubleToIntArray(double val) {
  uint16_t tempValue;
  // multiply by 100 to get 2 digits mantissa and convert into uint16_t
  tempValue = (uint16_t)(val * 100);
  // set  LSB of array
  ble_data[0] = tempValue;
  // set MSB of characteristic
  ble_data[1] = tempValue>>8;
}

/* 
 * We're going to store all stroke readings until the force drops below a given threshhold, and then send all readings at once.  
 */
void gatherData( void * parameter )
{
  for (;;) {
    double voltage = readVoltage(ADC1_CHANNEL_0_GPIO_NUM);
    double ohms = voltageToOhms(voltage);
    reading = ohmsToKgs(ohms);

    sprintf(buffer, "force = %f", reading);
    Serial.println(buffer);

    // Append to the readings array
    if (reading > READING_THRESHOLD_KGS) {
      data_available = false;
      doubleToIntArray(reading); // populates ble_data
      for (int i=0; i<BLE_DATA_LENGTH; i++) {
        readings[num_readings * BLE_DATA_LENGTH + i] = ble_data[i];
      }
      num_readings++;
      num_return_readings = 0;
    }
    else {
      num_return_readings++;

      if (num_return_readings >= NUM_RETURN_READINGS) {
        // We're defaulting returning from a stroke now
        // Take the semaphore because we want to lock both the readings_buffer and data_available flag 
        //xSemaphoreTake( semaphore, portMAX_DELAY );

        // Transfer all available readings to a buffer so we can start sending them and continue to accumulate
        // readings for the next batch
        int index = 0;
        for (int r=0; r<num_readings; r++) {
          for (int i=0; i<BLE_DATA_LENGTH; i++) {
            index = r * BLE_DATA_LENGTH + i;
            readings_buffer[index] = ble_data[index];
          }          
        }
        num_buffer_readings = num_readings;
        num_readings = 0;

        // Readings buffer populated and data_available flag set, so give the semaphore back so the sending can start
        data_available = true;
        //xSemaphoreGive( semaphore );
      }
    }
    delay(READING_INTERVAL_MS);        
  }
}

void loop() {
    // notify changed value
    if (deviceConnected && data_available && !data_sending) {
        // Wait here for the semaphore to indicate that the data is RTS, set the flag indicating we're sending then hand back the semaphore
        data_sending = true;
        xSemaphoreGive( semaphore );        

        // Send a stroke index for sequencing, cycling back to 1
        stroke_index = (stroke_index + 1) % 255;
        sprintf(buffer, "stroke_index = %d", stroke_index);
        Serial.println(buffer);

        // Send all the data. Depending on the length of the stroke, could take 0.5-1.0 seconds 
        // This will return if data becomes available and we haven't finished sending yet
        sendData(readings_buffer, stroke_index, num_readings * BLE_DATA_LENGTH, pCharacteristic, &data_available);
        data_sending = false; // Possibly redundant as we're doing the sending synchronously
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}

/* ADC readings v voltage
 *  y = -0.000000000009824x3 + 0.000000016557283x2 + 0.000854596860691x + 0.065440348345433
 // Polynomial curve match, based on raw data thus:
 *   464     0.5
 *  1088     1.0
 *  1707     1.5
 *  2331     2.0
 *  2951     2.5 
 *  3775     3.0
 *  
 */

// See: https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-adc.h
