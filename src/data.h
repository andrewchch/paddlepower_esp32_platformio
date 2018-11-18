

#ifndef HEADER_DATA
#define HEADER_DATA

#define BLE_DATA_LENGTH 2
#define BLE_HEADER_BYTES 2
#define BLE_MAX_BYTES 20
#define BLE_PAYLOAD_BYTES 18
#define MIN_READINGS 50
#define MAX_READINGS 100
   
//Prototype for data helper functions found in data.cpp
void sendData (uint8_t data[], int stroke_index, int size, BLECharacteristic* pCharacteristic, bool* canSend);
   
#endif
