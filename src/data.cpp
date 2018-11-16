#include <BLEUtils.h>
#include <math.h>
#include <Arduino.h>
#include "data.h"

inline int min(int a, int b) { return ((a)<(b) ? (a) : (b)); }

/*

The data schema is:

- 2 bytes sequencing header:
  - byte 1: stroke number (0-255, cycles back to 0)
  - byte 2: reading number (0-255 max, will give 2.5 seconds at 50Hz)
- 18 bytes of data

totalling the maximum of 20 bytes per GATT packet.

We'll still break down 

*/

void sendData (uint8_t data[], int stroke_index, int size, BLECharacteristic* pCharacteristic) {
  int index = 0;
  int tx_index = 0;
  int packet_index = 0;
  uint8_t tx_buffer[BLE_MAX_BYTES];

  // TODO: need to add current time (millis) and length of packet to buffer
  
  while (index < size) {
    // Set the header bytes in the tx_buffer
    packet_index += 1;
    tx_buffer[0] = stroke_index;
    tx_buffer[1] = packet_index;
    tx_index = BLE_HEADER_BYTES;

    // Transfer a chunk of payload data to the tx_buffer
    for (int i=index; i<min(index + BLE_PAYLOAD_BYTES,size); i++) {
      tx_buffer[tx_index] = data[index];
      tx_index++;
    }

    // Send the buffer
    pCharacteristic->setValue(tx_buffer, BLE_MAX_BYTES);
    pCharacteristic->indicate();

    // Increment the readings buffer index 
    index += BLE_PAYLOAD_BYTES;
    
    delay(20); // bluetooth stack will go into congestion, if too many packets are sent
  }
}
