#include <BLEUtils.h>
#include <math.h>
#include <Arduino.h>
#include "data.h"

inline int min(int a, int b) { return ((a)<(b) ? (a) : (b)); }

/*

The data schema is:

- 2 bytes sequencing header:
  - byte 1: stroke number (0-255, cycles back to 0)
  - byte 2: sequence number of a block of readings (0-255 max, will give 2.5 seconds at 50Hz)
- 18 bytes of data

totalling the maximum of 20 bytes per GATT packet. The client is expected to reassemble blocks of
readings using the sequence number for ordering.

We also send a terminating packet of:

  - byte 1: stroke number
  - byte 2: block number
  - byte 3: 0xff
  - byte 4: 0xff

*/

void sendData (uint8_t data[], int stroke_index, int size, BLECharacteristic* pCharacteristic) {
  int index = 0;
  int tx_index = 0;
  int packet_index = 0;
  uint8_t tx_buffer[BLE_MAX_BYTES];

  // TODO: need to add current time (millis) and length of packet to buffer
  
  while (index < size) {
    // Set the header bytes in the tx_buffer
    packet_index++;
    tx_buffer[0] = stroke_index;
    tx_buffer[1] = packet_index;
    tx_index = BLE_HEADER_BYTES;

    // Transfer a chunk of payload data to the tx_buffer
    for (int i=index; i<min(index + BLE_PAYLOAD_BYTES, size); i += BLE_DATA_LENGTH) {
      tx_buffer[tx_index] = data[index];
      tx_buffer[tx_index+1] = data[index+1];
      tx_index += BLE_DATA_LENGTH;
    }

    // Send the buffer using a notify
    // TODO: try using indicate instead, to miniimise the risk of dropping packets
    pCharacteristic->setValue(tx_buffer, BLE_MAX_BYTES);
    pCharacteristic->notify();

    // Increment the readings buffer index 
    index += BLE_PAYLOAD_BYTES;
    
    delay(20); // bluetooth stack will go into congestion, if too many packets are sent
  }

  // We need to send a trailing packet to indicate that this stroke data has been sent.
  // For now, the safest value to send is to max out both bytes, i.e., an unrealistically high value
  packet_index++;
  tx_buffer[0] = stroke_index;
  tx_buffer[1] = packet_index;
  tx_buffer[2] = 0xff;
  tx_buffer[3] = 0xff;

  pCharacteristic->setValue(tx_buffer, BLE_MAX_BYTES);
  pCharacteristic->notify();
}
