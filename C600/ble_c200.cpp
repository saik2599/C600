/*
    Author: Christopher Glanville
    Description:    Bluetooth low energy class for handling the connection to BLE devices. Nons of the 
                    data is processed here, but instead sent to a higher application layer (modbus_c200.cpp)
                    for processing. 
   Date: 25/5/24
   updated : 25/5/24
*/
#include <ArduinoBLE.h>

#include "C200.h"   // Required for loop_two()
#include "modbus_c200.h"

// UUID are used to communicate with different BLE modules. The two I have specified here are simulating a 
// JDY-33 dual mode BLE chip and the Microchip BM78 module. These UUIDs are used because the app HMI Droid 
// is known to work with them both, other than that if we were using our own app we could apecify our own 
// UUID of any type. These UUID simply provide compatability. 
// JDY-33
//#define SERVICE_UUID              "0000ffe0-0000-1000-8000-00805f9b34fb"
//#define CHARACTERISTIC_UUID_RX    "0000ffe1-0000-1000-8000-00805f9b34fb"
//#define CHARACTERISTIC_UUID_TX    "0000ffe2-0000-1000-8000-00805f9b34fb"

#define SERVICE_UUID                "49535343-FE7D-4AE5-8FA9-9FAFD205E455"
#define CHARACTERISTIC_UUID_TX      "49535343-1E4D-4BD9-BA61-23C647249616"
#define CHARACTERISTIC_UUID_RX      "49535343-8841-43F4-A8D4-ECBE34729BB3"

BLEService modbusService(SERVICE_UUID); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLECharacteristic modbusCharacteristicRx(CHARACTERISTIC_UUID_RX, BLERead | BLEWrite, 16);
BLEByteCharacteristic modbusCharacteristicTx(CHARACTERISTIC_UUID_TX, BLERead | BLENotify);

/*
    Name:         ble_init
    Arguments:    none
    Returns:      nil
    Description:  ble_init needs to be called before the main loop processing. This will setup the BLE device on the Giga and broadcase 
                  using the name specified below ("C200"). Once the BLE radio has been initialized data is gathered by the loop below
                  which is called repeatedly. 
*/
void ble_init()
{
  // set advertised local name and service UUID:
  BLE.begin();
  BLE.setLocalName("C300-2");  
  BLE.setAdvertisedService(modbusService);

  // add the characteristic to the service
  modbusService.addCharacteristic(modbusCharacteristicRx);
  modbusService.addCharacteristic(modbusCharacteristicTx);

  // add service
  BLE.addService(modbusService);

  // start advertising
  BLE.advertise();

}

/*
    Name:         ble_loop
    Arguments:    none
    Returns:      nil - data is passed from this loop directly into a MODBUS processing data buffer
    Description:  The main data processing loop for receive data from the BLE interface. This function will enter 
                  receved bytes directly into the modbus data processing buffer.
                  IMPORTANT! This is a blocking function - ARGH!?!??!!?!! Once the connection is made this function 
                  will no longer return. The only way I have managed to get around this was to call a second loop_two()
                  function once the connection is made. This will cause the stack to always have a depth of 1 while the BLE
                  interface is connected, however THERE WAS NO OTHER SOLUTION THAT COULD BE FOUND!!
*/
void ble_loop()
{
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if device is conneced to our peripheral...
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

     // while the central is still connected to peripheral:
    while (central.connected()) {
      if (modbusCharacteristicRx.written()){     
        uint8_t small_buffer[16]; 
        uint8_t ble_rx_d = modbusCharacteristicRx.readValue(small_buffer, 16);
        Serial.print("BLE Byte RX bytes ");
        Serial.println(ble_rx_d);
        for(int i = 0; i < ble_rx_d; i++){
          modbuxRxData(small_buffer[i], BLE_BUFFER);
        }
      }

      if (modbus_loop(BLE_BUFFER)){
        int tx_bytes = get_tx_bytes(BLE_BUFFER);
        uint8_t * temp_ble_buffer_ptr = get_tx_buffer(BLE_BUFFER);
        Serial.print("BLE Byte TX bytes ");
        Serial.println(tx_bytes);
        for(int i = 0; i < tx_bytes; i++){
          modbusCharacteristicTx.writeValue(temp_ble_buffer_ptr[i]);
        }
      }
      loop_two();     // FUCKING Arduino!
    }
    Serial.println("Connected BLE disconnected");
  } // end if (central)
}

