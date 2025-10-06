#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include "eeprom.h"
#include "config_c200.h"

#define PAGE_SIZE 256
#define CS_PIN 10

Adafruit_FlashTransport_SPI flashTransport(CS_PIN, SPI1);
Adafruit_SPIFlash flash(&flashTransport);

bool eeprom_online = false;

void printUniqueID() {
  uint8_t uniqueID[8];

  digitalWrite(CS_PIN, LOW);
  SPI1.transfer(0x4B); // Unique ID read command
  for (int i = 0; i < 4; i++) {
    SPI1.transfer(0x00); // Send dummy bytes
  }
  for (int i = 0; i < 8; i++) {
    uniqueID[i] = SPI1.transfer(0x00); // Read Unique ID
  }
  digitalWrite(CS_PIN, HIGH);

  for (int i = 0; i < 8; i++) {
    if (uniqueID[i] < 0x10) Serial.print("0"); // Leading zero for single hex digits
    Serial.print(uniqueID[i], HEX);
    if (i < 7) {
      Serial.print(":");
    }
  }
  Serial.println();
}

void eeprom_setup(void) {
  SPI1.begin();

  if (flash.begin()) {
    const uint32_t startAddress = 0x000000;
    uint32_t currentAddress = startAddress;
    int totalBytesToRead = 1279;
    uint8_t dataRead[PAGE_SIZE * 4];
    FLOATUNION_t read_config_var;

    Serial.println("SPIFlash library initialized.");
    eeprom_online = false; // Disable the EEPROM writes while reading to prevent a loopback effect

    uint32_t jedecID = flash.getJEDECID();
    Serial.print("JEDEC ID: 0x");
    Serial.println(jedecID, HEX);

    Serial.print("Unique ID: ");
    printUniqueID();

    for (int i = 0; i < totalBytesToRead; i += PAGE_SIZE) {
      int bytesToRead = min(PAGE_SIZE, totalBytesToRead - i);

      if (flash.readBuffer(currentAddress, dataRead, bytesToRead)) {
        Serial.print("eeprom_setup read Data: ");
        for (int j = 0; j < bytesToRead; j++) {
          Serial.print(dataRead[j], HEX);
          Serial.print(" ");
        }
        Serial.println();

        if (i == 0) {
          // First page (0 - 255)

          //Serial Number
          memcpy(read_config_var.bytes, &dataRead, 4);
          set_config_parameter(CONFIG_PARAM_SERIAL_NUM, read_config_var.number);
          //Slave ID
          memcpy(read_config_var.bytes, &dataRead[4], 4);
          if ((read_config_var.number > 0) && (read_config_var.number < 255)) {
            set_config_parameter(CONFIG_PARAM_SLAVE_ID, read_config_var.number);
          } else {
            Serial.println("Invalid Modbus ID read from EEPROM. Disregarding.");
            Serial.println(read_config_var.number);
          }

         //PT-405
        memcpy(read_config_var.bytes, &dataRead[8], 4);
        set_config_parameter(CONFIG_PARAM_PT405_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[12], 4);
        set_config_parameter(CONFIG_PARAM_PT405_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[16], 4);
        set_config_parameter(CONFIG_PARAM_PT405_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[20], 4);
        set_config_parameter(CONFIG_PARAM_PT405_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[24], 4);
        set_config_parameter(CONFIG_PARAM_PT405_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[28], 4);
        set_config_parameter(CONFIG_PARAM_PT405_STOP_MAX, read_config_var.number);

        //PT-426
        memcpy(read_config_var.bytes, &dataRead[32], 4);
        set_config_parameter(CONFIG_PARAM_PT426_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[36], 4);
        set_config_parameter(CONFIG_PARAM_PT426_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[40], 4);
        set_config_parameter(CONFIG_PARAM_PT426_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[44], 4);
        set_config_parameter(CONFIG_PARAM_PT426_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[48], 4);
        set_config_parameter(CONFIG_PARAM_PT426_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[52], 4);
        set_config_parameter(CONFIG_PARAM_PT426_STOP_MAX, read_config_var.number);

        //PT-442
        memcpy(read_config_var.bytes, &dataRead[56], 4);
        set_config_parameter(CONFIG_PARAM_PT442_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[60], 4);
        set_config_parameter(CONFIG_PARAM_PT442_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[64], 4);
        set_config_parameter(CONFIG_PARAM_PT442_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[68], 4);
        set_config_parameter(CONFIG_PARAM_PT442_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[72], 4);
        set_config_parameter(CONFIG_PARAM_PT442_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[76], 4);
        set_config_parameter(CONFIG_PARAM_PT442_STOP_MAX, read_config_var.number);

        //PT-410
        memcpy(read_config_var.bytes, &dataRead[80], 4);
        set_config_parameter(CONFIG_PARAM_PT410_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[84], 4);
        set_config_parameter(CONFIG_PARAM_PT410_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[88], 4);
        set_config_parameter(CONFIG_PARAM_PT410_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[92], 4);
        set_config_parameter(CONFIG_PARAM_PT410_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[96], 4);
        set_config_parameter(CONFIG_PARAM_PT410_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[100], 4);
        set_config_parameter(CONFIG_PARAM_PT410_STOP_MAX, read_config_var.number);

        //PT-457
        memcpy(read_config_var.bytes, &dataRead[104], 4);
        set_config_parameter(CONFIG_PARAM_PT457_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[108], 4);
        set_config_parameter(CONFIG_PARAM_PT457_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[112], 4);
        set_config_parameter(CONFIG_PARAM_PT457_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[116], 4);
        set_config_parameter(CONFIG_PARAM_PT457_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[120], 4);
        set_config_parameter(CONFIG_PARAM_PT457_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[124], 4);
        set_config_parameter(CONFIG_PARAM_PT457_STOP_MAX, read_config_var.number);

        //PT-416
        memcpy(read_config_var.bytes, &dataRead[128], 4);
        set_config_parameter(CONFIG_PARAM_PT416_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[132], 4);
        set_config_parameter(CONFIG_PARAM_PT416_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[136], 4);
        set_config_parameter(CONFIG_PARAM_PT416_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[140], 4);
        set_config_parameter(CONFIG_PARAM_PT416_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[144], 4);
        set_config_parameter(CONFIG_PARAM_PT416_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[148], 4);
        set_config_parameter(CONFIG_PARAM_PT416_STOP_MAX, read_config_var.number);

        //PT-468
        memcpy(read_config_var.bytes, &dataRead[152], 4);
        set_config_parameter(CONFIG_PARAM_PT468_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[156], 4);
        set_config_parameter(CONFIG_PARAM_PT468_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[160], 4);
        set_config_parameter(CONFIG_PARAM_PT468_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[164], 4);
        set_config_parameter(CONFIG_PARAM_PT468_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[168], 4);
        set_config_parameter(CONFIG_PARAM_PT468_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[172], 4);
        set_config_parameter(CONFIG_PARAM_PT468_STOP_MAX, read_config_var.number);

        //PT-531
        memcpy(read_config_var.bytes, &dataRead[176], 4);
        set_config_parameter(CONFIG_PARAM_PT531_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[180], 4);
        set_config_parameter(CONFIG_PARAM_PT531_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[184], 4);
        set_config_parameter(CONFIG_PARAM_PT531_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[188], 4);
        set_config_parameter(CONFIG_PARAM_PT531_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[192], 4);
        set_config_parameter(CONFIG_PARAM_PT531_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[196], 4);
        set_config_parameter(CONFIG_PARAM_PT531_STOP_MAX, read_config_var.number);

        //PT-576
        memcpy(read_config_var.bytes, &dataRead[200], 4);
        set_config_parameter(CONFIG_PARAM_PT576_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[204], 4);
        set_config_parameter(CONFIG_PARAM_PT576_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[208], 4);
        set_config_parameter(CONFIG_PARAM_PT576_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[212], 4);
        set_config_parameter(CONFIG_PARAM_PT576_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[216], 4);
        set_config_parameter(CONFIG_PARAM_PT576_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[220], 4);
        set_config_parameter(CONFIG_PARAM_PT576_STOP_MAX, read_config_var.number);

        //PT-487
        memcpy(read_config_var.bytes, &dataRead[224], 4);
        set_config_parameter(CONFIG_PARAM_PT487_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[228], 4);
        set_config_parameter(CONFIG_PARAM_PT487_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[232], 4);
        set_config_parameter(CONFIG_PARAM_PT487_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[236], 4);
        set_config_parameter(CONFIG_PARAM_PT487_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[240], 4);
        set_config_parameter(CONFIG_PARAM_PT487_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[244], 4);
        set_config_parameter(CONFIG_PARAM_PT487_STOP_MAX, read_config_var.number);

        } 
        else if (i == PAGE_SIZE) {
          // Second page (256 - 511) 
        //PT-496
        memcpy(read_config_var.bytes, &dataRead, 4);
        set_config_parameter(CONFIG_PARAM_PT496_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[4], 4);
        set_config_parameter(CONFIG_PARAM_PT496_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[8], 4);
        set_config_parameter(CONFIG_PARAM_PT496_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[12], 4);
        set_config_parameter(CONFIG_PARAM_PT496_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[16], 4);
        set_config_parameter(CONFIG_PARAM_PT496_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[20], 4);
        set_config_parameter(CONFIG_PARAM_PT496_STOP_MAX, read_config_var.number);

        //TT-432
        memcpy(read_config_var.bytes, &dataRead[24], 4);
        set_config_parameter(CONFIG_PARAM_TT432_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[28], 4);
        set_config_parameter(CONFIG_PARAM_TT432_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[32], 4);
        set_config_parameter(CONFIG_PARAM_TT432_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[36], 4);
        set_config_parameter(CONFIG_PARAM_TT432_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[40], 4);
        set_config_parameter(CONFIG_PARAM_TT432_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[44], 4);
        set_config_parameter(CONFIG_PARAM_TT432_STOP_MAX, read_config_var.number);

       //TT-439
        memcpy(read_config_var.bytes, &dataRead[48], 4);
        set_config_parameter(CONFIG_PARAM_TT439_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[52], 4);
        set_config_parameter(CONFIG_PARAM_TT439_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[56], 4);
        set_config_parameter(CONFIG_PARAM_TT439_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[60], 4);
        set_config_parameter(CONFIG_PARAM_TT439_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[64], 4);
        set_config_parameter(CONFIG_PARAM_TT439_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[68], 4);
        set_config_parameter(CONFIG_PARAM_TT439_STOP_MAX, read_config_var.number);

        //TT-435
        memcpy(read_config_var.bytes, &dataRead[72], 4);
        set_config_parameter(CONFIG_PARAM_TT435_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[76], 4);
        set_config_parameter(CONFIG_PARAM_TT435_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[80], 4);
        set_config_parameter(CONFIG_PARAM_TT435_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[84], 4);
        set_config_parameter(CONFIG_PARAM_TT435_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[88], 4);
        set_config_parameter(CONFIG_PARAM_TT435_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[92], 4);
        set_config_parameter(CONFIG_PARAM_TT435_STOP_MAX, read_config_var.number);

        //TT-436
        memcpy(read_config_var.bytes, &dataRead[96], 4);
        set_config_parameter(CONFIG_PARAM_TT436_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[100], 4);
        set_config_parameter(CONFIG_PARAM_TT436_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[104], 4);
        set_config_parameter(CONFIG_PARAM_TT436_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[108], 4);
        set_config_parameter(CONFIG_PARAM_TT436_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[112], 4);
        set_config_parameter(CONFIG_PARAM_TT436_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[116], 4);
        set_config_parameter(CONFIG_PARAM_TT436_STOP_MAX, read_config_var.number);

        //TT-447
        memcpy(read_config_var.bytes, &dataRead[120], 4);
        set_config_parameter(CONFIG_PARAM_TT447_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[124], 4);
        set_config_parameter(CONFIG_PARAM_TT447_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[128], 4);
        set_config_parameter(CONFIG_PARAM_TT447_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[132], 4);
        set_config_parameter(CONFIG_PARAM_TT447_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[136], 4);
        set_config_parameter(CONFIG_PARAM_TT447_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[140], 4);
        set_config_parameter(CONFIG_PARAM_TT447_STOP_MAX, read_config_var.number);

        //TT-454
        memcpy(read_config_var.bytes, &dataRead[144], 4);
        set_config_parameter(CONFIG_PARAM_TT454_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[148], 4);
        set_config_parameter(CONFIG_PARAM_TT454_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[152], 4);
        set_config_parameter(CONFIG_PARAM_TT454_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[156], 4);
        set_config_parameter(CONFIG_PARAM_TT454_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[160], 4);
        set_config_parameter(CONFIG_PARAM_TT454_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[164], 4);
        set_config_parameter(CONFIG_PARAM_TT454_STOP_MAX, read_config_var.number);

        //TT-450
        memcpy(read_config_var.bytes, &dataRead[168], 4);
        set_config_parameter(CONFIG_PARAM_TT450_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[172], 4);
        set_config_parameter(CONFIG_PARAM_TT450_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[176], 4);
        set_config_parameter(CONFIG_PARAM_TT450_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[180], 4);
        set_config_parameter(CONFIG_PARAM_TT450_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[184], 4);
        set_config_parameter(CONFIG_PARAM_TT450_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[188], 4);
        set_config_parameter(CONFIG_PARAM_TT450_STOP_MAX, read_config_var.number);

        //TT-451
        memcpy(read_config_var.bytes, &dataRead[192], 4);
        set_config_parameter(CONFIG_PARAM_TT451_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[196], 4);
        set_config_parameter(CONFIG_PARAM_TT451_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[200], 4);
        set_config_parameter(CONFIG_PARAM_TT451_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[204], 4);
        set_config_parameter(CONFIG_PARAM_TT451_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[208], 4);
        set_config_parameter(CONFIG_PARAM_TT451_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[212], 4);
        set_config_parameter(CONFIG_PARAM_TT451_STOP_MAX, read_config_var.number);

        //TT-462
        memcpy(read_config_var.bytes, &dataRead[216], 4);
        set_config_parameter(CONFIG_PARAM_TT462_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[220], 4);
        set_config_parameter(CONFIG_PARAM_TT462_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[224], 4);
        set_config_parameter(CONFIG_PARAM_TT462_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[228], 4);
        set_config_parameter(CONFIG_PARAM_TT462_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[232], 4);
        set_config_parameter(CONFIG_PARAM_TT462_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[236], 4);
        set_config_parameter(CONFIG_PARAM_TT462_STOP_MAX, read_config_var.number);

        }  
        else if (i == 2 * PAGE_SIZE) {
        // Third page (512 - 767)
          //TT-474
        memcpy(read_config_var.bytes, &dataRead, 4);
        set_config_parameter(CONFIG_PARAM_TT474_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[4], 4);
        set_config_parameter(CONFIG_PARAM_TT474_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[8], 4);
        set_config_parameter(CONFIG_PARAM_TT474_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[12], 4);
        set_config_parameter(CONFIG_PARAM_TT474_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[16], 4);
        set_config_parameter(CONFIG_PARAM_TT474_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[20], 4);
        set_config_parameter(CONFIG_PARAM_TT474_STOP_MAX, read_config_var.number);

        //TT-465
        memcpy(read_config_var.bytes, &dataRead[24], 4);
        set_config_parameter(CONFIG_PARAM_TT465_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[28], 4);
        set_config_parameter(CONFIG_PARAM_TT465_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[32], 4);
        set_config_parameter(CONFIG_PARAM_TT465_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[36], 4);
        set_config_parameter(CONFIG_PARAM_TT465_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[40], 4);
        set_config_parameter(CONFIG_PARAM_TT465_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[44], 4);
        set_config_parameter(CONFIG_PARAM_TT465_STOP_MAX, read_config_var.number);

        //TT-471
        memcpy(read_config_var.bytes, &dataRead[48], 4);
        set_config_parameter(CONFIG_PARAM_TT471_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[52], 4);
        set_config_parameter(CONFIG_PARAM_TT471_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[56], 4);
        set_config_parameter(CONFIG_PARAM_TT471_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[60], 4);
        set_config_parameter(CONFIG_PARAM_TT471_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[64], 4);
        set_config_parameter(CONFIG_PARAM_TT471_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[68], 4);
        set_config_parameter(CONFIG_PARAM_TT471_STOP_MAX, read_config_var.number);

        //TT-532
        memcpy(read_config_var.bytes, &dataRead[72], 4);
        set_config_parameter(CONFIG_PARAM_TT532_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[76], 4);
        set_config_parameter(CONFIG_PARAM_TT532_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[80], 4);
        set_config_parameter(CONFIG_PARAM_TT532_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[84], 4);
        set_config_parameter(CONFIG_PARAM_TT532_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[88], 4);
        set_config_parameter(CONFIG_PARAM_TT532_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[92], 4);
        set_config_parameter(CONFIG_PARAM_TT532_STOP_MAX, read_config_var.number);    

        //TT-557
        memcpy(read_config_var.bytes, &dataRead[96], 4);
        set_config_parameter(CONFIG_PARAM_TT557_STOP_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[100], 4);
        set_config_parameter(CONFIG_PARAM_TT557_IDLE_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[104], 4);
        set_config_parameter(CONFIG_PARAM_TT557_RUN_MIN, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[108], 4);
        set_config_parameter(CONFIG_PARAM_TT557_RUN_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[112], 4);
        set_config_parameter(CONFIG_PARAM_TT557_IDLE_MAX, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[116], 4);
        set_config_parameter(CONFIG_PARAM_TT557_STOP_MAX, read_config_var.number);

        memcpy(read_config_var.bytes, &dataRead[120], 4);
        set_config_parameter(CONFIG_PARAM_S1A_FLOW_COUNT, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[124], 4);
        set_config_parameter(CONFIG_PARAM_S1B_FLOW_COUNT, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[128], 4);
        set_config_parameter(CONFIG_PARAM_S2A_FLOW_COUNT, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[132], 4);
        set_config_parameter(CONFIG_PARAM_S2B_FLOW_COUNT, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[136], 4);
        set_config_parameter(CONFIG_PARAM_S3A_FLOW_COUNT, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[140], 4);
        set_config_parameter(CONFIG_PARAM_S3B_FLOW_COUNT, read_config_var.number);

      ///Left some space - this for debugging CPM issue
        memcpy(read_config_var.bytes, &dataRead[156], 4);
        set_config_parameter(CONFIG_PARAM_S1_MIN_CPM, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[160], 4);
        set_config_parameter(CONFIG_PARAM_S2_MIN_CPM, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[164], 4);
        set_config_parameter(CONFIG_PARAM_S3_MIN_CPM, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[168], 4);
        set_config_parameter(CONFIG_PARAM_S1_MAX_CPM, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[172], 4);
        set_config_parameter(CONFIG_PARAM_S2_MAX_CPM, read_config_var.number);
        memcpy(read_config_var.bytes, &dataRead[176], 4);
        set_config_parameter(CONFIG_PARAM_S3_MAX_CPM, read_config_var.number);
        
        } 
        Serial.println("SPIFlash read completed.");
      } else {
        Serial.println("Failed to read data.");
        break;
      }

      currentAddress += bytesToRead;
    }
    eeprom_online = true;
  } else {
    Serial.println("Failed to initialize SPIFlash library.");
  }
}

void eeprom_save(void) {
  if (!eeprom_online) {
    Serial.println("EEPROM Write blocked by eeprom_online.");
    return;
  }

  FLOATUNION_t save_config_var;
  const uint32_t startAddress = 0x000000;
  uint8_t dataToWrite[PAGE_SIZE * 4];
  uint32_t currentAddress = startAddress;
  int totalBytesToWrite = 1279;
  
        // SERIAL_NUMBER
        save_config_var.number = get_config_parameter(CONFIG_PARAM_SERIAL_NUM);
        memcpy(&dataToWrite, save_config_var.bytes, 4);
        
        // SLAVE_ID
        save_config_var.number = get_config_parameter(CONFIG_PARAM_SLAVE_ID);
        memcpy(&dataToWrite[4], save_config_var.bytes, 4);

        // PT-405
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT405_STOP_MIN);
        memcpy(&dataToWrite[8], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT405_IDLE_MIN);
        memcpy(&dataToWrite[12], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT405_RUN_MIN);
        memcpy(&dataToWrite[16], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT405_RUN_MAX);
        memcpy(&dataToWrite[20], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT405_IDLE_MAX);
        memcpy(&dataToWrite[24], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT405_STOP_MAX);
        memcpy(&dataToWrite[28], save_config_var.bytes, 4);
        
        //PT-426
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT426_STOP_MIN);
        memcpy(&dataToWrite[32], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT426_IDLE_MIN);
        memcpy(&dataToWrite[36], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT426_RUN_MIN);
        memcpy(&dataToWrite[40], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT426_RUN_MAX);
        memcpy(&dataToWrite[44], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT426_IDLE_MAX);
        memcpy(&dataToWrite[48], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT426_STOP_MAX);
        memcpy(&dataToWrite[52], save_config_var.bytes, 4);

        //PT-442
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT442_STOP_MIN);
        memcpy(&dataToWrite[56], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT442_IDLE_MIN);
        memcpy(&dataToWrite[60], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT442_RUN_MIN);
        memcpy(&dataToWrite[64], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT442_RUN_MAX);
        memcpy(&dataToWrite[68], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT442_IDLE_MAX);
        memcpy(&dataToWrite[72], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT442_STOP_MAX);
        memcpy(&dataToWrite[76], save_config_var.bytes, 4);

        //PT-410
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT410_STOP_MIN);
        memcpy(&dataToWrite[80], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT410_IDLE_MIN);
        memcpy(&dataToWrite[84], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT410_RUN_MIN);
        memcpy(&dataToWrite[88], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT410_RUN_MAX);
        memcpy(&dataToWrite[92], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT410_IDLE_MAX);
        memcpy(&dataToWrite[96], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT410_STOP_MAX);
        memcpy(&dataToWrite[100], save_config_var.bytes, 4);

        //PT-457
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT457_STOP_MIN);
        memcpy(&dataToWrite[104], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT457_IDLE_MIN);
        memcpy(&dataToWrite[108], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT457_RUN_MIN);
        memcpy(&dataToWrite[112], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT457_RUN_MAX);
        memcpy(&dataToWrite[116], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT457_IDLE_MAX);
        memcpy(&dataToWrite[120], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT457_STOP_MAX);
        memcpy(&dataToWrite[124], save_config_var.bytes, 4);

        //PT-416
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT416_STOP_MIN);
        memcpy(&dataToWrite[128], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT416_IDLE_MIN);
        memcpy(&dataToWrite[132], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT416_RUN_MIN);
        memcpy(&dataToWrite[136], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT416_RUN_MAX);
        memcpy(&dataToWrite[140], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT416_IDLE_MAX);
        memcpy(&dataToWrite[144], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT416_STOP_MAX);
        memcpy(&dataToWrite[148], save_config_var.bytes, 4);

        //PT-468
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT468_STOP_MIN);
        memcpy(&dataToWrite[152], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT468_IDLE_MIN);
        memcpy(&dataToWrite[156], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT468_RUN_MIN);
        memcpy(&dataToWrite[160], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT468_RUN_MAX);
        memcpy(&dataToWrite[164], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT468_IDLE_MAX);
        memcpy(&dataToWrite[168], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT468_STOP_MAX);
        memcpy(&dataToWrite[172], save_config_var.bytes, 4);    

        //PT-531
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT531_STOP_MIN);
        memcpy(&dataToWrite[176], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT531_IDLE_MIN);
        memcpy(&dataToWrite[180], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT531_RUN_MIN);
        memcpy(&dataToWrite[184], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT531_RUN_MAX);
        memcpy(&dataToWrite[188], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT531_IDLE_MAX);
        memcpy(&dataToWrite[192], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT531_STOP_MAX);
        memcpy(&dataToWrite[196], save_config_var.bytes, 4);    

        //PT-576
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT576_STOP_MIN);
        memcpy(&dataToWrite[200], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT576_IDLE_MIN);
        memcpy(&dataToWrite[204], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT576_RUN_MIN);
        memcpy(&dataToWrite[208], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT576_RUN_MAX);
        memcpy(&dataToWrite[212], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT576_IDLE_MAX);
        memcpy(&dataToWrite[216], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT576_STOP_MAX);
        memcpy(&dataToWrite[220], save_config_var.bytes, 4);        

        //PT-487
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT487_STOP_MIN);
        memcpy(&dataToWrite[224], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT487_IDLE_MIN);
        memcpy(&dataToWrite[228], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT487_RUN_MIN);
        memcpy(&dataToWrite[232], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT487_RUN_MAX);
        memcpy(&dataToWrite[236], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT487_IDLE_MAX);
        memcpy(&dataToWrite[240], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT487_STOP_MAX);
        memcpy(&dataToWrite[244], save_config_var.bytes, 4);        

        //PT-496  BLOCK2 Started
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT496_STOP_MIN);
        memcpy(&dataToWrite[256], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT496_IDLE_MIN);
        memcpy(&dataToWrite[260], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT496_RUN_MIN);
        memcpy(&dataToWrite[264], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT496_RUN_MAX);
        memcpy(&dataToWrite[268], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT496_IDLE_MAX);
        memcpy(&dataToWrite[272], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_PT496_STOP_MAX);
        memcpy(&dataToWrite[276], save_config_var.bytes, 4);        

        //TT-432
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT432_STOP_MIN);
        memcpy(&dataToWrite[280], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT432_IDLE_MIN);
        memcpy(&dataToWrite[284], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT432_RUN_MIN);
        memcpy(&dataToWrite[288], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT432_RUN_MAX);
        memcpy(&dataToWrite[292], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT432_IDLE_MAX);
        memcpy(&dataToWrite[296], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT432_STOP_MAX);
        memcpy(&dataToWrite[300], save_config_var.bytes, 4);        

        //TT-439
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT439_STOP_MIN);
        memcpy(&dataToWrite[304], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT439_IDLE_MIN);
        memcpy(&dataToWrite[308], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT439_RUN_MIN);
        memcpy(&dataToWrite[312], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT439_RUN_MAX);
        memcpy(&dataToWrite[316], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT439_IDLE_MAX);
        memcpy(&dataToWrite[320], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT439_STOP_MAX);
        memcpy(&dataToWrite[324], save_config_var.bytes, 4);        

        //TT-435
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT435_STOP_MIN);
        memcpy(&dataToWrite[328], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT435_IDLE_MIN);
        memcpy(&dataToWrite[332], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT435_RUN_MIN);
        memcpy(&dataToWrite[336], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT435_RUN_MAX);
        memcpy(&dataToWrite[340], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT435_IDLE_MAX);
        memcpy(&dataToWrite[344], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT435_STOP_MAX);
        memcpy(&dataToWrite[348], save_config_var.bytes, 4);        

        //TT-436
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT436_STOP_MIN);
        memcpy(&dataToWrite[352], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT436_IDLE_MIN);
        memcpy(&dataToWrite[356], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT436_RUN_MIN);
        memcpy(&dataToWrite[360], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT436_RUN_MAX);
        memcpy(&dataToWrite[364], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT436_IDLE_MAX);
        memcpy(&dataToWrite[368], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT436_STOP_MAX);
        memcpy(&dataToWrite[372], save_config_var.bytes, 4);        

        //TT-447
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT447_STOP_MIN);
        memcpy(&dataToWrite[376], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT447_IDLE_MIN);
        memcpy(&dataToWrite[380], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT447_RUN_MIN);
        memcpy(&dataToWrite[384], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT447_RUN_MAX);
        memcpy(&dataToWrite[388], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT447_IDLE_MAX);
        memcpy(&dataToWrite[392], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT447_STOP_MAX);
        memcpy(&dataToWrite[396], save_config_var.bytes, 4);        

        //TT-454
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT454_STOP_MIN);
        memcpy(&dataToWrite[400], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT454_IDLE_MIN);
        memcpy(&dataToWrite[404], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT454_RUN_MIN);
        memcpy(&dataToWrite[408], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT454_RUN_MAX);
        memcpy(&dataToWrite[412], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT454_IDLE_MAX);
        memcpy(&dataToWrite[416], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT454_STOP_MAX);
        memcpy(&dataToWrite[420], save_config_var.bytes, 4);        

        //TT-450
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT450_STOP_MIN);
        memcpy(&dataToWrite[424], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT450_IDLE_MIN);
        memcpy(&dataToWrite[428], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT450_RUN_MIN);
        memcpy(&dataToWrite[432], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT450_RUN_MAX);
        memcpy(&dataToWrite[436], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT450_IDLE_MAX);
        memcpy(&dataToWrite[440], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT450_STOP_MAX);
        memcpy(&dataToWrite[444], save_config_var.bytes, 4);        

        //TT-451
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT451_STOP_MIN);
        memcpy(&dataToWrite[448], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT451_IDLE_MIN);
        memcpy(&dataToWrite[452], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT451_RUN_MIN);
        memcpy(&dataToWrite[456], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT451_RUN_MAX);
        memcpy(&dataToWrite[460], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT451_IDLE_MAX);
        memcpy(&dataToWrite[464], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT451_STOP_MAX);
        memcpy(&dataToWrite[468], save_config_var.bytes, 4);        

        //TT-462
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT462_STOP_MIN);
        memcpy(&dataToWrite[472], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT462_IDLE_MIN);
        memcpy(&dataToWrite[476], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT462_RUN_MIN);
        memcpy(&dataToWrite[480], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT462_RUN_MAX);
        memcpy(&dataToWrite[484], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT462_IDLE_MAX);
        memcpy(&dataToWrite[488], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT462_STOP_MAX);
        memcpy(&dataToWrite[492], save_config_var.bytes, 4);        

        //TT-474 Block3 starts here
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT474_STOP_MIN);
        memcpy(&dataToWrite[512], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT474_IDLE_MIN);
        memcpy(&dataToWrite[516], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT474_RUN_MIN);
        memcpy(&dataToWrite[520], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT474_RUN_MAX);
        memcpy(&dataToWrite[524], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT474_IDLE_MAX);
        memcpy(&dataToWrite[528], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT474_STOP_MAX);
        memcpy(&dataToWrite[532], save_config_var.bytes, 4);        

        //TT-465
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT465_STOP_MIN);
        memcpy(&dataToWrite[536], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT465_IDLE_MIN);
        memcpy(&dataToWrite[540], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT465_RUN_MIN);
        memcpy(&dataToWrite[544], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT465_RUN_MAX);
        memcpy(&dataToWrite[548], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT465_IDLE_MAX);
        memcpy(&dataToWrite[552], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT465_STOP_MAX);
        memcpy(&dataToWrite[556], save_config_var.bytes, 4);        

        //TT-471
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT471_STOP_MIN);
        memcpy(&dataToWrite[560], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT471_IDLE_MIN);
        memcpy(&dataToWrite[564], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT471_RUN_MIN);
        memcpy(&dataToWrite[568], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT471_RUN_MAX);
        memcpy(&dataToWrite[572], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT471_IDLE_MAX);
        memcpy(&dataToWrite[576], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT471_STOP_MAX);
        memcpy(&dataToWrite[580], save_config_var.bytes, 4);        

        //TT-532
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT532_STOP_MIN);
        memcpy(&dataToWrite[584], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT532_IDLE_MIN);
        memcpy(&dataToWrite[588], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT532_RUN_MIN);
        memcpy(&dataToWrite[592], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT532_RUN_MAX);
        memcpy(&dataToWrite[596], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT532_IDLE_MAX);
        memcpy(&dataToWrite[600], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT532_STOP_MAX);
        memcpy(&dataToWrite[604], save_config_var.bytes, 4);        

        //TT-557
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT557_STOP_MIN);
        memcpy(&dataToWrite[608], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT557_IDLE_MIN);
        memcpy(&dataToWrite[612], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT557_RUN_MIN);
        memcpy(&dataToWrite[616], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT557_RUN_MAX);
        memcpy(&dataToWrite[620], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT557_IDLE_MAX);
        memcpy(&dataToWrite[624], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_TT557_STOP_MAX);
        memcpy(&dataToWrite[628], save_config_var.bytes, 4);        

        // Flowcount for intensifiers-auto mode
        save_config_var.number = get_config_parameter(CONFIG_PARAM_S1A_FLOW_COUNT);
        memcpy(&dataToWrite[632], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_S1B_FLOW_COUNT);
        memcpy(&dataToWrite[636], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_S2A_FLOW_COUNT);
        memcpy(&dataToWrite[640], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_S2B_FLOW_COUNT);
        memcpy(&dataToWrite[644], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_S3A_FLOW_COUNT);
        memcpy(&dataToWrite[648], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_S3B_FLOW_COUNT);
        memcpy(&dataToWrite[652], save_config_var.bytes, 4);

        save_config_var.number = get_config_parameter(CONFIG_PARAM_S1_MIN_CPM);
        memcpy(&dataToWrite[668], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_S2_MIN_CPM);
        memcpy(&dataToWrite[672], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_S3_MIN_CPM);
        memcpy(&dataToWrite[676], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_S1_MAX_CPM);
        memcpy(&dataToWrite[680], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_S2_MAX_CPM);
        memcpy(&dataToWrite[684], save_config_var.bytes, 4);
        save_config_var.number = get_config_parameter(CONFIG_PARAM_S3_MAX_CPM);
        memcpy(&dataToWrite[688], save_config_var.bytes, 4);

  if (flash.eraseSector(currentAddress)) {
    Serial.println("Sector erased.");

    for (int i = 0; i < totalBytesToWrite; i += PAGE_SIZE) {
      int bytesToWrite = min(PAGE_SIZE, totalBytesToWrite - i);

      if (flash.writeBuffer(currentAddress, dataToWrite + i, bytesToWrite)) {
        Serial.println("Data written.");

        uint8_t dataRead[PAGE_SIZE * 4];
        if (flash.readBuffer(currentAddress, dataRead, bytesToWrite)) {
          Serial.print("Validated read Data: ");
          for (int j = 0; j < bytesToWrite; j++) {
            Serial.print(dataRead[j], HEX);
            Serial.print(" ");
          }
          Serial.println();
        } else {
          Serial.println("Failed to read data.");
        }
      } else {
        Serial.println("Failed to write data.");
      }

      currentAddress += bytesToWrite;
    }
  } else {
    Serial.println("Failed to erase sector.");
  }
}
