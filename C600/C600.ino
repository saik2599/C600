
#include <Arduino.h>
#include "io_c200.h"
#include "config_c200.h"
#include "C200.h"             // Hardware base header file
#include "watchdog_c200.h"
#include "modbus_c200.h"
#include "wifi_c200.h"
#include "ble_c200.h"
#include <ModbusRTU.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

ModbusRTU local485;
ModbusRTU o_local485;
bool isStroking1 = false;
bool isStroking2 = false;
bool isStroking3 = false;
// Stroke-A, Stroke-B, Vol max A, Vol max B, fm pulse count, CH Analog out, Sx CPM, stroke_start timer milliseconds
bool c600_stroking_current(bool, bool, int, int, int, int, int, unsigned long int *);

unsigned long int stroke_start1 = 0; 
unsigned long int stroke_start2 = 0; 
unsigned long int stroke_start3 = 0; 

union floatToBytes {
  char asBytes[4] = { 0 };
  float asFloat;
};
void setup() {
 Serial1.begin(9600);
 Serial2.begin(19200, SERIAL_8E1);     // Local RS-485
 Serial3.begin(9600, SERIAL_8N1);      // ????
 Serial4.begin(9600, SERIAL_8N1);      // GLOBAL RS-485

  #if defined(DEBUG_MODE)
    Serial.println("System start.");
  #endif

  local485.begin(&Serial2, RE);
  local485.setBaudrate(19200);        // https://www.rheonik.com/fileadmin/Documents/PDF/Installation/RHE20_Desktop_Reference.pdf
  local485.client();

  o_local485.begin(&Serial2, RE);
  o_local485.setBaudrate(19200);      // https://www.rheonik.com/fileadmin/Documents/PDF/Installation/RHE20_Desktop_Reference.pdf
  o_local485.client();
  io_setup();
  init_config();        // Initialize the operating thresholds and parameters of the C200
  watchdog_setup();  
  #if defined(HARDWARE_GIGA)
    digital_io_loop();  // We need to do at least one loop in order to gather the switch states
    if (!test_config_parameter(SWITCH_STATE_ALL, SWITCH_STATE_BLE_NAME)){
      watchdog_disable();
      ble_init();
      watchdog_enable();
    }
    if (!test_config_parameter(SWITCH_STATE_ALL, SWITCH_STATE_WIFI_DISABLE)){  // Switch inputs are NOT working!!
      wifi_init();   
    }
  #endif
  //  set_config_bit(CONFIG_PARAM_RELAYS, TRUE, RELAY_BIT_24_RS485); // Turn ON RS485 Global Relay as soon as C300 board is ON
}

void loop() {
  if (!test_config_parameter(SWITCH_STATE_ALL, SWITCH_STATE_WIFI_DISABLE)){
      wifi_loop();      // Process any WiFi related data
    }
    if (!test_config_parameter(SWITCH_STATE_ALL, SWITCH_STATE_BLE_NAME)){
      ble_loop();
    }
    loop_two();
}

unsigned long int auto_state_timer = 0; 
unsigned long int manual_state_timer = 0; 
void loop_two() {
  digital_io_loop();
  analog_io_loop(); 
  // // If we are in automatic mode....
  // if(test_config_parameter(CONFIG_PARAM_OP_STATE, CONFIG_PARAM_MANUAL_BIT)){
  //   manual_state_timer=0;
  //   // And there are no errors.....
  //   // Test the coolant flow switch
  //   if (get_config_parameter(ERROR_PARAM_ALL) || get_config_parameter(ERROR_PARAM2_ALL)){
  //     auto_state_timer = 0;
  //     c600_stroking_current(!isStroking1, !isStroking1, CONFIG_PARAM_S1A_FLOW_COUNT, CONFIG_PARAM_S1B_FLOW_COUNT, CONFIG_PARAM_PULSE1, CONFIG_PARAM_PV1_CL, CONFIG_PARAM_S1_CPM, &stroke_start1);
  //     c600_stroking_current(!isStroking2, !isStroking2, CONFIG_PARAM_S2A_FLOW_COUNT, CONFIG_PARAM_S2B_FLOW_COUNT, CONFIG_PARAM_PULSE2, CONFIG_PARAM_PV2_CL, CONFIG_PARAM_S2_CPM, &stroke_start2);
  //     c600_stroking_current(!isStroking3, !isStroking3, CONFIG_PARAM_S3A_FLOW_COUNT, CONFIG_PARAM_S3B_FLOW_COUNT, CONFIG_PARAM_PULSE3, CONFIG_PARAM_PV3_CL, CONFIG_PARAM_S3_CPM, &stroke_start3);
  //     set_config_parameter(CONFIG_PARAM_PV1_CL, 50);    // Set all PCVs to OFF
  //     set_config_parameter(CONFIG_PARAM_PV2_CL, 50);    
  //     set_config_parameter(CONFIG_PARAM_PV3_CL, 50);    
  //     set_config_bit(CONFIG_PARAM_RELAYS, FALSE, RELAY_BIT_24_6);   // Close Suction XV if any errors
  //     set_config_bit(CONFIG_PARAM_RELAYS, FALSE, RELAY_BIT_24_15);   // Stop the coolant pumps
  //     set_config_bit(CONFIG_PARAM_RELAYS, FALSE, RELAY_BIT_24_13);   // Stop the hydraulic pumps
  //     set_config_bit(CONFIG_PARAM_OP_STATE, FALSE, CONFIG_PARAM_MANUAL_BIT); // Return to manual mode
  //   }else if (auto_state_timer == 0){
  //     auto_state_timer = millis();
  //   }else if ((millis() - auto_state_timer) < 2000){   // Start the cooling pumps
  //     set_config_bit(CONFIG_PARAM_RELAYS, TRUE, RELAY_BIT_24_15);            // start the coolant pumps
  //   }else if ((millis() - auto_state_timer) < 4000){   // Start the hydraulic pumps
  //     set_config_bit(CONFIG_PARAM_RELAYS, TRUE, RELAY_BIT_24_13);            // start the hydraulic pumps
  //   }else if ((millis() - auto_state_timer) < 6000){   // Open the suction XV
  //     set_config_bit(CONFIG_PARAM_RELAYS, TRUE, RELAY_BIT_24_6);            //Open Suction XV 
  //   }else{
  //       if (get_config_parameter(IDLE_STATE_S1_ALL) == 0){   // Then stroke away!
  //         if (!c600_stroking_current(isStroking1, !isStroking1, CONFIG_PARAM_S1A_FLOW_COUNT, CONFIG_PARAM_S1B_FLOW_COUNT, 
  //                             CONFIG_PARAM_PULSE1, CONFIG_PARAM_PV1_CL, CONFIG_PARAM_S1_CPM, &stroke_start1)){
  //           isStroking1=!isStroking1;
  //         }else if((millis() - stroke_start1) > 10000){   // 10s timeout
  //           stroke_start1 = millis();
  //           isStroking1=!isStroking1; 
  //           set_config_parameter(CONFIG_PARAM_PULSE1, 0);
  //         }
  //       }else{  // end if there are no IDLE bits active, otherwise....
  //         c600_stroking_current(!isStroking1, !isStroking1, CONFIG_PARAM_S1A_FLOW_COUNT, CONFIG_PARAM_S1B_FLOW_COUNT, 
  //                             CONFIG_PARAM_PULSE1, CONFIG_PARAM_PV1_CL, CONFIG_PARAM_S1_CPM, &stroke_start1);
  //       } 

  //       if (get_config_parameter(IDLE_STATE_S2_ALL) == 0){   // Then stroke away!
  //         if (!c600_stroking_current(isStroking2, !isStroking2, CONFIG_PARAM_S2A_FLOW_COUNT, CONFIG_PARAM_S2B_FLOW_COUNT, 
  //                             CONFIG_PARAM_PULSE2, CONFIG_PARAM_PV2_CL, CONFIG_PARAM_S2_CPM, &stroke_start2)){
  //           isStroking2=!isStroking2;
  //         }else if((millis() - stroke_start2) > 10000){   // 10s timeout
  //           stroke_start2 = millis();
  //           isStroking2=!isStroking2; 
  //           set_config_parameter(CONFIG_PARAM_PULSE2, 0);
  //         }
  //       }else{  // end if there are no IDLE bits active, otherwise....
  //         c600_stroking_current(!isStroking2, !isStroking2, CONFIG_PARAM_S2A_FLOW_COUNT, CONFIG_PARAM_S2B_FLOW_COUNT, 
  //                             CONFIG_PARAM_PULSE2, CONFIG_PARAM_PV2_CL, CONFIG_PARAM_S2_CPM, &stroke_start2);
  //       } 

  //       if (get_config_parameter(IDLE_STATE_S3_ALL) == 0){   // Then stroke away!
  //         if (!c600_stroking_current(isStroking3, !isStroking3, CONFIG_PARAM_S3A_FLOW_COUNT, CONFIG_PARAM_S3B_FLOW_COUNT, 
  //                               CONFIG_PARAM_PULSE3, CONFIG_PARAM_PV3_CL, CONFIG_PARAM_S3_CPM, &stroke_start3)){
  //           isStroking3=!isStroking3;
  //         }else if((millis() - stroke_start3) > 10000){   // 10s timeout
  //           stroke_start3 = millis();
  //           isStroking3=!isStroking3; 
  //           set_config_parameter(CONFIG_PARAM_PULSE3, 0);
  //         }
  //       }else{  // end if there are no IDLE bits active, otherwise....
  //         c600_stroking_current(!isStroking3, !isStroking3, CONFIG_PARAM_S3A_FLOW_COUNT, CONFIG_PARAM_S3B_FLOW_COUNT, 
  //                             CONFIG_PARAM_PULSE3, CONFIG_PARAM_PV3_CL, CONFIG_PARAM_S3_CPM, &stroke_start3);
  //       } 
  //   }
  // }else{
  //   if(auto_state_timer){
  //     //if previously in automode then,
  //     auto_state_timer = 0; // Set Auto-mode timer to zero and turn-off Automode
  //     if(manual_state_timer==0){
  //     manual_state_timer=millis();
  //     set_config_parameter(CONFIG_PARAM_PV1_CL, 50);    // reset all PCVs to OFF
  //     set_config_parameter(CONFIG_PARAM_PV2_CL, 50);    
  //     set_config_parameter(CONFIG_PARAM_PV3_CL, 50);    
  //     }
  //   }else if ((millis() - manual_state_timer) < 2000){   // Open the suction XV
  //     set_config_bit(CONFIG_PARAM_RELAYS, FALSE, RELAY_BIT_24_6);            //Open Suction XV 
  //   }else if ((millis() - manual_state_timer) < 4000){   // Start the cooling pumps
  //     set_config_bit(CONFIG_PARAM_RELAYS, FALSE, RELAY_BIT_24_15);            // start the coolant pumps
  //   }else if ((millis() - manual_state_timer) < 6000){   // Start the hydraulic pumps
  //     set_config_bit(CONFIG_PARAM_RELAYS, FALSE, RELAY_BIT_24_13);            // start the hydraulic pumps
  //   }else{
  //   auto_state_timer = 0; // Hold the timer for when we return to auto-mode
  //   }
  // }

  // ========================= your Auto/Manual block with updated calls =========================

// If we are in automatic mode....
if (test_config_parameter(CONFIG_PARAM_OP_STATE, CONFIG_PARAM_MANUAL_BIT)) {
  manual_state_timer = 0;

  // And there are no errors.....
  // Test the coolant flow switch
  if (get_config_parameter(ERROR_PARAM_ALL) || get_config_parameter(ERROR_PARAM2_ALL)) {
    auto_state_timer = 0;

    // === Park all three stages (updated args: min/max/actual CPM + two temps) ===
    c600_stroking_current(!isStroking1, !isStroking1, CONFIG_PARAM_S1A_FLOW_COUNT, CONFIG_PARAM_S1B_FLOW_COUNT,CONFIG_PARAM_PULSE1, CONFIG_PARAM_PV1_CL,CONFIG_PARAM_S1_MIN_CPM, CONFIG_PARAM_S1_MAX_CPM, CONFIG_PARAM_S1_ACTUAL_CPM, CONFIG_PARAM_TC5, CONFIG_PARAM_TC6,&stroke_start1);

    c600_stroking_current(!isStroking2, !isStroking2, CONFIG_PARAM_S2A_FLOW_COUNT, CONFIG_PARAM_S2B_FLOW_COUNT, CONFIG_PARAM_PULSE2, CONFIG_PARAM_PV2_CL, CONFIG_PARAM_S2_MIN_CPM,CONFIG_PARAM_S2_MAX_CPM, CONFIG_PARAM_S2_ACTUAL_CPM, CONFIG_PARAM_TC9, CONFIG_PARAM_TC10,&stroke_start2);

    c600_stroking_current(!isStroking3, !isStroking3, CONFIG_PARAM_S3A_FLOW_COUNT, CONFIG_PARAM_S3B_FLOW_COUNT, CONFIG_PARAM_PULSE3, CONFIG_PARAM_PV3_CL, CONFIG_PARAM_S3_MIN_CPM, CONFIG_PARAM_S3_MAX_CPM, CONFIG_PARAM_S3_ACTUAL_CPM, CONFIG_PARAM_TC13, CONFIG_PARAM_TC14,&stroke_start3);

    set_config_parameter(CONFIG_PARAM_PV1_CL, 50);    // Set all PCVs to OFF
    set_config_parameter(CONFIG_PARAM_PV2_CL, 50);
    set_config_parameter(CONFIG_PARAM_PV3_CL, 50);
    set_config_bit(CONFIG_PARAM_RELAYS, FALSE, RELAY_BIT_24_6);   // Close Suction XV if any errors
    set_config_bit(CONFIG_PARAM_RELAYS, FALSE, RELAY_BIT_24_15);  // Stop the coolant pumps
    set_config_bit(CONFIG_PARAM_RELAYS, FALSE, RELAY_BIT_24_13);  // Stop the hydraulic pumps
    set_config_bit(CONFIG_PARAM_OP_STATE, FALSE, CONFIG_PARAM_MANUAL_BIT); // Return to manual mode

  } else if (auto_state_timer == 0) {
    auto_state_timer = millis();

  } else if ((millis() - auto_state_timer) < 2000) {   // Start the cooling pumps
    set_config_bit(CONFIG_PARAM_RELAYS, TRUE, RELAY_BIT_24_15);

  } else if ((millis() - auto_state_timer) < 4000) {   // Start the hydraulic pumps
    set_config_bit(CONFIG_PARAM_RELAYS, TRUE, RELAY_BIT_24_13);

  } else if ((millis() - auto_state_timer) < 6000) {   // Open the suction XV
    set_config_bit(CONFIG_PARAM_RELAYS, TRUE, RELAY_BIT_24_6);

  } else {
    // ========================= RUN: Stage 1 =========================
    if (get_config_parameter(IDLE_STATE_S1_ALL) == 0) {   // Then stroke away!
      if (!c600_stroking_current(isStroking1, !isStroking1, CONFIG_PARAM_S1A_FLOW_COUNT, CONFIG_PARAM_S1B_FLOW_COUNT, CONFIG_PARAM_PULSE1, CONFIG_PARAM_PV1_CL,
           CONFIG_PARAM_S1_MIN_CPM, CONFIG_PARAM_S1_MAX_CPM, CONFIG_PARAM_S1_ACTUAL_CPM, CONFIG_PARAM_TC5, CONFIG_PARAM_TC6,&stroke_start1)) {
        isStroking1 = !isStroking1;
      } else if ((millis() - stroke_start1) > 10000) {   // 10s timeout
        stroke_start1 = millis();
        isStroking1 = !isStroking1;
        set_config_parameter(CONFIG_PARAM_PULSE1, 0);
      }
    } else {  // otherwise park
      c600_stroking_current(!isStroking1, !isStroking1, CONFIG_PARAM_S1A_FLOW_COUNT, CONFIG_PARAM_S1B_FLOW_COUNT, CONFIG_PARAM_PULSE1, CONFIG_PARAM_PV1_CL, CONFIG_PARAM_S1_MIN_CPM, CONFIG_PARAM_S1_MAX_CPM, CONFIG_PARAM_S1_ACTUAL_CPM, CONFIG_PARAM_TC5, CONFIG_PARAM_TC6,&stroke_start1);
    }

    // ========================= RUN: Stage 2 =========================
    if (get_config_parameter(IDLE_STATE_S2_ALL) == 0) {   // Then stroke away!
      if (!c600_stroking_current(isStroking2, !isStroking2, CONFIG_PARAM_S2A_FLOW_COUNT, CONFIG_PARAM_S2B_FLOW_COUNT, CONFIG_PARAM_PULSE2, CONFIG_PARAM_PV2_CL,
            CONFIG_PARAM_S2_MIN_CPM, CONFIG_PARAM_S2_MAX_CPM, CONFIG_PARAM_S2_ACTUAL_CPM, CONFIG_PARAM_TC9, CONFIG_PARAM_TC10,&stroke_start2)) {
        isStroking2 = !isStroking2;
      } else if ((millis() - stroke_start2) > 10000) {   // 10s timeout
        stroke_start2 = millis();
        isStroking2 = !isStroking2;
        set_config_parameter(CONFIG_PARAM_PULSE2, 0);
      }
    } else {  // otherwise park
      c600_stroking_current(!isStroking2, !isStroking2, CONFIG_PARAM_S2A_FLOW_COUNT, CONFIG_PARAM_S2B_FLOW_COUNT, CONFIG_PARAM_PULSE2, CONFIG_PARAM_PV2_CL,
          CONFIG_PARAM_S2_MIN_CPM, CONFIG_PARAM_S2_MAX_CPM, CONFIG_PARAM_S2_ACTUAL_CPM, CONFIG_PARAM_TC9, CONFIG_PARAM_TC10,&stroke_start2);
    }

    // ========================= RUN: Stage 3 =========================
    if (get_config_parameter(IDLE_STATE_S3_ALL) == 0) {   // Then stroke away!
      if (!c600_stroking_current(isStroking3, !isStroking3, CONFIG_PARAM_S3A_FLOW_COUNT, CONFIG_PARAM_S3B_FLOW_COUNT, CONFIG_PARAM_PULSE3, CONFIG_PARAM_PV3_CL,
           CONFIG_PARAM_S3_MIN_CPM, CONFIG_PARAM_S3_MAX_CPM, CONFIG_PARAM_S3_ACTUAL_CPM, CONFIG_PARAM_TC13, CONFIG_PARAM_TC14, &stroke_start3)) {
        isStroking3 = !isStroking3;
      } else if ((millis() - stroke_start3) > 10000) {   // 10s timeout
        stroke_start3 = millis();
        isStroking3 = !isStroking3;
        set_config_parameter(CONFIG_PARAM_PULSE3, 0);
      }
    } else {  // otherwise park
      c600_stroking_current(!isStroking3, !isStroking3, CONFIG_PARAM_S3A_FLOW_COUNT, CONFIG_PARAM_S3B_FLOW_COUNT, CONFIG_PARAM_PULSE3, CONFIG_PARAM_PV3_CL,
          CONFIG_PARAM_S3_MIN_CPM, CONFIG_PARAM_S3_MAX_CPM, CONFIG_PARAM_S3_ACTUAL_CPM, CONFIG_PARAM_TC13, CONFIG_PARAM_TC14,&stroke_start3);
    }
  }

 } else {
  // ==== Manual branch unchanged ====
  if (auto_state_timer) {
    auto_state_timer = 0;
    if (manual_state_timer == 0) {
      manual_state_timer = millis();
      set_config_parameter(CONFIG_PARAM_PV1_CL, 50);
      set_config_parameter(CONFIG_PARAM_PV2_CL, 50);
      set_config_parameter(CONFIG_PARAM_PV3_CL, 50);
    }
  } else if ((millis() - manual_state_timer) < 2000) {
    set_config_bit(CONFIG_PARAM_RELAYS, FALSE, RELAY_BIT_24_6);
  } else if ((millis() - manual_state_timer) < 4000) {
    set_config_bit(CONFIG_PARAM_RELAYS, FALSE, RELAY_BIT_24_15);
  } else if ((millis() - manual_state_timer) < 6000) {
    set_config_bit(CONFIG_PARAM_RELAYS, FALSE, RELAY_BIT_24_13);
  } else {
    auto_state_timer = 0;
  }
}
  parameter_check();
 // ***** Receive and process the global RS_485 data *****************
 // Start by reading all available bytes in the buffer and storing them for processing
  while (Serial4.available()){
    char thisChar = Serial4.read();
    modbuxRxData(thisChar, RS_485_BUFFER);
  }

  // Check the RS-485 Serial data MODBUS buffer and send any required responses to the Serial port
  if (modbus_loop(RS_485_BUFFER)){
    int tx_bytes = get_tx_bytes(RS_485_BUFFER);
    Serial4.write(get_tx_buffer(RS_485_BUFFER), tx_bytes);
  }  
}

// void parameter_check(){
//   //Stage control for PT405- Suction PT
//     if ((get_config_parameter(CONFIG_PARAM_SUCTION_PT) < get_config_parameter(CONFIG_PARAM_PT405_STOP_MIN)) 
//               || (get_config_parameter(CONFIG_PARAM_SUCTION_PT) > get_config_parameter(CONFIG_PARAM_PT405_STOP_MAX))){          
//       // This error will remain here until the red button is presssed by the operator to clear it
//       set_config_bit(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT405);
//     }else if (get_config_parameter(CONFIG_PARAM_SUCTION_PT) < get_config_parameter(CONFIG_PARAM_PT405_IDLE_MIN)){
//       set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT405);
//     }else if (get_config_parameter(CONFIG_PARAM_SUCTION_PT) > get_config_parameter(CONFIG_PARAM_PT405_IDLE_MAX)){
//       set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT405);
//     }else if ((get_config_parameter(CONFIG_PARAM_SUCTION_PT) < get_config_parameter(CONFIG_PARAM_PT405_RUN_MAX)) 
//               && (get_config_parameter(CONFIG_PARAM_SUCTION_PT) > get_config_parameter(CONFIG_PARAM_PT405_RUN_MIN))){
//         set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT405);
//     }
    
//     //Stage control for PT426- Suction TANK PT
//     if ((get_config_parameter(CONFIG_PARAM_SUCTION_TANK) < get_config_parameter(CONFIG_PARAM_PT426_STOP_MIN)) 
//               || (get_config_parameter(CONFIG_PARAM_SUCTION_TANK) > get_config_parameter(CONFIG_PARAM_PT426_STOP_MAX))){          
//       // This error will remain here until the red button is presssed by the operator to clear it
//       set_config_bit(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT426);
//     }else if (get_config_parameter(CONFIG_PARAM_SUCTION_TANK) < get_config_parameter(CONFIG_PARAM_PT426_IDLE_MIN)){
//       set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT426);
//     }else if (get_config_parameter(CONFIG_PARAM_SUCTION_TANK) > get_config_parameter(CONFIG_PARAM_PT426_IDLE_MAX)){
//       set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT426);
//     }else if ((get_config_parameter(CONFIG_PARAM_SUCTION_TANK) < get_config_parameter(CONFIG_PARAM_PT426_RUN_MAX)) 
//               && (get_config_parameter(CONFIG_PARAM_SUCTION_TANK) > get_config_parameter(CONFIG_PARAM_PT426_RUN_MIN))){
//         set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT426);
//     }

//     // //Stage control for PT442- Stage1 Tank PT
//     // if ((get_config_parameter(CONFIG_PARAM_STAGE1_TANK) < get_config_parameter(CONFIG_PARAM_PT442_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_STAGE1_TANK) > get_config_parameter(CONFIG_PARAM_PT442_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT442);
//     // }else if (get_config_parameter(CONFIG_PARAM_STAGE1_TANK) < get_config_parameter(CONFIG_PARAM_PT442_IDLE_MIN)){
//     //   // When S1 Tank goes low start compressing gas from S1 into the interstage tank (No longer IDLE)
//     //   set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT442);
//     //   // And IDLE S2 and stop taking gas from the interstage tank
//     //   set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT442);
//     //   Serial.println(test_config_parameter(IDLE_STATE_S2_ALL,IDLE_STATE_PT442));
//     // }else if (get_config_parameter(CONFIG_PARAM_STAGE1_TANK) > get_config_parameter(CONFIG_PARAM_PT442_IDLE_MAX)){
//     //   // When S1 Tank goes high - IDLE S1 and stop compressing gas into the interstage tank
//     //   set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT442);
//     //   // And start taking gas from the interstage tank into S2 (No longer IDLE)
//     //   set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT442);
//     // }else if ((get_config_parameter(CONFIG_PARAM_STAGE1_TANK) < get_config_parameter(CONFIG_PARAM_PT442_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_STAGE1_TANK) > get_config_parameter(CONFIG_PARAM_PT442_RUN_MIN))){
//     //     // When S1 tank interstage pressure is in range run both S1 and S2
//     //     set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT442);
//     //     set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT442);
//     // }

//     // UNUSED PT -410
//     //   //Stage control for PT410- Stage1 tank PT
//     // if ((get_config_parameter(CONFIG_PARAM_STAGE1_TANK) < get_config_parameter(CONFIG_PARAM_PT410_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_STAGE1_TANK) > get_config_parameter(CONFIG_PARAM_PT410_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT410);
//     // }else if (get_config_parameter(CONFIG_PARAM_STAGE1_TANK) < get_config_parameter(CONFIG_PARAM_PT410_IDLE_MIN)){
//     //   set_config_parameter(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT410);
//     // }else if (get_config_parameter(CONFIG_PARAM_STAGE1_TANK) > get_config_parameter(CONFIG_PARAM_PT410_IDLE_MAX)){
//     //   set_config_parameter(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT410);
//     // }else if ((get_config_parameter(CONFIG_PARAM_STAGE1_TANK) < get_config_parameter(CONFIG_PARAM_PT410_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_STAGE1_TANK) > get_config_parameter(CONFIG_PARAM_PT410_RUN_MIN))){
//     //     set_config_parameter(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT410);
//     // }

//     // //Stage control for PT457- Stage2 Tank PT
//     // if ((get_config_parameter(CONFIG_PARAM_STAGE2_TANK) < get_config_parameter(CONFIG_PARAM_PT457_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_STAGE2_TANK) > get_config_parameter(CONFIG_PARAM_PT457_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT457);
//     // }else if (get_config_parameter(CONFIG_PARAM_STAGE2_TANK) < get_config_parameter(CONFIG_PARAM_PT457_IDLE_MIN)){
//     //   // When S2 tank is low, start compressing gas from S2 into the tank (No longer IDLE)
//     //   set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT457);
//     //   // And stop taking gas from the tank into S3
//     //   set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT457);
//     // }else if (get_config_parameter(CONFIG_PARAM_STAGE2_TANK) > get_config_parameter(CONFIG_PARAM_PT457_IDLE_MAX)){
//     //   // When S2 tank is high stop compressing gas from S2 into the tank
//     //   set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT457);
//     //   // And start taking gas into S3 to take the pressure down (No longer IDLE)
//     //   set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT457);
//     // }else if ((get_config_parameter(CONFIG_PARAM_STAGE2_TANK) < get_config_parameter(CONFIG_PARAM_PT457_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_STAGE2_TANK) > get_config_parameter(CONFIG_PARAM_PT457_RUN_MIN))){
//     //     set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT457);
//     //     set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT457);
//     // }

//     // UNUSED PT -416
//     //   //Stage control for PT416- Stage2 Tank PT
//     // if ((get_config_parameter(CONFIG_PARAM_STAGE2_TANK) < get_config_parameter(CONFIG_PARAM_PT416_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_STAGE2_TANK) > get_config_parameter(CONFIG_PARAM_PT416_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT416);
//     // }else if (get_config_parameter(CONFIG_PARAM_STAGE2_TANK) < get_config_parameter(CONFIG_PARAM_PT416_IDLE_MIN)){
//     //   set_config_parameter(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT416);
//     // }else if (get_config_parameter(CONFIG_PARAM_STAGE2_TANK) > get_config_parameter(CONFIG_PARAM_PT416_IDLE_MAX)){
//     //   set_config_parameter(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT416);
//     // }else if ((get_config_parameter(CONFIG_PARAM_STAGE2_TANK) < get_config_parameter(CONFIG_PARAM_PT416_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_STAGE2_TANK) > get_config_parameter(CONFIG_PARAM_PT416_RUN_MIN))){
//     //     set_config_parameter(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT416);
//     // }

//     // //Stage control for PT468- Stage3 Tank PT
//     // if ((get_config_parameter(CONFIG_PARAM_STAGE3_TANK) < get_config_parameter(CONFIG_PARAM_PT468_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_STAGE3_TANK) > get_config_parameter(CONFIG_PARAM_PT468_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT468);
//     // }else if (get_config_parameter(CONFIG_PARAM_STAGE3_TANK) < get_config_parameter(CONFIG_PARAM_PT468_IDLE_MIN)){
//     //   // When S3 Tank is low, run S3 (No longer IDLEO)
//     //   set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT468);
//     // }else if (get_config_parameter(CONFIG_PARAM_STAGE3_TANK) > get_config_parameter(CONFIG_PARAM_PT468_IDLE_MAX)){
//     //   // When S3 Tank is high IDLE S3
//     //   set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT468);
//     // }else if ((get_config_parameter(CONFIG_PARAM_STAGE3_TANK) < get_config_parameter(CONFIG_PARAM_PT468_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_STAGE3_TANK) > get_config_parameter(CONFIG_PARAM_PT468_RUN_MIN))){
//     //     set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT468);
//     // }

//     //  //Stage control for PT487- Stage3 PT
//     // if ((get_config_parameter(CONFIG_PARAM_STAGE3_PT) < get_config_parameter(CONFIG_PARAM_PT487_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_STAGE3_PT) > get_config_parameter(CONFIG_PARAM_PT487_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT487);
//     // }else if (get_config_parameter(CONFIG_PARAM_STAGE3_PT) < get_config_parameter(CONFIG_PARAM_PT487_IDLE_MIN)){
//     //   // When S3 discharge is low no longer IDLE S3
//     //   set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT487);
//     // }else if (get_config_parameter(CONFIG_PARAM_STAGE3_PT) > get_config_parameter(CONFIG_PARAM_PT487_IDLE_MAX)){
//     //   // When S3 discharge is too high IDLE S3
//     //   set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT487);
//     // }else if ((get_config_parameter(CONFIG_PARAM_STAGE3_PT) < get_config_parameter(CONFIG_PARAM_PT487_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_STAGE3_PT) > get_config_parameter(CONFIG_PARAM_PT487_RUN_MIN))){
//     //     // When S3 discharge is in range no longer IDLE
//     //     set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT487);
//     // }

//     //Stage control for PT496- HFL Pump PT
//     if ((get_config_parameter(CONFIG_PARAM_HFL_PUMP1) < get_config_parameter(CONFIG_PARAM_PT496_STOP_MIN)) 
//               || (get_config_parameter(CONFIG_PARAM_HFL_PUMP1) > get_config_parameter(CONFIG_PARAM_PT496_STOP_MAX))){          
//       // This error will remain here until the red button is presssed by the operator to clear it
//       set_config_bit(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT496);
//     }else if (get_config_parameter(CONFIG_PARAM_HFL_PUMP1) < get_config_parameter(CONFIG_PARAM_PT496_IDLE_MIN)){
//       // When the HFL PT is out of range - IDLE all 3 stages 
//       set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT496);
//       set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT496);
//       set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT496);
//     }else if (get_config_parameter(CONFIG_PARAM_HFL_PUMP1) > get_config_parameter(CONFIG_PARAM_PT496_IDLE_MAX)){
//       // When the HFL PT is out of range - IDLE all 3 stages 
//       set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT496);
//       set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT496);
//       set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT496);
//     }else if ((get_config_parameter(CONFIG_PARAM_HFL_PUMP1) < get_config_parameter(CONFIG_PARAM_PT496_RUN_MAX)) 
//               && (get_config_parameter(CONFIG_PARAM_HFL_PUMP1) > get_config_parameter(CONFIG_PARAM_PT496_RUN_MIN))){
//       // When the HFL PT is in range all intensifiers can run (No longer IDLE)
//       set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT496);
//       set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT496);
//       set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT496);
//     }


//     //Stage control for PT531- Coolant1 PT
//     if ((get_config_parameter(CONFIG_PARAM_COOLANT1_PT) < get_config_parameter(CONFIG_PARAM_PT531_STOP_MIN)) 
//               || (get_config_parameter(CONFIG_PARAM_COOLANT1_PT) > get_config_parameter(CONFIG_PARAM_PT531_STOP_MAX))){          
//       // This error will remain here until the red button is presssed by the operator to clear it
//       set_config_bit(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT531);
//     }else if (get_config_parameter(CONFIG_PARAM_COOLANT1_PT) < get_config_parameter(CONFIG_PARAM_PT531_IDLE_MIN)){
//       // When the coolant PT is out of range, IDLE all 3 intensifiers 
//       set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT531);
//       set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT531);
//       set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT531);
//     }else if (get_config_parameter(CONFIG_PARAM_COOLANT1_PT) > get_config_parameter(CONFIG_PARAM_PT531_IDLE_MAX)){
//       // When the coolant PT is out of range, IDLE all 3 intensifiers 
//       set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT531);
//       set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT531);
//       set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT531);
//     }else if ((get_config_parameter(CONFIG_PARAM_COOLANT1_PT) < get_config_parameter(CONFIG_PARAM_PT531_RUN_MAX)) 
//               && (get_config_parameter(CONFIG_PARAM_COOLANT1_PT) > get_config_parameter(CONFIG_PARAM_PT531_RUN_MIN))){
//       // When the coolant PT is out of range, IDLE all 3 intensifiers 
//       set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT531);
//       set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT531);
//       set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT531);
//     }

//     //Stage control for PT576- Coolant2 PT
//     if ((get_config_parameter(CONFIG_PARAM_COOLANT2_PT) < get_config_parameter(CONFIG_PARAM_PT576_STOP_MIN)) 
//               || (get_config_parameter(CONFIG_PARAM_COOLANT2_PT) > get_config_parameter(CONFIG_PARAM_PT576_STOP_MAX))){          
//       // This error will remain here until the red button is presssed by the operator to clear it
//       set_config_bit(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT576);
//     }else if (get_config_parameter(CONFIG_PARAM_COOLANT2_PT) < get_config_parameter(CONFIG_PARAM_PT576_IDLE_MIN)){
//       // When the coolant PT is out of range, IDLE all 3 intensifiers 
//       set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT576);
//       set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT576);
//       set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT576);
//     }else if (get_config_parameter(CONFIG_PARAM_COOLANT2_PT) > get_config_parameter(CONFIG_PARAM_PT576_IDLE_MAX)){
//       // When the coolant PT is out of range, IDLE all 3 intensifiers 
//       set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT576);
//       set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT576);
//       set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT576);
//     }else if ((get_config_parameter(CONFIG_PARAM_COOLANT2_PT) < get_config_parameter(CONFIG_PARAM_PT576_RUN_MAX)) 
//               && (get_config_parameter(CONFIG_PARAM_COOLANT2_PT) > get_config_parameter(CONFIG_PARAM_PT576_RUN_MIN))){
//         // When the coolant PT is out of range, IDLE all 3 intensifiers 
//         set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT576);
//         set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT576);
//         set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT576);
//     }


    
//     // Fetch scaled PSI once for ratio math
//     const float pt426 = (float)get_config_parameter(CONFIG_PARAM_SUCTION_TANK);   // PT426
//     const float pt442 = (float)get_config_parameter(CONFIG_PARAM_STAGE1_TANK);    // PT442
//     const float pt457 = (float)get_config_parameter(CONFIG_PARAM_STAGE2_TANK);    // PT457
//     const float pt487 = (float)get_config_parameter(CONFIG_PARAM_STAGE3_PT);      // PT487

//    // Common ratio constants (band: 1:2.75..1:3.25, 5% hysteresis)
//    const float R_MIN  = 2.75f;
//    const float R_MAX  = 3.25f;
//    const float HYST_L = 0.05f;   // clear low-idle when back +5%
//    const float HYST_H = 0.05f;   // clear high-idle when back -5%

//    // ============================================================================
//    // 3) PT426 → PT442 (Stage 1 discharge ratio gate)  [UNCHANGED PATTERN]
//    //     - < 2.75× : Intensifier 2 IDLE
//    //     - within  : Allow Intensifier 2
//    //     - > 3.25× : Intensifier 1 IDLE
//    // ============================================================================
//     {
//      const float pt442_low  = pt426 * R_MIN;
//      const float pt442_high = pt426 * R_MAX;
//      set_config_parameter(CONFIG_PARAM_PT442_RATIO_LOW_PSI,  pt442_low);
//      set_config_parameter(CONFIG_PARAM_PT442_RATIO_HIGH_PSI, pt442_high);

//       // S2 low-side protection (idle S2 if below; clear once back above low*1.05)
//       if (pt442 < pt442_low) {
//       set_config_parameter(IDLE_STATE_S2_ALL, TRUE,  IDLE_STATE_PT442);
//        } else if (test_config_parameter(IDLE_STATE_S2_ALL, IDLE_STATE_PT442) && (pt442 >= (pt442_low * HYST_L))) {
//        set_config_parameter(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT442);
//       }

//       // S1 high-side protection (idle S1 if above; clear once back below high*0.95)
//       if (pt442 > pt442_high) {
//        set_config_parameter(IDLE_STATE_S1_ALL, TRUE,  IDLE_STATE_PT442);
//       } else if (test_config_parameter(IDLE_STATE_S1_ALL, IDLE_STATE_PT442) && (pt442 <= (pt442_high * HYST_H))) {
//        set_config_parameter(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT442);
//       }
//     }

//     // Site-specific MAX protections for PT442 (unchanged)
//     if ((get_config_parameter(CONFIG_PARAM_STAGE1_TANK) > get_config_parameter(CONFIG_PARAM_PT442_STOP_MAX))){
//      set_config_parameter(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT442);
//      Serial.println("ERROR PRESS - PT442");
//     } else if (get_config_parameter(CONFIG_PARAM_STAGE1_TANK) > get_config_parameter(CONFIG_PARAM_PT442_IDLE_MAX)){
//      set_config_parameter(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT442);
//     } else if (get_config_parameter(CONFIG_PARAM_STAGE1_TANK) < get_config_parameter(CONFIG_PARAM_PT442_RUN_MAX)){
//      set_config_parameter(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT442);
//     }



//     // ============================================================================
//     // 4) PT442 → PT457 (Interstage ratio gate; SAME LOW/HIGH PATTERN)
//     //     - < 2.75× : Intensifier 3 IDLE
//     //     - within  : Allow Intensifier 3
//     //     - > 3.25× : Intensifier 2 IDLE
//     // ============================================================================
//     {
//      const float pt457_low  = pt442 * R_MIN;
//      const float pt457_high = pt442 * R_MAX;
//      set_config_parameter(CONFIG_PARAM_PT457_RATIO_LOW_PSI,  pt457_low);
//      set_config_parameter(CONFIG_PARAM_PT457_RATIO_HIGH_PSI, pt457_high);

//       // S3 low-side protection (idle S3 if below; clear once back above low*1.05)
//       if (pt457 < pt457_low) {
//        set_config_parameter(IDLE_STATE_S3_ALL, TRUE,  IDLE_STATE_PT457);
//       } else if (test_config_parameter(IDLE_STATE_S3_ALL, IDLE_STATE_PT457) && (pt457 >= (pt457_low * HYST_L))) {
//        set_config_parameter(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT457);
//       }

//       // S2 high-side protection (idle S2 if above; clear once back below high*0.95)
//       if (pt457 > pt457_high) {
//        set_config_parameter(IDLE_STATE_S2_ALL, TRUE,  IDLE_STATE_PT457);
//       } else if (test_config_parameter(IDLE_STATE_S2_ALL, IDLE_STATE_PT457) && (pt457 <= (pt457_high * HYST_H))) {
//        set_config_parameter(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT457);
//       }
//     }

//     // Existing site-specific MAX protections for PT457 (unchanged)
//     if ((get_config_parameter(CONFIG_PARAM_STAGE2_TANK) > get_config_parameter(CONFIG_PARAM_PT457_STOP_MAX))){
//      set_config_parameter(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT457);
//      Serial.println("ERROR PRESS - PT457");
//     } else if (get_config_parameter(CONFIG_PARAM_STAGE2_TANK) > get_config_parameter(CONFIG_PARAM_PT457_IDLE_MAX)){
//      set_config_parameter(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT457);
//     } else if ((get_config_parameter(CONFIG_PARAM_STAGE2_TANK) < get_config_parameter(CONFIG_PARAM_PT457_RUN_MAX))){
//      set_config_parameter(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT457);
//     }

//     // ============================================================================
//     // 5) PT468 – HIGH LIMITS ONLY (extra ESTOP)  [unchanged]
//     // ============================================================================
//     if ((get_config_parameter(CONFIG_PARAM_STAGE3_TANK) > get_config_parameter(CONFIG_PARAM_PT468_STOP_MAX))){
//      set_config_parameter(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT468);
//      Serial.println("ERROR PRESS - PT468");
//     } else if (get_config_parameter(CONFIG_PARAM_STAGE3_TANK) > get_config_parameter(CONFIG_PARAM_PT468_IDLE_MAX)){
//      set_config_parameter(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT468);
//     } else if ((get_config_parameter(CONFIG_PARAM_STAGE3_TANK) < get_config_parameter(CONFIG_PARAM_PT468_RUN_MAX))){
//      set_config_parameter(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT468);
//     }

//    // ============================================================================
//    // 6) PT457 → PT487 (Tank ratio gate; SAME LOW/HIGH PATTERN)
//    //     - < 2.75× : Intensifier 3 IDLE (not enough tank build)
//    //     - within  : Normal (keep compressing)
//    //     - ≥ 3.25× : ENTIRE COMPRESSOR IDLE (idle all stages here)
//    // ============================================================================
//     {
//      const float pt487_low  = pt457 * R_MIN;
//      const float pt487_high = pt457 * R_MAX;
//      set_config_parameter(CONFIG_PARAM_PT487_RATIO_LOW_PSI,  pt487_low);
//      set_config_parameter(CONFIG_PARAM_PT487_RATIO_HIGH_PSI, pt487_high);

//      // Low-side: idle Stage 3; clear when back above low*1.05
//       if (pt487 < pt487_low) {
//        set_config_parameter(IDLE_STATE_S3_ALL, TRUE,  IDLE_STATE_PT487);
//       } else if (test_config_parameter(IDLE_STATE_S3_ALL, IDLE_STATE_PT487) && (pt487 >= (pt487_low * HYST_L))) {
//        set_config_parameter(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT487);
//       }

//       // High-side: compressor-wide idle. If you have a global idle bit, set it here.
//       if (pt487 >= pt487_high) {
//         set_config_parameter(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT487);
//         set_config_parameter(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT487);
//         set_config_parameter(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT487);
//       } else {
//        // Clear high-side idles once back below high*0.95
//        const bool s1_hi = test_config_parameter(IDLE_STATE_S1_ALL, IDLE_STATE_PT487);
//        const bool s2_hi = test_config_parameter(IDLE_STATE_S2_ALL, IDLE_STATE_PT487);
//        const bool s3_hi = test_config_parameter(IDLE_STATE_S3_ALL, IDLE_STATE_PT487);
//         if ((s1_hi || s2_hi || s3_hi) && (pt487 <= (pt487_high * HYST_H))) {
//          set_config_parameter(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT487);
//          set_config_parameter(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT487);
//          set_config_parameter(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT487);
//         }
//       }
//     }
   

//     // Site-specific MAX protections for PT487 remain as-is (ALWAYS ACTIVE)
//     if ((get_config_parameter(CONFIG_PARAM_STAGE3_PT) > get_config_parameter(CONFIG_PARAM_PT487_STOP_MAX))){
//      set_config_parameter(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT487);
//      Serial.println("ERROR PRESS - PT487");
//     } else if (get_config_parameter(CONFIG_PARAM_STAGE3_PT) > get_config_parameter(CONFIG_PARAM_PT487_IDLE_MAX)){
//      set_config_parameter(IDLE_STATE_S3_ALL, TRUE,  IDLE_STATE_PT487);
//     } else if ((get_config_parameter(CONFIG_PARAM_STAGE3_PT) < get_config_parameter(CONFIG_PARAM_PT487_RUN_MAX))){
//      set_config_parameter(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT487);
//     }



//     // //Stage control for TC3-TT432
//     // if ((get_config_parameter(CONFIG_PARAM_TC3) < get_config_parameter(CONFIG_PARAM_TT432_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_TC3) > get_config_parameter(CONFIG_PARAM_TT432_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT432);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC3) < get_config_parameter(CONFIG_PARAM_TT432_IDLE_MIN)){
//     //   // When S1 suction-A is low, IDLE S1
//     //   set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT432);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC3) > get_config_parameter(CONFIG_PARAM_TT432_IDLE_MAX)){
//     //   // When S1 suction-A is high, IDLE S1
//     //   set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT432);
//     // }else if ((get_config_parameter(CONFIG_PARAM_TC3) < get_config_parameter(CONFIG_PARAM_TT432_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_TC3) > get_config_parameter(CONFIG_PARAM_TT432_RUN_MIN))){
//     //     // When S1 suction-A is in range no longer need to IDLE S1                
//     //     set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_TT432);
//     // }

//     //    //Stage control for TC4-TT439
//     // if ((get_config_parameter(CONFIG_PARAM_TC4) < get_config_parameter(CONFIG_PARAM_TT439_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_TC4) > get_config_parameter(CONFIG_PARAM_TT439_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT439);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC4) < get_config_parameter(CONFIG_PARAM_TT439_IDLE_MIN)){
//     //   // When S1 suction-B is low, IDLE S1
//     //   set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT439);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC4) > get_config_parameter(CONFIG_PARAM_TT439_IDLE_MAX)){
//     //   // When S1 suction-B is low, IDLE S1
//     //   set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT439);
//     // }else if ((get_config_parameter(CONFIG_PARAM_TC4) < get_config_parameter(CONFIG_PARAM_TT439_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_TC4) > get_config_parameter(CONFIG_PARAM_TT439_RUN_MIN))){
//     //     // When S1 suction-B is in range no longer need to IDLE S1       
//     //     set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_TT439);
//     // }

//     // //Stage control for TC5-TT435
//     // if ((get_config_parameter(CONFIG_PARAM_TC5) < get_config_parameter(CONFIG_PARAM_TT435_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_TC5) > get_config_parameter(CONFIG_PARAM_TT435_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT435);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC5) < get_config_parameter(CONFIG_PARAM_TT435_IDLE_MIN)){
//     //   // When S1 discharge-A is low, IDLE S1
//     //   set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT435);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC5) > get_config_parameter(CONFIG_PARAM_TT435_IDLE_MAX)){
//     //   // When S1 discharge-A is high, IDLE S1
//     //   set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT435);
//     // }else if ((get_config_parameter(CONFIG_PARAM_TC5) < get_config_parameter(CONFIG_PARAM_TT435_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_TC5) > get_config_parameter(CONFIG_PARAM_TT435_RUN_MIN))){
//     //     // When S1 discharge-A is in range RUN (No longer need to IDLE) S1
//     //     set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_TT435);
//     // }

//     // //Stage control for TC6-TT436
//     // if ((get_config_parameter(CONFIG_PARAM_TC6) < get_config_parameter(CONFIG_PARAM_TT436_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_TC6) > get_config_parameter(CONFIG_PARAM_TT436_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT436);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC6) < get_config_parameter(CONFIG_PARAM_TT436_IDLE_MIN)){
//     //   // When S1 discharge-B is low, IDLE S1
//     //   set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT436);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC6) > get_config_parameter(CONFIG_PARAM_TT436_IDLE_MAX)){
//     //   // When S1 discharge-B is high, IDLE S1
//     //   set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT436);
//     // }else if ((get_config_parameter(CONFIG_PARAM_TC6) < get_config_parameter(CONFIG_PARAM_TT436_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_TC6) > get_config_parameter(CONFIG_PARAM_TT436_RUN_MIN))){
//     //     // When S1 discharge-B is in range RUN (No longer need to IDLE) S1
//     //     set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_TT436);
//     // }

//     // //Stage control for TC7-TT447
//     // if ((get_config_parameter(CONFIG_PARAM_TC7) < get_config_parameter(CONFIG_PARAM_TT447_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_TC7) > get_config_parameter(CONFIG_PARAM_TT447_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT447);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC7) < get_config_parameter(CONFIG_PARAM_TT447_IDLE_MIN)){
//     //   // When S2 suction-A is low, IDLE S2
//     //   set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT447);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC7) > get_config_parameter(CONFIG_PARAM_TT447_IDLE_MAX)){
//     //   // When S2 suction-A is high, IDLE S2
//     //   set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT447);
//     // }else if ((get_config_parameter(CONFIG_PARAM_TC7) < get_config_parameter(CONFIG_PARAM_TT447_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_TC7) > get_config_parameter(CONFIG_PARAM_TT447_RUN_MIN))){
//     //     // When S2 suction-A is in range RUN (No longer need to IDLE) S2
//     //     set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_TT447);
//     // }

//     // //Stage control for TC8-TT454
//     // if ((get_config_parameter(CONFIG_PARAM_TC8) < get_config_parameter(CONFIG_PARAM_TT454_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_TC8) > get_config_parameter(CONFIG_PARAM_TT454_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT454);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC8) < get_config_parameter(CONFIG_PARAM_TT454_IDLE_MIN)){
//     //   // When S2 suction-B is low, IDLE S2
//     //   set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT454);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC8) > get_config_parameter(CONFIG_PARAM_TT454_IDLE_MAX)){
//     //   // When S2 suction-B is high, IDLE S2
//     //   set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT454);
//     // }else if ((get_config_parameter(CONFIG_PARAM_TC8) < get_config_parameter(CONFIG_PARAM_TT454_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_TC8) > get_config_parameter(CONFIG_PARAM_TT454_RUN_MIN))){
//     //     // When S2 suction-B is in range RUN (No longer need to IDLE) S2
//     //     set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_TT454);
//     // }

//     // //Stage control for TC9-TT450
//     // if ((get_config_parameter(CONFIG_PARAM_TC9) < get_config_parameter(CONFIG_PARAM_TT450_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_TC9) > get_config_parameter(CONFIG_PARAM_TT450_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT450);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC9) < get_config_parameter(CONFIG_PARAM_TT450_IDLE_MIN)){
//     //   // When S2 discharge-A is low, IDLE S2
//     //   set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT450);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC9) > get_config_parameter(CONFIG_PARAM_TT450_IDLE_MAX)){
//     //   // When S2 discharge-A is high, IDLE S2
//     //   set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT450);
//     // }else if ((get_config_parameter(CONFIG_PARAM_TC9) < get_config_parameter(CONFIG_PARAM_TT450_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_TC9) > get_config_parameter(CONFIG_PARAM_TT450_RUN_MIN))){
//     //     // When S2 discharge-A is in range RUN (No longer need to IDLE) S2
//     //     set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_TT450);
//     // }

//     // //Stage control for TC10-TT451
//     // if ((get_config_parameter(CONFIG_PARAM_TC10) < get_config_parameter(CONFIG_PARAM_TT451_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_TC10) > get_config_parameter(CONFIG_PARAM_TT451_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT451);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC10) < get_config_parameter(CONFIG_PARAM_TT451_IDLE_MIN)){
//     //   // When S2 discharge-B is low, IDLE S2
//     //   set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT451);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC10) > get_config_parameter(CONFIG_PARAM_TT451_IDLE_MAX)){
//     //   // When S2 discharge-B is high, IDLE S2
//     //   set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT451);
//     // }else if ((get_config_parameter(CONFIG_PARAM_TC10) < get_config_parameter(CONFIG_PARAM_TT451_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_TC10) > get_config_parameter(CONFIG_PARAM_TT451_RUN_MIN))){
//     //     // When S2 discharge-B is in range RUN (No longer need to IDLE) S2
//     //     set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_TT451);
//     // }

//     // //Stage control for TC11-TT462
//     // if ((get_config_parameter(CONFIG_PARAM_TC11) < get_config_parameter(CONFIG_PARAM_TT462_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_TC11) > get_config_parameter(CONFIG_PARAM_TT462_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT462);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC11) < get_config_parameter(CONFIG_PARAM_TT462_IDLE_MIN)){
//     //   // When S3 suction-A is low, IDLE S3
//     //   set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT462);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC11) > get_config_parameter(CONFIG_PARAM_TT462_IDLE_MAX)){
//     //   // When S3 suction-A is high, IDLE S3
//     //   set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT462);
//     // }else if ((get_config_parameter(CONFIG_PARAM_TC11) < get_config_parameter(CONFIG_PARAM_TT462_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_TC11) > get_config_parameter(CONFIG_PARAM_TT462_RUN_MIN))){
//     //     // When S3 suction-A is in range RUN (No longer need to IDLE) S3
//     //     set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_TT462);
//     // }

//     // //Stage control for TC12-TT474
//     // if ((get_config_parameter(CONFIG_PARAM_TC12) < get_config_parameter(CONFIG_PARAM_TT474_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_TC12) > get_config_parameter(CONFIG_PARAM_TT474_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT474);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC12) < get_config_parameter(CONFIG_PARAM_TT474_IDLE_MIN)){
//     //   // When S3 suction-B is low, IDLE S3
//     //   set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT474);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC12) > get_config_parameter(CONFIG_PARAM_TT474_IDLE_MAX)){
//     //   // When S3 suction-B is high, IDLE S3
//     //   set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT474);
//     // }else if ((get_config_parameter(CONFIG_PARAM_TC12) < get_config_parameter(CONFIG_PARAM_TT474_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_TC12) > get_config_parameter(CONFIG_PARAM_TT474_RUN_MIN))){
//     //     // When S3 suction-B is in range RUN (No longer need to IDLE) S3
//     //     set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_TT474);
//     // }

//     // //Stage control for TC13-TT465
//     // if ((get_config_parameter(CONFIG_PARAM_TC13) < get_config_parameter(CONFIG_PARAM_TT465_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_TC13) > get_config_parameter(CONFIG_PARAM_TT465_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT465);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC13) < get_config_parameter(CONFIG_PARAM_TT465_IDLE_MIN)){
//     //   // When S3 discharge-A is low, IDLE S3
//     //   set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT465);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC13) > get_config_parameter(CONFIG_PARAM_TT465_IDLE_MAX)){
//     //   // When S3 discharge-A is high, IDLE S3
//     //   set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT465);
//     // }else if ((get_config_parameter(CONFIG_PARAM_TC13) < get_config_parameter(CONFIG_PARAM_TT465_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_TC13) > get_config_parameter(CONFIG_PARAM_TT465_RUN_MIN))){
//     //     // When S3 discharge-A is in range RUN (No longer need to IDLE) S3
//     //     set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_TT465);
//     // }

//     // //Stage control for TC14-TT471
//     // if ((get_config_parameter(CONFIG_PARAM_TC14) < get_config_parameter(CONFIG_PARAM_TT471_STOP_MIN)) 
//     //           || (get_config_parameter(CONFIG_PARAM_TC14) > get_config_parameter(CONFIG_PARAM_TT471_STOP_MAX))){          
//     //   // This error will remain here until the red button is presssed by the operator to clear it
//     //   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT471);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC14) < get_config_parameter(CONFIG_PARAM_TT471_IDLE_MIN)){
//     //   // When S3 discharge-B is low, IDLE S3
//     //   set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT471);
//     // }else if (get_config_parameter(CONFIG_PARAM_TC14) > get_config_parameter(CONFIG_PARAM_TT471_IDLE_MAX)){
//     //   // When S3 discharge-B is low, IDLE S3
//     //   set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT471);
//     // }else if ((get_config_parameter(CONFIG_PARAM_TC14) < get_config_parameter(CONFIG_PARAM_TT471_RUN_MAX)) 
//     //           && (get_config_parameter(CONFIG_PARAM_TC14) > get_config_parameter(CONFIG_PARAM_TT471_RUN_MIN))){
//     //     // When S3 discharge-B is in range RUN (No longer need to IDLE) S3
//     //     set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_TT471);
//     // }

//     // Stage control for TC3 - TT432 (max-only)
// if (get_config_parameter(CONFIG_PARAM_TC3) > get_config_parameter(CONFIG_PARAM_TT432_STOP_MAX)) {
//   // This error will remain here until the red button is presssed by the operator to clear it
//   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT432);
// } else if (get_config_parameter(CONFIG_PARAM_TC3) > get_config_parameter(CONFIG_PARAM_TT432_IDLE_MAX)) {
//   // High side ONLY → IDLE S1
//   set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT432);
// } else {
//   // Release IDLE only if back below RUN_MAX
//   if (get_config_parameter(CONFIG_PARAM_TC3) < get_config_parameter(CONFIG_PARAM_TT432_RUN_MAX)) {
//     set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_TT432);
//   }
// }

// // Stage control for TC4 - TT439 (max-only)
// if (get_config_parameter(CONFIG_PARAM_TC4) > get_config_parameter(CONFIG_PARAM_TT439_STOP_MAX)) {
//   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT439);
// } else if (get_config_parameter(CONFIG_PARAM_TC4) > get_config_parameter(CONFIG_PARAM_TT439_IDLE_MAX)) {
//   set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT439);
// } else {
//   if (get_config_parameter(CONFIG_PARAM_TC4) < get_config_parameter(CONFIG_PARAM_TT439_RUN_MAX)) {
//     set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_TT439);
//   }
// }

// // Stage control for TC5 - TT435 (max-only)
// if (get_config_parameter(CONFIG_PARAM_TC5) > get_config_parameter(CONFIG_PARAM_TT435_STOP_MAX)) {
//   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT435);
// } else if (get_config_parameter(CONFIG_PARAM_TC5) > get_config_parameter(CONFIG_PARAM_TT435_IDLE_MAX)) {
//   set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT435);
// } else {
//   if (get_config_parameter(CONFIG_PARAM_TC5) < get_config_parameter(CONFIG_PARAM_TT435_RUN_MAX)) {
//     set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_TT435);
//   }
// }

// // Stage control for TC6 - TT436 (max-only)
// if (get_config_parameter(CONFIG_PARAM_TC6) > get_config_parameter(CONFIG_PARAM_TT436_STOP_MAX)) {
//   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT436);
// } else if (get_config_parameter(CONFIG_PARAM_TC6) > get_config_parameter(CONFIG_PARAM_TT436_IDLE_MAX)) {
//   set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT436);
// } else {
//   if (get_config_parameter(CONFIG_PARAM_TC6) < get_config_parameter(CONFIG_PARAM_TT436_RUN_MAX)) {
//     set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_TT436);
//   }
// }

// // Stage control for TC7 - TT447 (max-only)
// if (get_config_parameter(CONFIG_PARAM_TC7) > get_config_parameter(CONFIG_PARAM_TT447_STOP_MAX)) {
//   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT447);
// } else if (get_config_parameter(CONFIG_PARAM_TC7) > get_config_parameter(CONFIG_PARAM_TT447_IDLE_MAX)) {
//   set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT447);
// } else {
//   if (get_config_parameter(CONFIG_PARAM_TC7) < get_config_parameter(CONFIG_PARAM_TT447_RUN_MAX)) {
//     set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_TT447);
//   }
// }

// // Stage control for TC8 - TT454 (max-only)
// if (get_config_parameter(CONFIG_PARAM_TC8) > get_config_parameter(CONFIG_PARAM_TT454_STOP_MAX)) {
//   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT454);
// } else if (get_config_parameter(CONFIG_PARAM_TC8) > get_config_parameter(CONFIG_PARAM_TT454_IDLE_MAX)) {
//   set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT454);
// } else {
//   if (get_config_parameter(CONFIG_PARAM_TC8) < get_config_parameter(CONFIG_PARAM_TT454_RUN_MAX)) {
//     set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_TT454);
//   }
// }

// // Stage control for TC9 - TT450 (max-only)
// if (get_config_parameter(CONFIG_PARAM_TC9) > get_config_parameter(CONFIG_PARAM_TT450_STOP_MAX)) {
//   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT450);
// } else if (get_config_parameter(CONFIG_PARAM_TC9) > get_config_parameter(CONFIG_PARAM_TT450_IDLE_MAX)) {
//   set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT450);
// } else {
//   if (get_config_parameter(CONFIG_PARAM_TC9) < get_config_parameter(CONFIG_PARAM_TT450_RUN_MAX)) {
//     set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_TT450);
//   }
// }

// // Stage control for TC10 - TT451 (max-only)
// if (get_config_parameter(CONFIG_PARAM_TC10) > get_config_parameter(CONFIG_PARAM_TT451_STOP_MAX)) {
//   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT451);
// } else if (get_config_parameter(CONFIG_PARAM_TC10) > get_config_parameter(CONFIG_PARAM_TT451_IDLE_MAX)) {
//   set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT451);
// } else {
//   if (get_config_parameter(CONFIG_PARAM_TC10) < get_config_parameter(CONFIG_PARAM_TT451_RUN_MAX)) {
//     set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_TT451);
//   }
// }

// // Stage control for TC11 - TT462 (max-only)
// if (get_config_parameter(CONFIG_PARAM_TC11) > get_config_parameter(CONFIG_PARAM_TT462_STOP_MAX)) {
//   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT462);
// } else if (get_config_parameter(CONFIG_PARAM_TC11) > get_config_parameter(CONFIG_PARAM_TT462_IDLE_MAX)) {
//   set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT462);
// } else {
//   if (get_config_parameter(CONFIG_PARAM_TC11) < get_config_parameter(CONFIG_PARAM_TT462_RUN_MAX)) {
//     set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_TT462);
//   }
// }

// // Stage control for TC12 - TT474 (max-only)
// if (get_config_parameter(CONFIG_PARAM_TC12) > get_config_parameter(CONFIG_PARAM_TT474_STOP_MAX)) {
//   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT474);
// } else if (get_config_parameter(CONFIG_PARAM_TC12) > get_config_parameter(CONFIG_PARAM_TT474_IDLE_MAX)) {
//   set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT474);
// } else {
//   if (get_config_parameter(CONFIG_PARAM_TC12) < get_config_parameter(CONFIG_PARAM_TT474_RUN_MAX)) {
//     set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_TT474);
//   }
// }

// // Stage control for TC13 - TT465 (max-only)
// if (get_config_parameter(CONFIG_PARAM_TC13) > get_config_parameter(CONFIG_PARAM_TT465_STOP_MAX)) {
//   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT465);
// } else if (get_config_parameter(CONFIG_PARAM_TC13) > get_config_parameter(CONFIG_PARAM_TT465_IDLE_MAX)) {
//   set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT465);
// } else {
//   if (get_config_parameter(CONFIG_PARAM_TC13) < get_config_parameter(CONFIG_PARAM_TT465_RUN_MAX)) {
//     set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_TT465);
//   }
// }

// // Stage control for TC14 - TT471 (max-only)
// if (get_config_parameter(CONFIG_PARAM_TC14) > get_config_parameter(CONFIG_PARAM_TT471_STOP_MAX)) {
//   set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT471);
// } else if (get_config_parameter(CONFIG_PARAM_TC14) > get_config_parameter(CONFIG_PARAM_TT471_IDLE_MAX)) {
//   set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT471);
// } else {
//   if (get_config_parameter(CONFIG_PARAM_TC14) < get_config_parameter(CONFIG_PARAM_TT471_RUN_MAX)) {
//     set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_TT471);
//   }
// }


//     //Stage control for TC1-TT532
//     if ((get_config_parameter(CONFIG_PARAM_TC1) < get_config_parameter(CONFIG_PARAM_TT532_STOP_MIN)) 
//               || (get_config_parameter(CONFIG_PARAM_TC1) > get_config_parameter(CONFIG_PARAM_TT532_STOP_MAX))){          
//       // This error will remain here until the red button is presssed by the operator to clear it
//       set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT532);
//     }else if (get_config_parameter(CONFIG_PARAM_TC1) < get_config_parameter(CONFIG_PARAM_TT532_IDLE_MIN)){
//       // When Coolant-1 is low, IDLE S1, S2 & S3
//       set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT532);
//       set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT532);
//       set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT532);
//     }else if (get_config_parameter(CONFIG_PARAM_TC1) > get_config_parameter(CONFIG_PARAM_TT532_IDLE_MAX)){
//       // When Coolant-1 is high, IDLE S1, S2 & S3
//       set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT532);
//       set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT532);
//       set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT532);
//     }else if ((get_config_parameter(CONFIG_PARAM_TC1) < get_config_parameter(CONFIG_PARAM_TT532_RUN_MAX)) 
//               && (get_config_parameter(CONFIG_PARAM_TC1) > get_config_parameter(CONFIG_PARAM_TT532_RUN_MIN))){
//         // When Coolant-1 is in range RUN (No longer need to IDLE) S1, S2 & S3
//         set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_TT532);
//         set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_TT532);
//         set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_TT532);
//     }

//     //Stage control for TC2-TT557
//     if ((get_config_parameter(CONFIG_PARAM_TC2) < get_config_parameter(CONFIG_PARAM_TT557_STOP_MIN)) 
//               || (get_config_parameter(CONFIG_PARAM_TC2) > get_config_parameter(CONFIG_PARAM_TT557_STOP_MAX))){          
//       // This error will remain here until the red button is presssed by the operator to clear it
//       set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT557);
//     }else if (get_config_parameter(CONFIG_PARAM_TC2) < get_config_parameter(CONFIG_PARAM_TT557_IDLE_MIN)){
//       // When Coolant-2 is low, IDLE S1, S2 & S3
//       set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT557);
//       set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT557);
//       set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT557);
//     }else if (get_config_parameter(CONFIG_PARAM_TC2) > get_config_parameter(CONFIG_PARAM_TT557_IDLE_MAX)){
//       // When Coolant-2 is high, IDLE S1, S2 & S3
//       set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT557);
//       set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT557);
//       set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT557);
//     }else if ((get_config_parameter(CONFIG_PARAM_TC2) < get_config_parameter(CONFIG_PARAM_TT557_RUN_MAX)) 
//               && (get_config_parameter(CONFIG_PARAM_TC2) > get_config_parameter(CONFIG_PARAM_TT557_RUN_MIN))){
//         // When Coolant-1 is in range RUN (No longer need to IDLE) S1, S2 & S3
//         set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_TT557);
//         set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_TT557);
//         set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_TT557);
//     }

// }

void parameter_check(){

  // ===== PT405 – Suction PT =====
  if ((get_config_parameter(CONFIG_PARAM_SUCTION_PT) < get_config_parameter(CONFIG_PARAM_PT405_STOP_MIN)) 
   || (get_config_parameter(CONFIG_PARAM_SUCTION_PT) > get_config_parameter(CONFIG_PARAM_PT405_STOP_MAX))) {
    set_config_bit(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT405);
    Serial.print("[PT405] ERROR set: SUCTION_PT="); Serial.print(get_config_parameter(CONFIG_PARAM_SUCTION_PT),2);
    Serial.print(" out of [STOP_MIN="); Serial.print(get_config_parameter(CONFIG_PARAM_PT405_STOP_MIN),2);
    Serial.print(", STOP_MAX="); Serial.print(get_config_parameter(CONFIG_PARAM_PT405_STOP_MAX),2); Serial.println("]");
  } else if (get_config_parameter(CONFIG_PARAM_SUCTION_PT) < get_config_parameter(CONFIG_PARAM_PT405_IDLE_MIN)) {
    set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT405);
    Serial.print("[PT405] S1 IDLE set: SUCTION_PT="); Serial.print(get_config_parameter(CONFIG_PARAM_SUCTION_PT),2);
    Serial.print(" < IDLE_MIN="); Serial.println(get_config_parameter(CONFIG_PARAM_PT405_IDLE_MIN),2);
  } else if (get_config_parameter(CONFIG_PARAM_SUCTION_PT) > get_config_parameter(CONFIG_PARAM_PT405_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT405);
    Serial.print("[PT405] S1 IDLE set: SUCTION_PT="); Serial.print(get_config_parameter(CONFIG_PARAM_SUCTION_PT),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_PT405_IDLE_MAX),2);
  } else if ((get_config_parameter(CONFIG_PARAM_SUCTION_PT) < get_config_parameter(CONFIG_PARAM_PT405_RUN_MAX)) 
          && (get_config_parameter(CONFIG_PARAM_SUCTION_PT) > get_config_parameter(CONFIG_PARAM_PT405_RUN_MIN))) {
    set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT405);
    Serial.print("[PT405] S1 IDLE cleared: SUCTION_PT in RUN band (");
    Serial.print(get_config_parameter(CONFIG_PARAM_PT405_RUN_MIN),2); Serial.print("..");
    Serial.print(get_config_parameter(CONFIG_PARAM_PT405_RUN_MAX),2); Serial.println(")");
  }

  // ===== PT426 – Suction TANK PT =====
  if ((get_config_parameter(CONFIG_PARAM_SUCTION_TANK) < get_config_parameter(CONFIG_PARAM_PT426_STOP_MIN)) 
   || (get_config_parameter(CONFIG_PARAM_SUCTION_TANK) > get_config_parameter(CONFIG_PARAM_PT426_STOP_MAX))) {
    set_config_bit(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT426);
    Serial.print("[PT426] ERROR set: SUCTION_TANK="); Serial.print(get_config_parameter(CONFIG_PARAM_SUCTION_TANK),2);
    Serial.print(" out of [STOP_MIN="); Serial.print(get_config_parameter(CONFIG_PARAM_PT426_STOP_MIN),2);
    Serial.print(", STOP_MAX="); Serial.print(get_config_parameter(CONFIG_PARAM_PT426_STOP_MAX),2); Serial.println("]");
  } else if (get_config_parameter(CONFIG_PARAM_SUCTION_TANK) < get_config_parameter(CONFIG_PARAM_PT426_IDLE_MIN)) {
    set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT426);
    Serial.print("[PT426] S1 IDLE set: SUCTION_TANK="); Serial.print(get_config_parameter(CONFIG_PARAM_SUCTION_TANK),2);
    Serial.print(" < IDLE_MIN="); Serial.println(get_config_parameter(CONFIG_PARAM_PT426_IDLE_MIN),2);
  } else if (get_config_parameter(CONFIG_PARAM_SUCTION_TANK) > get_config_parameter(CONFIG_PARAM_PT426_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT426);
    Serial.print("[PT426] S1 IDLE set: SUCTION_TANK="); Serial.print(get_config_parameter(CONFIG_PARAM_SUCTION_TANK),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_PT426_IDLE_MAX),2);
  } else if ((get_config_parameter(CONFIG_PARAM_SUCTION_TANK) < get_config_parameter(CONFIG_PARAM_PT426_RUN_MAX)) 
          && (get_config_parameter(CONFIG_PARAM_SUCTION_TANK) > get_config_parameter(CONFIG_PARAM_PT426_RUN_MIN))) {
    set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT426);
    Serial.print("[PT426] S1 IDLE cleared: SUCTION_TANK in RUN band (");
    Serial.print(get_config_parameter(CONFIG_PARAM_PT426_RUN_MIN),2); Serial.print("..");
    Serial.print(get_config_parameter(CONFIG_PARAM_PT426_RUN_MAX),2); Serial.println(")");
  }

  // ===== PT496 – HFL Pump PT (idles ALL stages) =====
  if ((get_config_parameter(CONFIG_PARAM_HFL_PUMP1) < get_config_parameter(CONFIG_PARAM_PT496_STOP_MIN)) 
   || (get_config_parameter(CONFIG_PARAM_HFL_PUMP1) > get_config_parameter(CONFIG_PARAM_PT496_STOP_MAX))) {
    set_config_bit(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT496);
    Serial.print("[PT496] ERROR set: HFL_PUMP1="); Serial.print(get_config_parameter(CONFIG_PARAM_HFL_PUMP1),2);
    Serial.print(" out of [STOP_MIN="); Serial.print(get_config_parameter(CONFIG_PARAM_PT496_STOP_MIN),2);
    Serial.print(", STOP_MAX="); Serial.print(get_config_parameter(CONFIG_PARAM_PT496_STOP_MAX),2); Serial.println("]");
  } else if (get_config_parameter(CONFIG_PARAM_HFL_PUMP1) < get_config_parameter(CONFIG_PARAM_PT496_IDLE_MIN)) {
    set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT496);
    set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT496);
    set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT496);
    Serial.print("[PT496] S1,S2,S3 IDLE set: HFL_PUMP1="); Serial.print(get_config_parameter(CONFIG_PARAM_HFL_PUMP1),2);
    Serial.print(" < IDLE_MIN="); Serial.println(get_config_parameter(CONFIG_PARAM_PT496_IDLE_MIN),2);
  } else if (get_config_parameter(CONFIG_PARAM_HFL_PUMP1) > get_config_parameter(CONFIG_PARAM_PT496_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT496);
    set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT496);
    set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT496);
    Serial.print("[PT496] S1,S2,S3 IDLE set: HFL_PUMP1="); Serial.print(get_config_parameter(CONFIG_PARAM_HFL_PUMP1),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_PT496_IDLE_MAX),2);
  } else if ((get_config_parameter(CONFIG_PARAM_HFL_PUMP1) < get_config_parameter(CONFIG_PARAM_PT496_RUN_MAX)) 
          && (get_config_parameter(CONFIG_PARAM_HFL_PUMP1) > get_config_parameter(CONFIG_PARAM_PT496_RUN_MIN))) {
    set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT496);
    set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT496);
    set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT496);
    Serial.print("[PT496] S1,S2,S3 IDLE cleared: HFL_PUMP1 in RUN band (");
    Serial.print(get_config_parameter(CONFIG_PARAM_PT496_RUN_MIN),2); Serial.print("..");
    Serial.print(get_config_parameter(CONFIG_PARAM_PT496_RUN_MAX),2); Serial.println(")");
  }

  // ===== PT531 – Coolant1 PT (idles ALL stages) =====
  if ((get_config_parameter(CONFIG_PARAM_COOLANT1_PT) < get_config_parameter(CONFIG_PARAM_PT531_STOP_MIN)) 
   || (get_config_parameter(CONFIG_PARAM_COOLANT1_PT) > get_config_parameter(CONFIG_PARAM_PT531_STOP_MAX))) {
    set_config_bit(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT531);
    Serial.print("[PT531] ERROR set: COOLANT1_PT="); Serial.print(get_config_parameter(CONFIG_PARAM_COOLANT1_PT),2);
    Serial.print(" out of [STOP_MIN="); Serial.print(get_config_parameter(CONFIG_PARAM_PT531_STOP_MIN),2);
    Serial.print(", STOP_MAX="); Serial.print(get_config_parameter(CONFIG_PARAM_PT531_STOP_MAX),2); Serial.println("]");
  } else if (get_config_parameter(CONFIG_PARAM_COOLANT1_PT) < get_config_parameter(CONFIG_PARAM_PT531_IDLE_MIN)) {
    set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT531);
    set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT531);
    set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT531);
    Serial.print("[PT531] S1,S2,S3 IDLE set: COOLANT1_PT="); Serial.print(get_config_parameter(CONFIG_PARAM_COOLANT1_PT),2);
    Serial.print(" < IDLE_MIN="); Serial.println(get_config_parameter(CONFIG_PARAM_PT531_IDLE_MIN),2);
  } else if (get_config_parameter(CONFIG_PARAM_COOLANT1_PT) > get_config_parameter(CONFIG_PARAM_PT531_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT531);
    set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT531);
    set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT531);
    Serial.print("[PT531] S1,S2,S3 IDLE set: COOLANT1_PT="); Serial.print(get_config_parameter(CONFIG_PARAM_COOLANT1_PT),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_PT531_IDLE_MAX),2);
  } else if ((get_config_parameter(CONFIG_PARAM_COOLANT1_PT) < get_config_parameter(CONFIG_PARAM_PT531_RUN_MAX)) 
          && (get_config_parameter(CONFIG_PARAM_COOLANT1_PT) > get_config_parameter(CONFIG_PARAM_PT531_RUN_MIN))) {
    set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT531);
    set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT531);
    set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT531);
    Serial.print("[PT531] S1,S2,S3 IDLE cleared: COOLANT1_PT in RUN band (");
    Serial.print(get_config_parameter(CONFIG_PARAM_PT531_RUN_MIN),2); Serial.print("..");
    Serial.print(get_config_parameter(CONFIG_PARAM_PT531_RUN_MAX),2); Serial.println(")");
  }

  // ===== PT576 – Coolant2 PT (idles ALL stages) =====
  if ((get_config_parameter(CONFIG_PARAM_COOLANT2_PT) < get_config_parameter(CONFIG_PARAM_PT576_STOP_MIN)) 
   || (get_config_parameter(CONFIG_PARAM_COOLANT2_PT) > get_config_parameter(CONFIG_PARAM_PT576_STOP_MAX))) {
    set_config_bit(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT576);
    Serial.print("[PT576] ERROR set: COOLANT2_PT="); Serial.print(get_config_parameter(CONFIG_PARAM_COOLANT2_PT),2);
    Serial.print(" out of [STOP_MIN="); Serial.print(get_config_parameter(CONFIG_PARAM_PT576_STOP_MIN),2);
    Serial.print(", STOP_MAX="); Serial.print(get_config_parameter(CONFIG_PARAM_PT576_STOP_MAX),2); Serial.println("]");
  } else if (get_config_parameter(CONFIG_PARAM_COOLANT2_PT) < get_config_parameter(CONFIG_PARAM_PT576_IDLE_MIN)) {
    set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT576);
    set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT576);
    set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT576);
    Serial.print("[PT576] S1,S2,S3 IDLE set: COOLANT2_PT="); Serial.print(get_config_parameter(CONFIG_PARAM_COOLANT2_PT),2);
    Serial.print(" < IDLE_MIN="); Serial.println(get_config_parameter(CONFIG_PARAM_PT576_IDLE_MIN),2);
  } else if (get_config_parameter(CONFIG_PARAM_COOLANT2_PT) > get_config_parameter(CONFIG_PARAM_PT576_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT576);
    set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT576);
    set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT576);
    Serial.print("[PT576] S1,S2,S3 IDLE set: COOLANT2_PT="); Serial.print(get_config_parameter(CONFIG_PARAM_COOLANT2_PT),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_PT576_IDLE_MAX),2);
  } else if ((get_config_parameter(CONFIG_PARAM_COOLANT2_PT) < get_config_parameter(CONFIG_PARAM_PT576_RUN_MAX)) 
          && (get_config_parameter(CONFIG_PARAM_COOLANT2_PT) > get_config_parameter(CONFIG_PARAM_PT576_RUN_MIN))) {
    set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT576);
    set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT576);
    set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT576);
    Serial.print("[PT576] S1,S2,S3 IDLE cleared: COOLANT2_PT in RUN band (");
    Serial.print(get_config_parameter(CONFIG_PARAM_PT576_RUN_MIN),2); Serial.print("..");
    Serial.print(get_config_parameter(CONFIG_PARAM_PT576_RUN_MAX),2); Serial.println(")");
  }

  // ===== Ratio gates (PT442 / PT457 / PT487) =====
  const float pt426 = (float)get_config_parameter(CONFIG_PARAM_SUCTION_TANK);   // PT426
  const float pt442 = (float)get_config_parameter(CONFIG_PARAM_STAGE1_TANK);    // PT442
  const float pt457 = (float)get_config_parameter(CONFIG_PARAM_STAGE2_TANK);    // PT457
  const float pt487 = (float)get_config_parameter(CONFIG_PARAM_STAGE3_PT);      // PT487

  const float R_MIN  = 2.75f;
  const float R_MAX  = 3.25f;
  const float HYST_L = 1.05f;  // +5% to clear low-side idle
  const float HYST_H = 0.95f;  // -5% to clear high-side idle

  // --- PT426 → PT442 ---
  {
    const float pt442_low  = pt426 * R_MIN;
    const float pt442_high = pt426 * R_MAX;
    set_config_parameter(CONFIG_PARAM_PT442_RATIO_LOW_PSI,  pt442_low);
    set_config_parameter(CONFIG_PARAM_PT442_RATIO_HIGH_PSI, pt442_high);

    if (pt442 < pt442_low) {
      set_config_parameter(IDLE_STATE_S2_ALL, TRUE,  IDLE_STATE_PT442);
      Serial.print("[PT442] S2 IDLE set (ratio low): pt442="); Serial.print(pt442,2);
      Serial.print(" < low="); Serial.println(pt442_low,2);
    } else if (test_config_parameter(IDLE_STATE_S2_ALL, IDLE_STATE_PT442) && (pt442 >= (pt442_low * HYST_L))) {
      set_config_parameter(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT442);
      Serial.print("[PT442] S2 IDLE cleared (ratio recovered): pt442="); Serial.print(pt442,2);
      Serial.print(" >= low*1.05="); Serial.println(pt442_low * HYST_L,2);
    }

    if (pt442 > pt442_high) {
      set_config_parameter(IDLE_STATE_S1_ALL, TRUE,  IDLE_STATE_PT442);
      Serial.print("[PT442] S1 IDLE set (ratio high): pt442="); Serial.print(pt442,2);
      Serial.print(" > high="); Serial.println(pt442_high,2);
    } else if (test_config_parameter(IDLE_STATE_S1_ALL, IDLE_STATE_PT442) && (pt442 <= (pt442_high * HYST_H))) {
      set_config_parameter(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT442);
      Serial.print("[PT442] S1 IDLE cleared (ratio relaxed): pt442="); Serial.print(pt442,2);
      Serial.print(" <= high*0.95="); Serial.println(pt442_high * HYST_H,2);
    }
  }

  // Site-specific MAX protections for PT442 (with guard to avoid collision)
  //Site-specific MAX protections for PT442 (unchanged)
  if ((get_config_parameter(CONFIG_PARAM_STAGE1_TANK) > get_config_parameter(CONFIG_PARAM_PT442_STOP_MAX))){
    set_config_parameter(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT442);
    Serial.println("ERROR PRESS - PT442");
  } else if (get_config_parameter(CONFIG_PARAM_STAGE1_TANK) > get_config_parameter(CONFIG_PARAM_PT442_IDLE_MAX)){
    set_config_parameter(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT442);
  } else if (get_config_parameter(CONFIG_PARAM_STAGE1_TANK) < get_config_parameter(CONFIG_PARAM_PT442_RUN_MAX)){
    set_config_parameter(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT442);
  }

  // --- PT442 → PT457 ---
  {
    const float pt457_low  = pt442 * R_MIN;
    const float pt457_high = pt442 * R_MAX;
    set_config_parameter(CONFIG_PARAM_PT457_RATIO_LOW_PSI,  pt457_low);
    set_config_parameter(CONFIG_PARAM_PT457_RATIO_HIGH_PSI, pt457_high);

    if (pt457 < pt457_low) {
      set_config_parameter(IDLE_STATE_S3_ALL, TRUE,  IDLE_STATE_PT457);
      Serial.print("[PT457] S3 IDLE set (ratio low): pt457="); Serial.print(pt457,2);
      Serial.print(" < low="); Serial.println(pt457_low,2);
    } else if (test_config_parameter(IDLE_STATE_S3_ALL, IDLE_STATE_PT457) && (pt457 >= (pt457_low * HYST_L))) {
      set_config_parameter(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT457);
      Serial.print("[PT457] S3 IDLE cleared (ratio recovered): pt457="); Serial.print(pt457,2);
      Serial.print(" >= low*1.05="); Serial.println(pt457_low * HYST_L,2);
    }

    if (pt457 > pt457_high) {
      set_config_parameter(IDLE_STATE_S2_ALL, TRUE,  IDLE_STATE_PT457);
      Serial.print("[PT457] S2 IDLE set (ratio high): pt457="); Serial.print(pt457,2);
      Serial.print(" > high="); Serial.println(pt457_high,2);
    } else if (test_config_parameter(IDLE_STATE_S2_ALL, IDLE_STATE_PT457) && (pt457 <= (pt457_high * HYST_H))) {
      set_config_parameter(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT457);
      Serial.print("[PT457] S2 IDLE cleared (ratio relaxed): pt457="); Serial.print(pt457,2);
      Serial.print(" <= high*0.95="); Serial.println(pt457_high * HYST_H,2);
    }
  }

  // Site-specific MAX protections for PT457 (no special guard needed normally)
  if (get_config_parameter(CONFIG_PARAM_STAGE2_TANK) > get_config_parameter(CONFIG_PARAM_PT457_STOP_MAX)){
    set_config_parameter(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT457);
    Serial.println("[PT457] ERROR set: STAGE2_TANK > STOP_MAX");
  } else if (get_config_parameter(CONFIG_PARAM_STAGE2_TANK) > get_config_parameter(CONFIG_PARAM_PT457_IDLE_MAX)){
    set_config_parameter(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT457);
    Serial.println("[PT457] S2 IDLE set (site): STAGE2_TANK > IDLE_MAX");
  } else if (get_config_parameter(CONFIG_PARAM_STAGE2_TANK) < get_config_parameter(CONFIG_PARAM_PT457_RUN_MAX)){
    set_config_parameter(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT457);
    Serial.println("[PT457] S2 IDLE cleared (site): STAGE2_TANK < RUN_MAX");
  }

  // --- PT457 → PT487 ---
  {
    const float pt487_low  = pt457 * R_MIN;
    const float pt487_high = pt457 * R_MAX;
    set_config_parameter(CONFIG_PARAM_PT487_RATIO_LOW_PSI,  pt487_low);
    set_config_parameter(CONFIG_PARAM_PT487_RATIO_HIGH_PSI, pt487_high);

    if (pt487 < pt487_low) {
      set_config_parameter(IDLE_STATE_S3_ALL, TRUE,  IDLE_STATE_PT487);
      Serial.print("[PT487] S3 IDLE set (ratio low): pt487="); Serial.print(pt487,2);
      Serial.print(" < low="); Serial.println(pt487_low,2);
    } else if (test_config_parameter(IDLE_STATE_S3_ALL, IDLE_STATE_PT487) && (pt487 >= (pt487_low * HYST_L))) {
      set_config_parameter(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT487);
      Serial.print("[PT487] S3 IDLE cleared (ratio recovered): pt487="); Serial.print(pt487,2);
      Serial.print(" >= low*1.05="); Serial.println(pt487_low * HYST_L,2);
    }

    if (pt487 >= pt487_high) {
      set_config_parameter(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_PT487);
      set_config_parameter(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_PT487);
      set_config_parameter(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_PT487);
      Serial.print("[PT487] S1,S2,S3 IDLE set (ratio high): pt487="); Serial.print(pt487,2);
      Serial.print(" >= high="); Serial.println(pt487_high,2);
    } else {
      const bool s1_hi = test_config_parameter(IDLE_STATE_S1_ALL, IDLE_STATE_PT487);
      const bool s2_hi = test_config_parameter(IDLE_STATE_S2_ALL, IDLE_STATE_PT487);
      const bool s3_hi = test_config_parameter(IDLE_STATE_S3_ALL, IDLE_STATE_PT487);
      if ((s1_hi || s2_hi || s3_hi) && (pt487 <= (pt487_high * HYST_H))) {
        set_config_parameter(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_PT487);
        set_config_parameter(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_PT487);
        set_config_parameter(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT487);
        Serial.print("[PT487] S1,S2,S3 IDLE cleared (ratio relaxed): pt487="); Serial.print(pt487,2);
        Serial.print(" <= high*0.95="); Serial.println(pt487_high * HYST_H,2);
      }
    }
  }

  // Site-specific MAX protections for PT487
  if (get_config_parameter(CONFIG_PARAM_STAGE3_PT) > get_config_parameter(CONFIG_PARAM_PT487_STOP_MAX)){
    set_config_parameter(ERROR_PARAM_ALL, ERROR_ACTIVE, ERROR_PT487);
    Serial.println("[PT487] ERROR set: STAGE3_PT > STOP_MAX");
  } else if (get_config_parameter(CONFIG_PARAM_STAGE3_PT) > get_config_parameter(CONFIG_PARAM_PT487_IDLE_MAX)){
    set_config_parameter(IDLE_STATE_S3_ALL, TRUE,  IDLE_STATE_PT487);
    Serial.println("[PT487] S3 IDLE set (site): STAGE3_PT > IDLE_MAX");
  } else if (get_config_parameter(CONFIG_PARAM_STAGE3_PT) < get_config_parameter(CONFIG_PARAM_PT487_RUN_MAX)){
    set_config_parameter(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_PT487);
    Serial.println("[PT487] S3 IDLE cleared (site): STAGE3_PT < RUN_MAX");
  }

  // ===== TC/TT max-only sections (all with prints) =====
  // TC3 – TT432
  if (get_config_parameter(CONFIG_PARAM_TC3) > get_config_parameter(CONFIG_PARAM_TT432_STOP_MAX)) {
    set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT432);
    Serial.print("[TT432] ERROR set: TC3="); Serial.print(get_config_parameter(CONFIG_PARAM_TC3),2);
    Serial.print(" > STOP_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT432_STOP_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC3) > get_config_parameter(CONFIG_PARAM_TT432_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT432);
    Serial.print("[TT432] S1 IDLE set: TC3="); Serial.print(get_config_parameter(CONFIG_PARAM_TC3),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT432_IDLE_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC3) < get_config_parameter(CONFIG_PARAM_TT432_RUN_MAX)) {
    set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_TT432);
    Serial.print("[TT432] S1 IDLE cleared: TC3 < RUN_MAX=");
    Serial.println(get_config_parameter(CONFIG_PARAM_TT432_RUN_MAX),2);
  }

  // TC4 – TT439
  if (get_config_parameter(CONFIG_PARAM_TC4) > get_config_parameter(CONFIG_PARAM_TT439_STOP_MAX)) {
    set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT439);
    Serial.print("[TT439] ERROR set: TC4="); Serial.print(get_config_parameter(CONFIG_PARAM_TC4),2);
    Serial.print(" > STOP_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT439_STOP_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC4) > get_config_parameter(CONFIG_PARAM_TT439_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT439);
    Serial.print("[TT439] S1 IDLE set: TC4="); Serial.print(get_config_parameter(CONFIG_PARAM_TC4),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT439_IDLE_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC4) < get_config_parameter(CONFIG_PARAM_TT439_RUN_MAX)) {
    set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_TT439);
    Serial.print("[TT439] S1 IDLE cleared: TC4 < RUN_MAX=");
    Serial.println(get_config_parameter(CONFIG_PARAM_TT439_RUN_MAX),2);
  }

  // TC5 – TT435
  if (get_config_parameter(CONFIG_PARAM_TC5) > get_config_parameter(CONFIG_PARAM_TT435_STOP_MAX)) {
    set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT435);
    Serial.print("[TT435] ERROR set: TC5="); Serial.print(get_config_parameter(CONFIG_PARAM_TC5),2);
    Serial.print(" > STOP_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT435_STOP_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC5) > get_config_parameter(CONFIG_PARAM_TT435_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT435);
    Serial.print("[TT435] S1 IDLE set: TC5="); Serial.print(get_config_parameter(CONFIG_PARAM_TC5),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT435_IDLE_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC5) < get_config_parameter(CONFIG_PARAM_TT435_RUN_MAX)) {
    set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_TT435);
    Serial.print("[TT435] S1 IDLE cleared: TC5 < RUN_MAX=");
    Serial.println(get_config_parameter(CONFIG_PARAM_TT435_RUN_MAX),2);
  }

  // TC6 – TT436
  if (get_config_parameter(CONFIG_PARAM_TC6) > get_config_parameter(CONFIG_PARAM_TT436_STOP_MAX)) {
    set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT436);
    Serial.print("[TT436] ERROR set: TC6="); Serial.print(get_config_parameter(CONFIG_PARAM_TC6),2);
    Serial.print(" > STOP_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT436_STOP_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC6) > get_config_parameter(CONFIG_PARAM_TT436_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S1_ALL, TRUE, IDLE_STATE_TT436);
    Serial.print("[TT436] S1 IDLE set: TC6="); Serial.print(get_config_parameter(CONFIG_PARAM_TC6),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT436_IDLE_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC6) < get_config_parameter(CONFIG_PARAM_TT436_RUN_MAX)) {
    set_config_bit(IDLE_STATE_S1_ALL, FALSE, IDLE_STATE_TT436);
    Serial.print("[TT436] S1 IDLE cleared: TC6 < RUN_MAX=");
    Serial.println(get_config_parameter(CONFIG_PARAM_TT436_RUN_MAX),2);
  }

  // TC7 – TT447
  if (get_config_parameter(CONFIG_PARAM_TC7) > get_config_parameter(CONFIG_PARAM_TT447_STOP_MAX)) {
    set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT447);
    Serial.print("[TT447] ERROR set: TC7="); Serial.print(get_config_parameter(CONFIG_PARAM_TC7),2);
    Serial.print(" > STOP_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT447_STOP_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC7) > get_config_parameter(CONFIG_PARAM_TT447_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT447);
    Serial.print("[TT447] S2 IDLE set: TC7="); Serial.print(get_config_parameter(CONFIG_PARAM_TC7),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT447_IDLE_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC7) < get_config_parameter(CONFIG_PARAM_TT447_RUN_MAX)) {
    set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_TT447);
    Serial.print("[TT447] S2 IDLE cleared: TC7 < RUN_MAX=");
    Serial.println(get_config_parameter(CONFIG_PARAM_TT447_RUN_MAX),2);
  }

  // TC8 – TT454
  if (get_config_parameter(CONFIG_PARAM_TC8) > get_config_parameter(CONFIG_PARAM_TT454_STOP_MAX)) {
    set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT454);
    Serial.print("[TT454] ERROR set: TC8="); Serial.print(get_config_parameter(CONFIG_PARAM_TC8),2);
    Serial.print(" > STOP_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT454_STOP_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC8) > get_config_parameter(CONFIG_PARAM_TT454_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT454);
    Serial.print("[TT454] S2 IDLE set: TC8="); Serial.print(get_config_parameter(CONFIG_PARAM_TC8),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT454_IDLE_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC8) < get_config_parameter(CONFIG_PARAM_TT454_RUN_MAX)) {
    set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_TT454);
    Serial.print("[TT454] S2 IDLE cleared: TC8 < RUN_MAX=");
    Serial.println(get_config_parameter(CONFIG_PARAM_TT454_RUN_MAX),2);
  }

  // TC9 – TT450
  if (get_config_parameter(CONFIG_PARAM_TC9) > get_config_parameter(CONFIG_PARAM_TT450_STOP_MAX)) {
    set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT450);
    Serial.print("[TT450] ERROR set: TC9="); Serial.print(get_config_parameter(CONFIG_PARAM_TC9),2);
    Serial.print(" > STOP_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT450_STOP_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC9) > get_config_parameter(CONFIG_PARAM_TT450_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT450);
    Serial.print("[TT450] S2 IDLE set: TC9="); Serial.print(get_config_parameter(CONFIG_PARAM_TC9),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT450_IDLE_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC9) < get_config_parameter(CONFIG_PARAM_TT450_RUN_MAX)) {
    set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_TT450);
    Serial.print("[TT450] S2 IDLE cleared: TC9 < RUN_MAX=");
    Serial.println(get_config_parameter(CONFIG_PARAM_TT450_RUN_MAX),2);
  }

  // TC10 – TT451
  if (get_config_parameter(CONFIG_PARAM_TC10) > get_config_parameter(CONFIG_PARAM_TT451_STOP_MAX)) {
    set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT451);
    Serial.print("[TT451] ERROR set: TC10="); Serial.print(get_config_parameter(CONFIG_PARAM_TC10),2);
    Serial.print(" > STOP_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT451_STOP_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC10) > get_config_parameter(CONFIG_PARAM_TT451_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S2_ALL, TRUE, IDLE_STATE_TT451);
    Serial.print("[TT451] S2 IDLE set: TC10="); Serial.print(get_config_parameter(CONFIG_PARAM_TC10),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT451_IDLE_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC10) < get_config_parameter(CONFIG_PARAM_TT451_RUN_MAX)) {
    set_config_bit(IDLE_STATE_S2_ALL, FALSE, IDLE_STATE_TT451);
    Serial.print("[TT451] S2 IDLE cleared: TC10 < RUN_MAX=");
    Serial.println(get_config_parameter(CONFIG_PARAM_TT451_RUN_MAX),2);
  }

  // TC11 – TT462
  if (get_config_parameter(CONFIG_PARAM_TC11) > get_config_parameter(CONFIG_PARAM_TT462_STOP_MAX)) {
    set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT462);
    Serial.print("[TT462] ERROR set: TC11="); Serial.print(get_config_parameter(CONFIG_PARAM_TC11),2);
    Serial.print(" > STOP_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT462_STOP_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC11) > get_config_parameter(CONFIG_PARAM_TT462_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT462);
    Serial.print("[TT462] S3 IDLE set: TC11="); Serial.print(get_config_parameter(CONFIG_PARAM_TC11),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT462_IDLE_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC11) < get_config_parameter(CONFIG_PARAM_TT462_RUN_MAX)) {
    set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_TT462);
    Serial.print("[TT462] S3 IDLE cleared: TC11 < RUN_MAX=");
    Serial.println(get_config_parameter(CONFIG_PARAM_TT462_RUN_MAX),2);
  }

  // TC12 – TT474
  if (get_config_parameter(CONFIG_PARAM_TC12) > get_config_parameter(CONFIG_PARAM_TT474_STOP_MAX)) {
    set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT474);
    Serial.print("[TT474] ERROR set: TC12="); Serial.print(get_config_parameter(CONFIG_PARAM_TC12),2);
    Serial.print(" > STOP_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT474_STOP_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC12) > get_config_parameter(CONFIG_PARAM_TT474_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT474);
    Serial.print("[TT474] S3 IDLE set: TC12="); Serial.print(get_config_parameter(CONFIG_PARAM_TC12),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT474_IDLE_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC12) < get_config_parameter(CONFIG_PARAM_TT474_RUN_MAX)) {
    set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_TT474);
    Serial.print("[TT474] S3 IDLE cleared: TC12 < RUN_MAX=");
    Serial.println(get_config_parameter(CONFIG_PARAM_TT474_RUN_MAX),2);
  }

  // TC13 – TT465
  if (get_config_parameter(CONFIG_PARAM_TC13) > get_config_parameter(CONFIG_PARAM_TT465_STOP_MAX)) {
    set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT465);
    Serial.print("[TT465] ERROR set: TC13="); Serial.print(get_config_parameter(CONFIG_PARAM_TC13),2);
    Serial.print(" > STOP_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT465_STOP_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC13) > get_config_parameter(CONFIG_PARAM_TT465_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT465);
    Serial.print("[TT465] S3 IDLE set: TC13="); Serial.print(get_config_parameter(CONFIG_PARAM_TC13),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT465_IDLE_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC13) < get_config_parameter(CONFIG_PARAM_TT465_RUN_MAX)) {
    set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_TT465);
    Serial.print("[TT465] S3 IDLE cleared: TC13 < RUN_MAX=");
    Serial.println(get_config_parameter(CONFIG_PARAM_TT465_RUN_MAX),2);
  }

  // TC14 – TT471
  if (get_config_parameter(CONFIG_PARAM_TC14) > get_config_parameter(CONFIG_PARAM_TT471_STOP_MAX)) {
    set_config_bit(ERROR_PARAM2_ALL, ERROR_ACTIVE, ERROR_TT471);
    Serial.print("[TT471] ERROR set: TC14="); Serial.print(get_config_parameter(CONFIG_PARAM_TC14),2);
    Serial.print(" > STOP_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT471_STOP_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC14) > get_config_parameter(CONFIG_PARAM_TT471_IDLE_MAX)) {
    set_config_bit(IDLE_STATE_S3_ALL, TRUE, IDLE_STATE_TT471);
    Serial.print("[TT471] S3 IDLE set: TC14="); Serial.print(get_config_parameter(CONFIG_PARAM_TC14),2);
    Serial.print(" > IDLE_MAX="); Serial.println(get_config_parameter(CONFIG_PARAM_TT471_IDLE_MAX),2);
  } else if (get_config_parameter(CONFIG_PARAM_TC14) < get_config_parameter(CONFIG_PARAM_TT471_RUN_MAX)) {
    set_config_bit(IDLE_STATE_S3_ALL, FALSE, IDLE_STATE_TT471);
    Serial.print("[TT471] S3 IDLE cleared: TC14 < RUN_MAX=");
    Serial.println(get_config_parameter(CONFIG_PARAM_TT471_RUN_MAX),2);
  }

  // ---- Optional one-line status summary (enable if useful) ----
  /*
  Serial.print("[SUM] PT426="); Serial.print(pt426,1);
  Serial.print(" PT442="); Serial.print(pt442,1);
  Serial.print(" PT457="); Serial.print(pt457,1);
  Serial.print(" PT487="); Serial.print(pt487,1);
  Serial.print(" S1idle="); Serial.print(test_config_parameter(IDLE_STATE_S1_ALL, IDLE_STATE_PT442));
  Serial.print(" S2idle="); Serial.print(test_config_parameter(IDLE_STATE_S2_ALL, IDLE_STATE_PT457));
  Serial.print(" S3idle="); Serial.println(test_config_parameter(IDLE_STATE_S3_ALL, IDLE_STATE_PT487));
  */
}


/*
    Name:         C300_stroking_current
    Arguments:    bool    - The current state to which PCV2A is to be set to (On/Off)
                  bool    - The current state to which PCV2B is to be set to (On/Off)
                  
    Returns:      void
    Description:  function to update the PCVs. 
                  TODO: Simplify this into a single function call for easy to understand PCV operation
*/
uint16_t flow_meter1=0;
uint16_t pulses=0;
uint16_t flow_meter_expected_value=0;
bool stroke2=false;
bool stroke_a=true;


// bool c600_stroking_current(bool stroke_a, bool stroke_b, int flow_max_a, int flow_max_b, int pulse_in, int analog_out, int cpm_rate_index, unsigned long int * stroke_start_ptr){
//   float strk1_flowcount=get_config_parameter(flow_max_a);
//   float strk2_flowcount=get_config_parameter(flow_max_b);
//   float cpm_rate = get_config_parameter(cpm_rate_index);

//   flow_meter1= get_config_parameter(pulse_in);
//   // At 12 CPM, we need 24 strokes per minute

//   if (stroke_a && !stroke_b){        //side-A
//     flow_meter_expected_value = strk1_flowcount * ((cpm_rate * 2)/60000) * (millis() - *stroke_start_ptr);
//     if (flow_meter1 >= strk1_flowcount){  // End of the stroke
//       *stroke_start_ptr = millis();    // Start the timer to mark the start of the stroke
//       set_config_parameter(analog_out, 25);  // Now that we are stroking the other way, start at the midpoint of the reverse stroke
//       set_config_parameter(pulse_in, 0); // Reset the flow meter counter
//       return(false);
//     }else if (flow_meter1 < flow_meter_expected_value){       // Go faster!!! Speed up the stroke
//       //set PCV to 100% in direction 1
//       if (get_config_parameter(analog_out) < 90){  
//         set_config_parameter(analog_out, (get_config_parameter(analog_out) + 10));
//       }else{
//         set_config_parameter(analog_out, 100); // 100 is the upper limit for this direction
//       }
//     }else if (flow_meter1 > flow_meter_expected_value){   // Slow down the stroke
//       if (get_config_parameter(analog_out) > 60){
//         set_config_parameter(analog_out, (get_config_parameter(analog_out) - 10)); 
//       }else{
//         set_config_parameter(analog_out, 50);  // 50 is the lower limit for this direction
//       }
//     }
//   }else if (stroke_b && !stroke_a){   // Side-B
//     flow_meter_expected_value = strk2_flowcount * ((cpm_rate * 2)/60000) * (millis() - *stroke_start_ptr);
//     if (flow_meter1 >= strk2_flowcount){  // End of the stroke
//       *stroke_start_ptr = millis();    // Start the timer to mark the start of the stroke
//       set_config_parameter(analog_out, 75);  // Now that we are stroking the other way, start at the midpoint of the reverse stroke
//       set_config_parameter(pulse_in, 0); // Reset the flow meter counter
//       return(false);
//     }else if (flow_meter1 < flow_meter_expected_value){       // Go faster!!! Speed up the stroke
//       //set PCV to 100% in direction 2 (0%)
//       if (get_config_parameter(analog_out) > 10){
//         set_config_parameter(analog_out, (get_config_parameter(analog_out) - 10));
//       }else{
//         set_config_parameter(analog_out, 0); // 0 is the upper limit for this direction
//       }
//     }else if (flow_meter1 > flow_meter_expected_value){   // Slow down the stroke
//       if (get_config_parameter(analog_out) < 40){
//         set_config_parameter(analog_out, (get_config_parameter(analog_out) + 10)); 
//       }else{
//         set_config_parameter(analog_out, 50);  // 50 is the lower limit for this direction
//       }
//     }
//   }else if (stroke_a && stroke_b){    // Both sides are trying to stroke together - logic error!
//     //stop stroking on both sides and RAISE ERROR 
//     set_config_parameter(analog_out, 50);    
//     Serial.println("error both strokes high");
//   }else{  // Neither side is stroking. OK, but sit and wait!
//     set_config_parameter(pulse_in, 0);
//     set_config_parameter(analog_out, 50); 
//     *stroke_start_ptr = millis();
//     Serial.println("Neither strokes low");
//   }
//   return(true);
// }

// Linear map breakpoints (edit if needed)
static const int T_SLOPE_START_C = 120;  // <=120°C -> MAX_CPM
static const int T_SLOPE_END_C   = 135;  // >=135°C -> MIN_CPM

// NOTE: signature now includes cpm_actual_index *after* cpm_max_index (per your request)
bool c600_stroking_current(bool stroke_a, bool stroke_b, int flow_max_a, int flow_max_b, int pulse_in, int analog_out, int cpm_min_index, int cpm_max_index, int cpm_actual_index, 
                           int tempa_index, int tempb_index, unsigned long int * stroke_start_ptr){
  // ---- Read config values ----
  float strk1_flowcount = get_config_parameter(flow_max_a);
  float strk2_flowcount = get_config_parameter(flow_max_b);

  int tempa   = get_config_parameter(tempa_index);
  int tempb   = get_config_parameter(tempb_index);
  int temp    = (tempa > tempb) ? tempa : tempb;  // choose hotter of the two

  int min_cpm = get_config_parameter(cpm_min_index);
  int max_cpm = get_config_parameter(cpm_max_index);

  // Safety: ensure min <= max
  if (min_cpm > max_cpm) { int t = min_cpm; min_cpm = max_cpm; max_cpm = t; }

  // ---- Linear temperature → CPM (decrease as temperature increases) ----
  int cpm_i;
  if (temp <= T_SLOPE_START_C) {
    cpm_i = max_cpm;
  } else if (temp >= T_SLOPE_END_C) {
    cpm_i = min_cpm;
  } else {
    int range = (T_SLOPE_END_C - T_SLOPE_START_C);            // 15
    int delta = (temp - T_SLOPE_START_C);                     // 1..14
    int slope_times_1000 = ((max_cpm - min_cpm) * 1000) / range;
    cpm_i = max_cpm - ((slope_times_1000 * delta + 500) / 1000); // rounded
  }

  // --- Publish ACTUAL_CPM (always live) ---
  set_config_parameter(cpm_actual_index, cpm_i);

  // Rate used for expected-pulse math (original behavior unchanged)
  float cpm_rate = (float)cpm_i;

  // ---- Snapshot runtime (unchanged) ----
  unsigned long now_ms = millis();
  unsigned long dt_ms  = (now_ms - *stroke_start_ptr);
  float pulses         = get_config_parameter(pulse_in);

  // ================== ORIGINAL CONTROL LOGIC (unchanged) ==================

  if (stroke_a && !stroke_b) {        // ---- Side-A (command band 50..100) ----
    float flow_meter_expected_value =
      strk1_flowcount * ((cpm_rate * 2) / 60000.0f) * dt_ms;

    if (pulses >= strk1_flowcount) {     // End of stroke A
      *stroke_start_ptr = now_ms;                  // reset stroke timer
      set_config_parameter(analog_out, 25);        // midpoint for reverse stroke (B band 0..50)
      set_config_parameter(pulse_in, 0);           // reset pulse counter
      return false;
    } else if (pulses < flow_meter_expected_value) {   // Behind schedule -> speed up
      int cmd = get_config_parameter(analog_out);
      int new_cmd = (cmd < 90) ? (cmd + 10) : 100;
      set_config_parameter(analog_out, new_cmd);
    } else if (pulses > flow_meter_expected_value) {   // Ahead -> slow down
      int cmd = get_config_parameter(analog_out);
      int new_cmd = (cmd > 60) ? (cmd - 10) : 50;
      set_config_parameter(analog_out, new_cmd);
    }

  } else if (stroke_b && !stroke_a) {   // ---- Side-B (command band 50..0) ----
    float flow_meter_expected_value =
      strk2_flowcount * ((cpm_rate * 2) / 60000.0f) * dt_ms;

    if (pulses >= strk2_flowcount) {     // End of stroke B
      *stroke_start_ptr = now_ms;
      set_config_parameter(analog_out, 75);        // midpoint for reverse stroke (A band 50..100)
      set_config_parameter(pulse_in, 0);
      return false;
    } else if (pulses < flow_meter_expected_value) { // Behind -> speed up (toward 0)
      int cmd = get_config_parameter(analog_out);
      int new_cmd = (cmd > 10) ? (cmd - 10) : 0;
      set_config_parameter(analog_out, new_cmd);
    } else if (pulses > flow_meter_expected_value) { // Ahead -> slow down (toward 50)
      int cmd = get_config_parameter(analog_out);
      int new_cmd = (cmd < 40) ? (cmd + 10) : 50;
      set_config_parameter(analog_out, new_cmd);
    }

  } else if (stroke_a && stroke_b) {    // ---- Logic error: both true ----
    set_config_parameter(analog_out, 50);

  } else {                              // ---- Parked: both false ----
    set_config_parameter(pulse_in, 0);
    set_config_parameter(analog_out, 50);
    *stroke_start_ptr = now_ms;
  }

  return true; // still in progress
}

