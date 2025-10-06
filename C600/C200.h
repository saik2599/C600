/*
   Author: Christopher Glanville
   C200 Compressor header file. To acompany CompressorCore.ino
   Date: 25/5/24
   updated : 25/5/24
*/

#if !defined(C200_H)
  #define  C200_H
  #include <stdint.h>

  // Uncomment to print debug string to the serial port
  #define DEBUG_MODE              1

  #define     SYSTEM_STATE_STOP                     0
  #define     SYSTEM_STATE_IDLE                     1   // UNUSED
  #define     SYSTEM_STATE_STANDBY                  2
  #define     SYSTEM_STATE_STROKING_START           3
  #define     SYSTEM_STATE_STROKING_RUN             4

  uint16_t get_system_state(void);
  void loop_two(void);
  // Only one of the following to be defined
  //#define HARDWARE_MEGA         ARDUINO_MEGA
  #define HARDWARE_GIGA         ARDUINO_GIGA
  //#define HARDWARE_7_0
  // #define HARDWARE_7_5    // This version also includes v7.4
  // #define HARDWARE_7_9
  // #define HARDWARE_7_10
  #define HARDWARE_7_11
  //#define HARDWARE_6_3

  #define DEF_FIRMWARE_VERSION    6

  #if defined(HARDWARE_7_11) 
    #if defined(HARDWARE_6_3) || defined(HARDWARE_7_0) || defined(HARDWARE_7_5) || defined(HARDWARE_7_9) || defined(HARDWARE_7_10)
      #error "Only one PCB type can be defined"
    #endif
  #endif

  #if defined(HARDWARE_MEGA) && defined(HARDWARE_GIGA)
    #error "Only one hardware platform can be defined"
  #endif


  #if defined(HARDWARE_GIGA)
    // ********* DIGITAL INPUTS ***************
    #define D_ESTOP_SWITCH P0_5
    #define D_SCHMERSAL P1_0

    #define D_GLOBAL P1_1
    #define D_GREEN_BUTTON P1_2
    #define D_YELLOW_BUTTON P1_3
    #define D_RED_BUTTON P1_4
    #define D_SCHMERSAL_RESET P1_5
    //const int D_MAIN_RELAY_FEEDBACK = 30;
    #define D_FILTERCHECK P0_0

    #define D_COOLANT_FLOW P0_2  // was 48
    //#define D_HYDR_FLUID_LEVELS P0_0
    //************* DIGITAL OUTPUTS ***************
    //#define O_SUCTION_SOLENOID_INPUT2 P0_0
    #define O_HYD_PUMP_ON2 P4_0

    #define DE 12     //  ******* HMI RS-485 pins *****
    #define RE 12
    //-----------Power Enable Digital Outputs-----------//
    #define ESTOP_BREAK 40
    #define LED_PWR 22
    #define TRACO_24VDC 23
    //--------------------------------------------------//
    //-------------------Analog Inputs------------------//
    

    // ****************** THERMAL VARIABLES *********************
    // resistance at 25 degrees C
    #define THERMISTORNOMINAL 10000
    // temp. for nominal resistance (almost always 25 C)
    #define TEMPERATURENOMINAL 25
    // how many samples to take and average, more takes longer
    // but is more 'smooth'
    #define NUMSAMPLES 5
    // The beta coefficient of the thermistor (usually 3000-4000)
    #define BCOEFFICIENT -3900.4  //good
    #define BCOEFFICIENT_d -3980
    #define BCOEFFICIENT_c -3988.4
    #define BCOEFFICIENT_e 3988
    // the value of the 'other' resistor
    #define SERIESRESISTOR 10000
  #endif
#endif    // enbd if this header has already been included
