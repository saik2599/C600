/*
   Author: Christopher Glanville
   config_c200.h configuration header file. To acompany config_c200.cpp
   Date: 25/5/24
   updated : 25/5/24
*/

#if !defined(_CONFIG_C200_H)
  #define  _CONFIG_C200_H

  #include <stdint.h>

  typedef union
{
  float number;
  uint16_t words[2];
  uint8_t bytes[4];
  uint32_t uint_value;
} FLOATUNION_t;

  #define MSB         16
  #define LSB         0
  #define AS_INT      32
  #define AS_INT_32   34 //Added as sai has done in E100 32bit param write code

  // PM-904 (The pressure reading from FM-904)
 

  #define   CONFIG_PARAM_SLAVE_ID               2001

  #define   CONFIG_PARAM_SERIAL_NUM             2010

  #define   DEVICE_ACTIVE                       0
  #define   DEVICE_INACTIVE                     1

  #define   REMOTE_LOCKOUT_MASK_ALL             254
  #define   REMOTE_LOCKOUT_ALL                  255   // Remote lockout 

  #define   ERROR_CLEAR                         0
  #define   ERROR_ACTIVE                        1

  #define CONFIG_PARAM_OP_STATE   104 
  #define CONFIG_PARAM_MANUAL_BIT     5

// PM-904 (The pressure reading from FM-904)
  #define   CONFIG_PARAM_RELAYS                     1   // 
  #define   RELAY_BIT_24_1                          0   // 
  #define   RELAY_BIT_24_2                          1   //
  #define   RELAY_BIT_24_4                          2   //
  #define   RELAY_BIT_24_6                          3   // Suction XV-TB32        
  #define   RELAY_BIT_24_7                          4   //
  #define   RELAY_BIT_24_8                          5   //
  #define   RELAY_BIT_24_9                          6   // 
  #define   RELAY_BIT_24_10                         7   // 
  #define   RELAY_BIT_24_11                         8   //
  #define   RELAY_BIT_24_12                         9   // 
  #define   RELAY_BIT_24_13                        10   // 
  #define   RELAY_BIT_24_14                        11   //   
  #define   RELAY_BIT_24_15                        12   //  // Coolant pumps
  #define   RELAY_BIT_24_16                        13   //  
  #define   RELAY_BIT_24_28                        14   //  
  #define   RELAY_BIT_24_RS485                     15   // 
  #define   RELAY_BIT_24_34                        16   // 
  #define   RELAY_BIT_24_35                        17   // 
  #define   RELAY_BIT_SSR21A                       18   // 
  #define   RELAY_BIT_SSR22A                       19   // 
  #define   RELAY_BIT_SSR23A                       20   // 
  
  #define   CONFIG_PARAM_INPUTS              3   // Digital input states
  #define   INPUT_BIT_24_0                   0   // Global Interlock
  #define   INPUT_BIT_24_1                   1   // Local Interlock Loop Input
  #define   INPUT_BIT_24_2                   2   // Remote Reset Input
  #define   INPUT_BIT_24_4                   3   // Amber Pilot Input
  #define   INPUT_BIT_24_6                   4   // XV GIP/ Suction Solenoid Valve Input
  #define   INPUT_BIT_24_11                  5   // Fan 1 Input
  #define   INPUT_BIT_24_12                  6   // Fan 2 Input
  #define   INPUT_BIT_24_13                  7   // Pump 1 Input
  #define   INPUT_BIT_24_14                  8   // Pump 2 Input
  #define   INPUT_BIT_24_15                  9   // Coolant Pumps Input
  #define   INPUT_BIT_24_16                  10  // Coolant Pump Latch
  #define   INPUT_BIT_24_18                  11  // Estop
  #define   INPUT_BIT_24_20                  12  // Amber Button Input
  #define   INPUT_BIT_24_22                  13  // LSR
  #define   INPUT_BIT_24_24                  14  // Coolant Flow Switch
  #define   INPUT_BIT_24_26                  15  // Contamination Sensor S1D
  #define   INPUT_BIT_24_27                  16  // Contamination Sensor S2D
  #define   INPUT_BIT_24_31                  17  // Discharge Flow meter
  #define   INPUT_BIT_24_28                  18  // Comparator
  #define   INPUT_BIT_24_30                  19  // Suction Flow meter
  #define   INPUT_BIT_24_36                  20  // HYD1 Ready
  #define   INPUT_BIT_24_37                  21  // HYD2 Ready
  #define   INPUT_BIT_24_38                  22  // HYD3 Ready

  #define   SWITCH_STATE_ALL                 7   // Digital input switches
  #define   DIP_BIT_0                   0   // Dip Swt 1
  #define   DIP_BIT_1                   1   // Dip Swt 2
  #define   SWITCH_STATE_BLE_NAME       2   // Dip Swt 3  BLE Switch
  #define   SWITCH_STATE_WIFI_DISABLE   3   // Dip Swt 4  Wifi Switch
  #define   DIP_BIT_4                   4   // Dip Swt 5
  #define   DIP_BIT_5                   5   // Dip Swt 6
  #define   DIP_BIT_6                   6   // Dip Swt 7
  #define   DIP_BIT_7                   7   // Dip Swt 8
  #define   DIP_BIT_8                   8   // Dip Swt 9
  #define   DIP_BIT_9                   9   // Dip Swt 10
  #define   DIP_BIT_10                  10  // Dip Swt 11
  #define   DIP_BIT_11                  11  // Dip Swt 12

 
  #define   ERROR_PARAM_ALL                    100
  #define   ERROR_PT405                        10
  #define   ERROR_PT426                        11
  #define   ERROR_PT442                        12
  #define   ERROR_PT410                        13
  #define   ERROR_PT457                        14
  #define   ERROR_PT416                        15
  #define   ERROR_PT468                        16   
  #define   ERROR_PT487                        17   
  #define   ERROR_PT496                        18   
  #define   ERROR_PT531                        19   
  #define   ERROR_PT576                        20   

  #define   ERROR_PARAM2_ALL                   102
  #define   ERROR_TT432                        0
  #define   ERROR_TT439                        1
  #define   ERROR_TT435                        2
  #define   ERROR_TT436                        3
  #define   ERROR_TT447                        4
  #define   ERROR_TT454                        5
  #define   ERROR_TT450                        6
  #define   ERROR_TT451                        7
  #define   ERROR_TT462                        8
  #define   ERROR_TT474                        9
  #define   ERROR_TT465                        10
  #define   ERROR_TT471                        11
  #define   ERROR_TT532                        12
  #define   ERROR_TT557                        13

  #define CONFIG_PARAM_PULSE1  53
  #define CONFIG_PARAM_PULSE2  55
  #define CONFIG_PARAM_PULSE3  57

  #define CONFIG_PARAM_S1A_FLOW_COUNT  2700
  #define CONFIG_PARAM_S1B_FLOW_COUNT  2702
  #define CONFIG_PARAM_S2A_FLOW_COUNT  2730
  #define CONFIG_PARAM_S2B_FLOW_COUNT  2732
  #define CONFIG_PARAM_S3A_FLOW_COUNT  2760
  #define CONFIG_PARAM_S3B_FLOW_COUNT  2762

  #define CONFIG_PARAM_S1_ACTUAL_CPM    2704
  #define CONFIG_PARAM_S1_MIN_CPM       2706
  #define CONFIG_PARAM_S1_MAX_CPM       2708
  #define CONFIG_PARAM_S2_ACTUAL_CPM    2734
  #define CONFIG_PARAM_S2_MIN_CPM       2736
  #define CONFIG_PARAM_S2_MAX_CPM       2738
  #define CONFIG_PARAM_S3_ACTUAL_CPM    2764
  #define CONFIG_PARAM_S3_MIN_CPM       2766
  #define CONFIG_PARAM_S3_MAX_CPM       2768


  #define   CONFIG_PARAM_PT442_RATIO_LOW_PSI                    2790  //read only
  #define   CONFIG_PARAM_PT442_RATIO_HIGH_PSI                   2792  //read only
  #define   CONFIG_PARAM_PT457_TARGET_PSI                       2794    //read only
  #define   CONFIG_PARAM_PT487_TARGET_PSI                       2796  //read only


  #define CONFIG_PARAM_COOLANT1_PT      21  //PT531
  #define CONFIG_PARAM_COOLANT2_PT      22  //PT576
  #define CONFIG_PARAM_SUCTION_PT        9  //PT405
  #define CONFIG_PARAM_SUCTION_TANK     10  //PT426
  #define CONFIG_PARAM_STAGE1_TANK      11  //PT442
  #define CONFIG_PARAM_STAGE1_PT        12  //PT410 - Reserved Currently Unused
  #define CONFIG_PARAM_STAGE2_TANK      13  //PT457
  #define CONFIG_PARAM_STAGE2_PT        14  //PT416 - Reserved Currently Unused
  #define CONFIG_PARAM_STAGE3_TANK      15  //PT468
  #define CONFIG_PARAM_STAGE3_PT        16  //PT487
  #define CONFIG_PARAM_HFL_PUMP1        17  //PT496


  #define CONFIG_PARAM_TC1  41
  #define CONFIG_PARAM_TC2  42
  #define CONFIG_PARAM_TC3  29  //TT432
  #define CONFIG_PARAM_TC4  30  //TT439
  #define CONFIG_PARAM_TC5  31  //TT435
  #define CONFIG_PARAM_TC6  32  //TT436
  #define CONFIG_PARAM_TC7  33  //TT447
  #define CONFIG_PARAM_TC8  34  //TC454
  #define CONFIG_PARAM_TC9  35  //TC450
  #define CONFIG_PARAM_TC10  36  //TC451
  #define CONFIG_PARAM_TC11  37  //TC462
  #define CONFIG_PARAM_TC12  38
  #define CONFIG_PARAM_TC13  39
  #define CONFIG_PARAM_TC14  40

    //PT405
    #define   CONFIG_PARAM_PT405_STOP_MIN         2039
    #define   CONFIG_PARAM_PT405_IDLE_MIN         2041
    #define   CONFIG_PARAM_PT405_RUN_MIN          2043
    #define   CONFIG_PARAM_PT405_RUN_MAX          2045
    #define   CONFIG_PARAM_PT405_IDLE_MAX         2047
    #define   CONFIG_PARAM_PT405_STOP_MAX         2049   
    #define   SETPOINT_PT405_ENTER_STOP_MIN       1000  
    #define   SETPOINT_PT405_ENTER_IDLE_MIN       1000   
    #define   SETPOINT_PT405_ENTER_RUN_MIN        1000   
    #define   SETPOINT_PT405_ENTER_RUN_MAX        1000
    #define   SETPOINT_PT405_ENTER_IDLE_MAX       1000  
    #define   SETPOINT_PT405_ENTER_STOP_MAX       1000

    //PT426 
    #define   CONFIG_PARAM_PT426_STOP_MIN         2051
    #define   CONFIG_PARAM_PT426_IDLE_MIN         2053
    #define   CONFIG_PARAM_PT426_RUN_MIN          2055
    #define   CONFIG_PARAM_PT426_RUN_MAX          2057
    #define   CONFIG_PARAM_PT426_IDLE_MAX         2059
    #define   CONFIG_PARAM_PT426_STOP_MAX         2061   
    #define   SETPOINT_PT426_ENTER_STOP_MIN       2000
    #define   SETPOINT_PT426_ENTER_IDLE_MIN       2000 
    #define   SETPOINT_PT426_ENTER_RUN_MIN        2000   
    #define   SETPOINT_PT426_ENTER_RUN_MAX        2000
    #define   SETPOINT_PT426_ENTER_IDLE_MAX       2000  
    #define   SETPOINT_PT426_ENTER_STOP_MAX       2000

    //PT442
    #define   CONFIG_PARAM_PT442_STOP_MIN         2063
    #define   CONFIG_PARAM_PT442_IDLE_MIN         2065
    #define   CONFIG_PARAM_PT442_RUN_MIN          2067
    #define   CONFIG_PARAM_PT442_RUN_MAX          2069
    #define   CONFIG_PARAM_PT442_IDLE_MAX         2071
    #define   CONFIG_PARAM_PT442_STOP_MAX         2073   
    #define   SETPOINT_PT442_ENTER_STOP_MIN       3000
    #define   SETPOINT_PT442_ENTER_IDLE_MIN       3000 
    #define   SETPOINT_PT442_ENTER_RUN_MIN        3000   
    #define   SETPOINT_PT442_ENTER_RUN_MAX        3000
    #define   SETPOINT_PT442_ENTER_IDLE_MAX       3000  
    #define   SETPOINT_PT442_ENTER_STOP_MAX       3000

    //PT410
    #define   CONFIG_PARAM_PT410_STOP_MIN         2075
    #define   CONFIG_PARAM_PT410_IDLE_MIN         2077
    #define   CONFIG_PARAM_PT410_RUN_MIN          2079
    #define   CONFIG_PARAM_PT410_RUN_MAX          2081
    #define   CONFIG_PARAM_PT410_IDLE_MAX         2083
    #define   CONFIG_PARAM_PT410_STOP_MAX         2085   
    #define   SETPOINT_PT410_ENTER_STOP_MIN       4000
    #define   SETPOINT_PT410_ENTER_IDLE_MIN       4000 
    #define   SETPOINT_PT410_ENTER_RUN_MIN        4000   
    #define   SETPOINT_PT410_ENTER_RUN_MAX        4000
    #define   SETPOINT_PT410_ENTER_IDLE_MAX       4000  
    #define   SETPOINT_PT410_ENTER_STOP_MAX       4000

    //PT457
    #define   CONFIG_PARAM_PT457_STOP_MIN         2087
    #define   CONFIG_PARAM_PT457_IDLE_MIN         2089
    #define   CONFIG_PARAM_PT457_RUN_MIN          2091
    #define   CONFIG_PARAM_PT457_RUN_MAX          2093
    #define   CONFIG_PARAM_PT457_IDLE_MAX         2095
    #define   CONFIG_PARAM_PT457_STOP_MAX         2097   
    #define   SETPOINT_PT457_ENTER_STOP_MIN       5000
    #define   SETPOINT_PT457_ENTER_IDLE_MIN       5000 
    #define   SETPOINT_PT457_ENTER_RUN_MIN        5000   
    #define   SETPOINT_PT457_ENTER_RUN_MAX        5000
    #define   SETPOINT_PT457_ENTER_IDLE_MAX       5000  
    #define   SETPOINT_PT457_ENTER_STOP_MAX       5000

    //PT416
    #define   CONFIG_PARAM_PT416_STOP_MIN         2099
    #define   CONFIG_PARAM_PT416_IDLE_MIN         2101
    #define   CONFIG_PARAM_PT416_RUN_MIN          2103
    #define   CONFIG_PARAM_PT416_RUN_MAX          2105
    #define   CONFIG_PARAM_PT416_IDLE_MAX         2107
    #define   CONFIG_PARAM_PT416_STOP_MAX         2109   
    #define   SETPOINT_PT416_ENTER_STOP_MIN       6000
    #define   SETPOINT_PT416_ENTER_IDLE_MIN       6000 
    #define   SETPOINT_PT416_ENTER_RUN_MIN        6000   
    #define   SETPOINT_PT416_ENTER_RUN_MAX        6000
    #define   SETPOINT_PT416_ENTER_IDLE_MAX       6000  
    #define   SETPOINT_PT416_ENTER_STOP_MAX       6000

    //PT468
    #define   CONFIG_PARAM_PT468_STOP_MIN         2111
    #define   CONFIG_PARAM_PT468_IDLE_MIN         2113
    #define   CONFIG_PARAM_PT468_RUN_MIN          2115
    #define   CONFIG_PARAM_PT468_RUN_MAX          2117
    #define   CONFIG_PARAM_PT468_IDLE_MAX         2119
    #define   CONFIG_PARAM_PT468_STOP_MAX         2121   
    #define   SETPOINT_PT468_ENTER_STOP_MIN       7000
    #define   SETPOINT_PT468_ENTER_IDLE_MIN       7000 
    #define   SETPOINT_PT468_ENTER_RUN_MIN        7000   
    #define   SETPOINT_PT468_ENTER_RUN_MAX        7000
    #define   SETPOINT_PT468_ENTER_IDLE_MAX       7000  
    #define   SETPOINT_PT468_ENTER_STOP_MAX       7000

    //PT496
    #define   CONFIG_PARAM_PT496_STOP_MIN         2123
    #define   CONFIG_PARAM_PT496_IDLE_MIN         2125
    #define   CONFIG_PARAM_PT496_RUN_MIN          2127
    #define   CONFIG_PARAM_PT496_RUN_MAX          2129
    #define   CONFIG_PARAM_PT496_IDLE_MAX         2131
    #define   CONFIG_PARAM_PT496_STOP_MAX         2133   
    #define   SETPOINT_PT496_ENTER_STOP_MIN       10000
    #define   SETPOINT_PT496_ENTER_IDLE_MIN       10000 
    #define   SETPOINT_PT496_ENTER_RUN_MIN        10000   
    #define   SETPOINT_PT496_ENTER_RUN_MAX        10000
    #define   SETPOINT_PT496_ENTER_IDLE_MAX       10000  
    #define   SETPOINT_PT496_ENTER_STOP_MAX       10000

    //PT531
    #define   CONFIG_PARAM_PT531_STOP_MIN         2135
    #define   CONFIG_PARAM_PT531_IDLE_MIN         2137
    #define   CONFIG_PARAM_PT531_RUN_MIN          2139
    #define   CONFIG_PARAM_PT531_RUN_MAX          2141
    #define   CONFIG_PARAM_PT531_IDLE_MAX         2143
    #define   CONFIG_PARAM_PT531_STOP_MAX         2145   
    #define   SETPOINT_PT531_ENTER_STOP_MIN       8000
    #define   SETPOINT_PT531_ENTER_IDLE_MIN       8000 
    #define   SETPOINT_PT531_ENTER_RUN_MIN        8000   
    #define   SETPOINT_PT531_ENTER_RUN_MAX        8000
    #define   SETPOINT_PT531_ENTER_IDLE_MAX       8000  
    #define   SETPOINT_PT531_ENTER_STOP_MAX       8000

    //PT576
    #define   CONFIG_PARAM_PT576_STOP_MIN         2147
    #define   CONFIG_PARAM_PT576_IDLE_MIN         2149
    #define   CONFIG_PARAM_PT576_RUN_MIN          2151
    #define   CONFIG_PARAM_PT576_RUN_MAX          2153
    #define   CONFIG_PARAM_PT576_IDLE_MAX         2155
    #define   CONFIG_PARAM_PT576_STOP_MAX         2157   
    #define   SETPOINT_PT576_ENTER_STOP_MIN       9000
    #define   SETPOINT_PT576_ENTER_IDLE_MIN       9000 
    #define   SETPOINT_PT576_ENTER_RUN_MIN        9000   
    #define   SETPOINT_PT576_ENTER_RUN_MAX        9000
    #define   SETPOINT_PT576_ENTER_IDLE_MAX       9000  
    #define   SETPOINT_PT576_ENTER_STOP_MAX       9000

    //PT487
    #define   CONFIG_PARAM_PT487_STOP_MIN         2159
    #define   CONFIG_PARAM_PT487_IDLE_MIN         2161
    #define   CONFIG_PARAM_PT487_RUN_MIN          2163
    #define   CONFIG_PARAM_PT487_RUN_MAX          2165
    #define   CONFIG_PARAM_PT487_IDLE_MAX         2167
    #define   CONFIG_PARAM_PT487_STOP_MAX         2169   
    #define   SETPOINT_PT487_ENTER_STOP_MIN       11000
    #define   SETPOINT_PT487_ENTER_IDLE_MIN       11000 
    #define   SETPOINT_PT487_ENTER_RUN_MIN        11000   
    #define   SETPOINT_PT487_ENTER_RUN_MAX        11000
    #define   SETPOINT_PT487_ENTER_IDLE_MAX       11000  
    #define   SETPOINT_PT487_ENTER_STOP_MAX       11000

    //TC3-TT432
    #define   CONFIG_PARAM_TT432_STOP_MIN         2299
    #define   CONFIG_PARAM_TT432_IDLE_MIN         2301
    #define   CONFIG_PARAM_TT432_RUN_MIN          2303
    #define   CONFIG_PARAM_TT432_RUN_MAX          2305
    #define   CONFIG_PARAM_TT432_IDLE_MAX         2307
    #define   CONFIG_PARAM_TT432_STOP_MAX         2309   
    #define   SETPOINT_TT432_ENTER_STOP_MIN       25
    #define   SETPOINT_TT432_ENTER_IDLE_MIN       25 
    #define   SETPOINT_TT432_ENTER_RUN_MIN        25   
    #define   SETPOINT_TT432_ENTER_RUN_MAX        25
    #define   SETPOINT_TT432_ENTER_IDLE_MAX       25  
    #define   SETPOINT_TT432_ENTER_STOP_MAX       25

     //TC4-TT439
    #define   CONFIG_PARAM_TT439_STOP_MIN         2311
    #define   CONFIG_PARAM_TT439_IDLE_MIN         2313
    #define   CONFIG_PARAM_TT439_RUN_MIN          2315
    #define   CONFIG_PARAM_TT439_RUN_MAX          2317
    #define   CONFIG_PARAM_TT439_IDLE_MAX         2319
    #define   CONFIG_PARAM_TT439_STOP_MAX         2321   
    #define   SETPOINT_TT439_ENTER_STOP_MIN       23
    #define   SETPOINT_TT439_ENTER_IDLE_MIN       23 
    #define   SETPOINT_TT439_ENTER_RUN_MIN        23   
    #define   SETPOINT_TT439_ENTER_RUN_MAX        26
    #define   SETPOINT_TT439_ENTER_IDLE_MAX       26  
    #define   SETPOINT_TT439_ENTER_STOP_MAX       26

    //TC5-TT435
    #define   CONFIG_PARAM_TT435_STOP_MIN         2323
    #define   CONFIG_PARAM_TT435_IDLE_MIN         2325
    #define   CONFIG_PARAM_TT435_RUN_MIN          2327
    #define   CONFIG_PARAM_TT435_RUN_MAX          2329
    #define   CONFIG_PARAM_TT435_IDLE_MAX         2331
    #define   CONFIG_PARAM_TT435_STOP_MAX         2333   
    #define   SETPOINT_TT435_ENTER_STOP_MIN       22
    #define   SETPOINT_TT435_ENTER_IDLE_MIN       22 
    #define   SETPOINT_TT435_ENTER_RUN_MIN        22   
    #define   SETPOINT_TT435_ENTER_RUN_MAX        27
    #define   SETPOINT_TT435_ENTER_IDLE_MAX       27  
    #define   SETPOINT_TT435_ENTER_STOP_MAX       27

    //TC6-TT436
    #define   CONFIG_PARAM_TT436_STOP_MIN         2335
    #define   CONFIG_PARAM_TT436_IDLE_MIN         2337
    #define   CONFIG_PARAM_TT436_RUN_MIN          2339
    #define   CONFIG_PARAM_TT436_RUN_MAX          2341
    #define   CONFIG_PARAM_TT436_IDLE_MAX         2343
    #define   CONFIG_PARAM_TT436_STOP_MAX         2345   
    #define   SETPOINT_TT436_ENTER_STOP_MIN       22
    #define   SETPOINT_TT436_ENTER_IDLE_MIN       23 
    #define   SETPOINT_TT436_ENTER_RUN_MIN        24   
    #define   SETPOINT_TT436_ENTER_RUN_MAX        27
    #define   SETPOINT_TT436_ENTER_IDLE_MAX       28  
    #define   SETPOINT_TT436_ENTER_STOP_MAX       29

     //TC7-TT447
    #define   CONFIG_PARAM_TT447_STOP_MIN         2347
    #define   CONFIG_PARAM_TT447_IDLE_MIN         2349
    #define   CONFIG_PARAM_TT447_RUN_MIN          2351
    #define   CONFIG_PARAM_TT447_RUN_MAX          2353
    #define   CONFIG_PARAM_TT447_IDLE_MAX         2355
    #define   CONFIG_PARAM_TT447_STOP_MAX         2357   
    #define   SETPOINT_TT447_ENTER_STOP_MIN       22
    #define   SETPOINT_TT447_ENTER_IDLE_MIN       23 
    #define   SETPOINT_TT447_ENTER_RUN_MIN        24   
    #define   SETPOINT_TT447_ENTER_RUN_MAX        27
    #define   SETPOINT_TT447_ENTER_IDLE_MAX       28  
    #define   SETPOINT_TT447_ENTER_STOP_MAX       29

    //TC8-TT454
    #define   CONFIG_PARAM_TT454_STOP_MIN         2359
    #define   CONFIG_PARAM_TT454_IDLE_MIN         2361
    #define   CONFIG_PARAM_TT454_RUN_MIN          2363
    #define   CONFIG_PARAM_TT454_RUN_MAX          2365
    #define   CONFIG_PARAM_TT454_IDLE_MAX         2367
    #define   CONFIG_PARAM_TT454_STOP_MAX         2369   
    #define   SETPOINT_TT454_ENTER_STOP_MIN       22
    #define   SETPOINT_TT454_ENTER_IDLE_MIN       23 
    #define   SETPOINT_TT454_ENTER_RUN_MIN        24   
    #define   SETPOINT_TT454_ENTER_RUN_MAX        27
    #define   SETPOINT_TT454_ENTER_IDLE_MAX       28  
    #define   SETPOINT_TT454_ENTER_STOP_MAX       29

  //TC9-TT450
    #define   CONFIG_PARAM_TT450_STOP_MIN         2371
    #define   CONFIG_PARAM_TT450_IDLE_MIN         2373
    #define   CONFIG_PARAM_TT450_RUN_MIN          2375
    #define   CONFIG_PARAM_TT450_RUN_MAX          2377
    #define   CONFIG_PARAM_TT450_IDLE_MAX         2379
    #define   CONFIG_PARAM_TT450_STOP_MAX         2381   
    #define   SETPOINT_TT450_ENTER_STOP_MIN       22
    #define   SETPOINT_TT450_ENTER_IDLE_MIN       23 
    #define   SETPOINT_TT450_ENTER_RUN_MIN        24   
    #define   SETPOINT_TT450_ENTER_RUN_MAX        27
    #define   SETPOINT_TT450_ENTER_IDLE_MAX       28  
    #define   SETPOINT_TT450_ENTER_STOP_MAX       29

  //TC10-TT451
    #define   CONFIG_PARAM_TT451_STOP_MIN         2383
    #define   CONFIG_PARAM_TT451_IDLE_MIN         2385
    #define   CONFIG_PARAM_TT451_RUN_MIN          2387
    #define   CONFIG_PARAM_TT451_RUN_MAX          2389
    #define   CONFIG_PARAM_TT451_IDLE_MAX         2391
    #define   CONFIG_PARAM_TT451_STOP_MAX         2393   
    #define   SETPOINT_TT451_ENTER_STOP_MIN       22
    #define   SETPOINT_TT451_ENTER_IDLE_MIN       23 
    #define   SETPOINT_TT451_ENTER_RUN_MIN        24   
    #define   SETPOINT_TT451_ENTER_RUN_MAX        27
    #define   SETPOINT_TT451_ENTER_IDLE_MAX       28  
    #define   SETPOINT_TT451_ENTER_STOP_MAX       29

    //TC11-TT462
    #define   CONFIG_PARAM_TT462_STOP_MIN         2395
    #define   CONFIG_PARAM_TT462_IDLE_MIN         2397
    #define   CONFIG_PARAM_TT462_RUN_MIN          2399
    #define   CONFIG_PARAM_TT462_RUN_MAX          2401
    #define   CONFIG_PARAM_TT462_IDLE_MAX         2403
    #define   CONFIG_PARAM_TT462_STOP_MAX         2405   
    #define   SETPOINT_TT462_ENTER_STOP_MIN       22
    #define   SETPOINT_TT462_ENTER_IDLE_MIN       23 
    #define   SETPOINT_TT462_ENTER_RUN_MIN        24   
    #define   SETPOINT_TT462_ENTER_RUN_MAX        27
    #define   SETPOINT_TT462_ENTER_IDLE_MAX       28  
    #define   SETPOINT_TT462_ENTER_STOP_MAX       29

    //TC12-TT474
    #define   CONFIG_PARAM_TT474_STOP_MIN         2407
    #define   CONFIG_PARAM_TT474_IDLE_MIN         2409
    #define   CONFIG_PARAM_TT474_RUN_MIN          2411
    #define   CONFIG_PARAM_TT474_RUN_MAX          2413
    #define   CONFIG_PARAM_TT474_IDLE_MAX         2415
    #define   CONFIG_PARAM_TT474_STOP_MAX         2417   
    #define   SETPOINT_TT474_ENTER_STOP_MIN       22
    #define   SETPOINT_TT474_ENTER_IDLE_MIN       23 
    #define   SETPOINT_TT474_ENTER_RUN_MIN        24   
    #define   SETPOINT_TT474_ENTER_RUN_MAX        27
    #define   SETPOINT_TT474_ENTER_IDLE_MAX       28  
    #define   SETPOINT_TT474_ENTER_STOP_MAX       29

    //TC13-TT465
    #define   CONFIG_PARAM_TT465_STOP_MIN         2419
    #define   CONFIG_PARAM_TT465_IDLE_MIN         2421
    #define   CONFIG_PARAM_TT465_RUN_MIN          2423
    #define   CONFIG_PARAM_TT465_RUN_MAX          2425
    #define   CONFIG_PARAM_TT465_IDLE_MAX         2427
    #define   CONFIG_PARAM_TT465_STOP_MAX         2429   
    #define   SETPOINT_TT465_ENTER_STOP_MIN       22
    #define   SETPOINT_TT465_ENTER_IDLE_MIN       23 
    #define   SETPOINT_TT465_ENTER_RUN_MIN        24   
    #define   SETPOINT_TT465_ENTER_RUN_MAX        27
    #define   SETPOINT_TT465_ENTER_IDLE_MAX       28  
    #define   SETPOINT_TT465_ENTER_STOP_MAX       29

    //TC14-TT471
    #define   CONFIG_PARAM_TT471_STOP_MIN         2431
    #define   CONFIG_PARAM_TT471_IDLE_MIN         2433
    #define   CONFIG_PARAM_TT471_RUN_MIN          2435
    #define   CONFIG_PARAM_TT471_RUN_MAX          2437
    #define   CONFIG_PARAM_TT471_IDLE_MAX         2439
    #define   CONFIG_PARAM_TT471_STOP_MAX         2441   
    #define   SETPOINT_TT471_ENTER_STOP_MIN       22
    #define   SETPOINT_TT471_ENTER_IDLE_MIN       23 
    #define   SETPOINT_TT471_ENTER_RUN_MIN        24   
    #define   SETPOINT_TT471_ENTER_RUN_MAX        27
    #define   SETPOINT_TT471_ENTER_IDLE_MAX       28  
    #define   SETPOINT_TT471_ENTER_STOP_MAX       29

    //TC1-TT532
    #define   CONFIG_PARAM_TT532_STOP_MIN         2443
    #define   CONFIG_PARAM_TT532_IDLE_MIN         2445
    #define   CONFIG_PARAM_TT532_RUN_MIN          2447
    #define   CONFIG_PARAM_TT532_RUN_MAX          2449
    #define   CONFIG_PARAM_TT532_IDLE_MAX         2451
    #define   CONFIG_PARAM_TT532_STOP_MAX         2453   
    #define   SETPOINT_TT532_ENTER_STOP_MIN       22
    #define   SETPOINT_TT532_ENTER_IDLE_MIN       23 
    #define   SETPOINT_TT532_ENTER_RUN_MIN        24   
    #define   SETPOINT_TT532_ENTER_RUN_MAX        27
    #define   SETPOINT_TT532_ENTER_IDLE_MAX       28  
    #define   SETPOINT_TT532_ENTER_STOP_MAX       29  

    //TC2-TT557
    #define   CONFIG_PARAM_TT557_STOP_MIN         2455
    #define   CONFIG_PARAM_TT557_IDLE_MIN         2457
    #define   CONFIG_PARAM_TT557_RUN_MIN          2459
    #define   CONFIG_PARAM_TT557_RUN_MAX          2461
    #define   CONFIG_PARAM_TT557_IDLE_MAX         2463
    #define   CONFIG_PARAM_TT557_STOP_MAX         2465   
    #define   SETPOINT_TT557_ENTER_STOP_MIN       22
    #define   SETPOINT_TT557_ENTER_IDLE_MIN       23 
    #define   SETPOINT_TT557_ENTER_RUN_MIN        24   
    #define   SETPOINT_TT557_ENTER_RUN_MAX        27
    #define   SETPOINT_TT557_ENTER_IDLE_MAX       28  
    #define   SETPOINT_TT557_ENTER_STOP_MAX       29  

  #define   IDLE_STATE_S1_ALL                   108
  #define   IDLE_STATE_S2_ALL                   112
  #define   IDLE_STATE_S3_ALL                   116
  #define   IDLE_STATE_PT405                    0
  #define   IDLE_STATE_PT426                    1
  #define   IDLE_STATE_PT442                    2
  #define   IDLE_STATE_PT410                    3
  #define   IDLE_STATE_PT457                    4
  #define   IDLE_STATE_PT416                    5
  #define   IDLE_STATE_PT468                    6
  #define   IDLE_STATE_PT487                    7
  #define   IDLE_STATE_PT496                    8
  #define   IDLE_STATE_PT531                    9
  #define   IDLE_STATE_PT576                    10
  #define   IDLE_STATE_TT432                    11
  #define   IDLE_STATE_TT439                    12
  #define   IDLE_STATE_TT435                    13
  #define   IDLE_STATE_TT436                    14
  #define   IDLE_STATE_TT447                    15
  #define   IDLE_STATE_TT454                    16
  #define   IDLE_STATE_TT450                    17
  #define   IDLE_STATE_TT451                    18
  #define   IDLE_STATE_TT462                    19
  #define   IDLE_STATE_TT474                    20
  #define   IDLE_STATE_TT465                    21
  #define   IDLE_STATE_TT471                    22
  #define   IDLE_STATE_TT532                    23
  #define   IDLE_STATE_TT557                    24

  #define   CONFIG_PARAM_PV1_CL                 81     //Analog output 1 0-10VDC Proportional valve 1 TB57.1 (PCB7.10 and later ONLY)
  #define   CONFIG_PARAM_PV2_CL                 83     //Analog output 2 0-10VDC Proportional valve 2 TB57.3 (PCB7.10 and later ONLY)
  #define   CONFIG_PARAM_PV3_CL                 85     //Analog output 3 0-10VDC Proportional valve 3 TB57.5 (PCB7.10 and later ONLY)
  #define   CONFIG_PARAM_PV4_CL                 87     //Analog output 4 0-10VDC Proportional valve 4 TB57.7 (PCB7.10 and later ONLY)

  #define   FLAG_CLEAR                          0
  #define   FLAG_ACTIVE                         1

  void init_config(void);
  float get_config_parameter(int);
  bool test_config_parameter(int, int);
  uint16_t get_config_parameter(int, int);
  void set_config_parameter(int, float);
  void set_config_parameter(int, uint16_t, int);
  void set_config_bit(int, uint16_t, int);

#endif
