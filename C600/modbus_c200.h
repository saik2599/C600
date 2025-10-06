/*
   Author: Christopher Glanville
   modbus_c200.h configuration header file. To acompany modbus_c200.cpp
   Date: 26/5/24
   updated : 26/5/24
*/
#if !defined(_MODBUS_C200_H)
  #define  _MODBUS_C200_H

  #define     TCP_BUFFER      1 
  #define     RS_485_BUFFER   2
  #define     BLE_BUFFER      3

  #define MODBUS_REG_RELAYS                 1 
  #define MODBUS_REG_INPUTS                 3
  #define MODBUS_REG_SWITCH_ALL             7
  #define MODBUS_REG_OP_STATE               104 

  #define MODBUS_REG_IDLE_STATE_S1_ALL               108 
  #define MODBUS_REG_IDLE_STATE_S2_ALL               112 
  #define MODBUS_REG_IDLE_STATE_S3_ALL               116 

  #define MODBUS_REG_PULSE1  53
  #define MODBUS_REG_PULSE2  55
  #define MODBUS_REG_PULSE3  57

  #define MODBUS_REG_ERROR_PARAM    100
  #define MODBUS_REG_ERROR2_PARAM   102

  #define MODBUS_REG_COOLANT1_PT      21  //PT531
  #define MODBUS_REG_COOLANT2_PT      22  //PT576
  #define MODBUS_REG_SUCTION_PT        9  //PT405
  #define MODBUS_REG_SUCTION_TANK     10  //PT426
  #define MODBUS_REG_STAGE1_TANK      11  //PT442
  #define MODBUS_REG_STAGE1_PT        12  //PT410 - Reserved Currently Unused
  #define MODBUS_REG_STAGE2_TANK      13  //PT457
  #define MODBUS_REG_STAGE2_PT        14  //PT416 - Reserved Currently Unused
  #define MODBUS_REG_STAGE3_TANK      15  //PT468
  #define MODBUS_REG_STAGE3_PT        16  //PT487
  #define MODBUS_REG_HFL_PUMP1        17  //PT496

  #define MODBUS_REG_TC1  41
  #define MODBUS_REG_TC2  42
  #define MODBUS_REG_TC3  29  //TT432
  #define MODBUS_REG_TC4  30
  #define MODBUS_REG_TC5  31
  #define MODBUS_REG_TC6  32
  #define MODBUS_REG_TC7  33
  #define MODBUS_REG_TC8  34
  #define MODBUS_REG_TC9  35
  #define MODBUS_REG_TC10  36
  #define MODBUS_REG_TC11  37
  #define MODBUS_REG_TC12  38
  #define MODBUS_REG_TC13  39
  #define MODBUS_REG_TC14  40

  #define MODBUS_REG_SLAVE_ID          2001 
  #define MODBUS_REG_SERIAL_NUM        2010

  #define MODBUS_REG_PT405_STOP_MIN      2039
  #define MODBUS_REG_PT405_IDLE_MIN      2041
  #define MODBUS_REG_PT405_RUN_MIN       2043
  #define MODBUS_REG_PT405_RUN_MAX       2045
  #define MODBUS_REG_PT405_IDLE_MAX      2047
  #define MODBUS_REG_PT405_STOP_MAX      2049 

  #define MODBUS_REG_PT426_STOP_MIN      2051
  #define MODBUS_REG_PT426_IDLE_MIN      2053
  #define MODBUS_REG_PT426_RUN_MIN       2055
  #define MODBUS_REG_PT426_RUN_MAX       2057
  #define MODBUS_REG_PT426_IDLE_MAX      2059
  #define MODBUS_REG_PT426_STOP_MAX      2061

  #define MODBUS_REG_PT442_STOP_MIN      2063
  #define MODBUS_REG_PT442_IDLE_MIN      2065
  #define MODBUS_REG_PT442_RUN_MIN       2067
  #define MODBUS_REG_PT442_RUN_MAX       2069
  #define MODBUS_REG_PT442_IDLE_MAX      2071
  #define MODBUS_REG_PT442_STOP_MAX      2073 

  #define MODBUS_REG_PT410_STOP_MIN      2075
  #define MODBUS_REG_PT410_IDLE_MIN      2077
  #define MODBUS_REG_PT410_RUN_MIN       2079
  #define MODBUS_REG_PT410_RUN_MAX       2081
  #define MODBUS_REG_PT410_IDLE_MAX      2083
  #define MODBUS_REG_PT410_STOP_MAX      2085

  #define MODBUS_REG_PT457_STOP_MIN      2087
  #define MODBUS_REG_PT457_IDLE_MIN      2089
  #define MODBUS_REG_PT457_RUN_MIN       2091
  #define MODBUS_REG_PT457_RUN_MAX       2093
  #define MODBUS_REG_PT457_IDLE_MAX      2095
  #define MODBUS_REG_PT457_STOP_MAX      2097 

  #define MODBUS_REG_PT416_STOP_MIN      2099
  #define MODBUS_REG_PT416_IDLE_MIN      2101
  #define MODBUS_REG_PT416_RUN_MIN       2103
  #define MODBUS_REG_PT416_RUN_MAX       2105
  #define MODBUS_REG_PT416_IDLE_MAX      2107
  #define MODBUS_REG_PT416_STOP_MAX      2109 

  #define MODBUS_REG_PT468_STOP_MIN      2111
  #define MODBUS_REG_PT468_IDLE_MIN      2113
  #define MODBUS_REG_PT468_RUN_MIN       2115
  #define MODBUS_REG_PT468_RUN_MAX       2117
  #define MODBUS_REG_PT468_IDLE_MAX      2119
  #define MODBUS_REG_PT468_STOP_MAX      2121 

  #define MODBUS_REG_PT496_STOP_MIN      2123
  #define MODBUS_REG_PT496_IDLE_MIN      2125
  #define MODBUS_REG_PT496_RUN_MIN       2127
  #define MODBUS_REG_PT496_RUN_MAX       2129
  #define MODBUS_REG_PT496_IDLE_MAX      2131
  #define MODBUS_REG_PT496_STOP_MAX      2133 

  #define MODBUS_REG_PT531_STOP_MIN      2135
  #define MODBUS_REG_PT531_IDLE_MIN      2137
  #define MODBUS_REG_PT531_RUN_MIN       2139
  #define MODBUS_REG_PT531_RUN_MAX       2141
  #define MODBUS_REG_PT531_IDLE_MAX      2143
  #define MODBUS_REG_PT531_STOP_MAX      2145 

  #define MODBUS_REG_PT576_STOP_MIN      2147
  #define MODBUS_REG_PT576_IDLE_MIN      2149
  #define MODBUS_REG_PT576_RUN_MIN       2151
  #define MODBUS_REG_PT576_RUN_MAX       2153
  #define MODBUS_REG_PT576_IDLE_MAX      2155
  #define MODBUS_REG_PT576_STOP_MAX      2157 

  #define MODBUS_REG_PT487_STOP_MIN      2159
  #define MODBUS_REG_PT487_IDLE_MIN      2161
  #define MODBUS_REG_PT487_RUN_MIN       2163
  #define MODBUS_REG_PT487_RUN_MAX       2165
  #define MODBUS_REG_PT487_IDLE_MAX      2167
  #define MODBUS_REG_PT487_STOP_MAX      2169 

  #define MODBUS_REG_TT432_STOP_MIN      2299
  #define MODBUS_REG_TT432_IDLE_MIN      2301
  #define MODBUS_REG_TT432_RUN_MIN       2303
  #define MODBUS_REG_TT432_RUN_MAX       2305
  #define MODBUS_REG_TT432_IDLE_MAX      2307
  #define MODBUS_REG_TT432_STOP_MAX      2309 

  #define MODBUS_REG_TT439_STOP_MIN      2311
  #define MODBUS_REG_TT439_IDLE_MIN      2313
  #define MODBUS_REG_TT439_RUN_MIN       2315
  #define MODBUS_REG_TT439_RUN_MAX       2317
  #define MODBUS_REG_TT439_IDLE_MAX      2319
  #define MODBUS_REG_TT439_STOP_MAX      2321 

  #define MODBUS_REG_TT435_STOP_MIN      2323
  #define MODBUS_REG_TT435_IDLE_MIN      2325
  #define MODBUS_REG_TT435_RUN_MIN       2327
  #define MODBUS_REG_TT435_RUN_MAX       2329
  #define MODBUS_REG_TT435_IDLE_MAX      2331
  #define MODBUS_REG_TT435_STOP_MAX      2333 

  #define MODBUS_REG_TT436_STOP_MIN      2335
  #define MODBUS_REG_TT436_IDLE_MIN      2337
  #define MODBUS_REG_TT436_RUN_MIN       2339
  #define MODBUS_REG_TT436_RUN_MAX       2341
  #define MODBUS_REG_TT436_IDLE_MAX      2343
  #define MODBUS_REG_TT436_STOP_MAX      2345 

  #define MODBUS_REG_TT447_STOP_MIN      2347
  #define MODBUS_REG_TT447_IDLE_MIN      2349
  #define MODBUS_REG_TT447_RUN_MIN       2351
  #define MODBUS_REG_TT447_RUN_MAX       2353
  #define MODBUS_REG_TT447_IDLE_MAX      2355
  #define MODBUS_REG_TT447_STOP_MAX      2357 

  #define MODBUS_REG_TT454_STOP_MIN      2359
  #define MODBUS_REG_TT454_IDLE_MIN      2361
  #define MODBUS_REG_TT454_RUN_MIN       2363
  #define MODBUS_REG_TT454_RUN_MAX       2365
  #define MODBUS_REG_TT454_IDLE_MAX      2367
  #define MODBUS_REG_TT454_STOP_MAX      2369 

  #define MODBUS_REG_TT450_STOP_MIN      2371
  #define MODBUS_REG_TT450_IDLE_MIN      2373
  #define MODBUS_REG_TT450_RUN_MIN       2375
  #define MODBUS_REG_TT450_RUN_MAX       2377
  #define MODBUS_REG_TT450_IDLE_MAX      2379
  #define MODBUS_REG_TT450_STOP_MAX      2381

  #define MODBUS_REG_TT451_STOP_MIN      2383
  #define MODBUS_REG_TT451_IDLE_MIN      2385
  #define MODBUS_REG_TT451_RUN_MIN       2387
  #define MODBUS_REG_TT451_RUN_MAX       2389
  #define MODBUS_REG_TT451_IDLE_MAX      2391
  #define MODBUS_REG_TT451_STOP_MAX      2393

  #define MODBUS_REG_TT462_STOP_MIN      2395
  #define MODBUS_REG_TT462_IDLE_MIN      2397
  #define MODBUS_REG_TT462_RUN_MIN       2399
  #define MODBUS_REG_TT462_RUN_MAX       2401
  #define MODBUS_REG_TT462_IDLE_MAX      2403
  #define MODBUS_REG_TT462_STOP_MAX      2405

  #define MODBUS_REG_TT474_STOP_MIN      2407
  #define MODBUS_REG_TT474_IDLE_MIN      2409
  #define MODBUS_REG_TT474_RUN_MIN       2411
  #define MODBUS_REG_TT474_RUN_MAX       2413
  #define MODBUS_REG_TT474_IDLE_MAX      2415
  #define MODBUS_REG_TT474_STOP_MAX      2417

  #define MODBUS_REG_TT465_STOP_MIN      2419
  #define MODBUS_REG_TT465_IDLE_MIN      2421
  #define MODBUS_REG_TT465_RUN_MIN       2423
  #define MODBUS_REG_TT465_RUN_MAX       2425
  #define MODBUS_REG_TT465_IDLE_MAX      2427
  #define MODBUS_REG_TT465_STOP_MAX      2429

  #define MODBUS_REG_TT471_STOP_MIN      2431
  #define MODBUS_REG_TT471_IDLE_MIN      2433
  #define MODBUS_REG_TT471_RUN_MIN       2435
  #define MODBUS_REG_TT471_RUN_MAX       2437
  #define MODBUS_REG_TT471_IDLE_MAX      2439
  #define MODBUS_REG_TT471_STOP_MAX      2441

  #define MODBUS_REG_TT532_STOP_MIN      2443
  #define MODBUS_REG_TT532_IDLE_MIN      2445
  #define MODBUS_REG_TT532_RUN_MIN       2447
  #define MODBUS_REG_TT532_RUN_MAX       2449
  #define MODBUS_REG_TT532_IDLE_MAX      2451
  #define MODBUS_REG_TT532_STOP_MAX      2453

  #define MODBUS_REG_TT557_STOP_MIN      2455
  #define MODBUS_REG_TT557_IDLE_MIN      2457
  #define MODBUS_REG_TT557_RUN_MIN       2459
  #define MODBUS_REG_TT557_RUN_MAX       2461
  #define MODBUS_REG_TT557_IDLE_MAX      2463
  #define MODBUS_REG_TT557_STOP_MAX      2465

  #define   MODBUS_REG_S1A_FLOW_COUNT   2700
  #define   MODBUS_REG_S1B_FLOW_COUNT   2702
  #define   MODBUS_REG_S2A_FLOW_COUNT   2730
  #define   MODBUS_REG_S2B_FLOW_COUNT   2732
  #define   MODBUS_REG_S3A_FLOW_COUNT   2760
  #define   MODBUS_REG_S3B_FLOW_COUNT   2762

  #define MODBUS_REG_S1_ACTUAL_CPM    2704
  #define MODBUS_REG_S1_MIN_CPM       2706
  #define MODBUS_REG_S1_MAX_CPM       2708
  #define MODBUS_REG_S2_ACTUAL_CPM    2734
  #define MODBUS_REG_S2_MIN_CPM       2736
  #define MODBUS_REG_S2_MAX_CPM       2738
  #define MODBUS_REG_S3_ACTUAL_CPM    2764
  #define MODBUS_REG_S3_MIN_CPM       2766
  #define MODBUS_REG_S3_MAX_CPM       2768

  #define   MODBUS_REG_PV1_CL           81     //Analog output 1 0-10VDC Proportional valve 1 TB57.1 (PCB7.10 and later ONLY)
  #define   MODBUS_REG_PV2_CL           83     //Analog output 2 0-10VDC Proportional valve 2 TB57.3 (PCB7.10 and later ONLY)
  #define   MODBUS_REG_PV3_CL           85     //Analog output 3 0-10VDC Proportional valve 3 TB57.5 (PCB7.10 and later ONLY)
  #define   MODBUS_REG_PV4_CL           87     //Analog output 4 0-10VDC Proportional valve 4 TB57.7 (PCB7.10 and later ONLY)

  #define   FLAG_CLEAR                          0
  #define   FLAG_ACTIVE                         1

  #if defined(HARDWARE_GIGA)
    void modbuxRxData(uint8_t, int);
    int modbus_loop(int);
    int get_tx_bytes(int);
    uint8_t * get_tx_buffer(int);
  #endif
#endif