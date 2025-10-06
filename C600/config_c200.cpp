/*
    Author: Christopher Glanville
    Description:    Storage of all operational variables. This class provides a single place to save and retrieve all operational variables 
                    in the c200. By using a class like this all other classes (modbus, ble, wifi, compressorStroke, compressorCore and USB)
                    can all have access to a single variable, and there is a single location to control those variables, i.e. limit min/max
                    values etc. 

                    This class provides a series of set, get and test functions to interact wth the operational variables. Depending on the 
                    variable type for example get..() may return a float, integer or boolean.
   Date: 25/5/24
   updated : 25/5/24
*/

#include <stdint.h>
#include <Wire.h>
#include "C200.h"   // Required so that we know the PCB version we are working with
#include "config_c200.h"
#include "movingAvg.h"
#include "watchdog_c200.h"
#include "eeprom.h"

// typedef union
// {
//   float number;
//   uint16_t words[2];
//   uint8_t bytes[4];
//   uint32_t uint_value;
// } FLOATUNION_t;


static FLOATUNION_t cmpStrokeTempVal;

static FLOATUNION_t error_bits,error2_bits;                   // 32 bit error state indicator
static FLOATUNION_t remote_lo_bits;               // Remote lockout bits
static FLOATUNION_t remote_lo_mask;               // Remote lockout bits
static FLOATUNION_t operational_bits;             // 32 bit operating state indicator
static FLOATUNION_t idle_s1_bits;                 // 32 bit idle state indicator (To show the cause of IDLE S1)
static FLOATUNION_t idle_s2_bits;                 // 32 bit idle state indicator (To show the cause of IDLE S2)
static FLOATUNION_t idle_s3_bits;                 // 32 bit idle state indicator (To show the cause of IDLE S3)
static FLOATUNION_t sw1_bits;                     // 12 switch states of SW1. b0 -> s1, s11 - >s12
static FLOATUNION_t memory;                       // Memory action functions
static FLOATUNION_t runtime_timer;
static FLOATUNION_t relays,inputs;
static FLOATUNION_t opstate;
static FLOATUNION_t serial_number, slave_id;
static FLOATUNION_t pv1_ao, pv2_ao, pv3_ao, pv4_ao;
static FLOATUNION_t pv1_cl, pv2_cl, pv3_cl, pv4_cl;
static FLOATUNION_t tt01, tt02, tt03, tt04,tt05, tt06, tt07, tt08,tt09, tt10, tt11, tt12,tt13, tt14;
static FLOATUNION_t pt01, pt02, pt03, pt04,pt05, pt06, pt07, pt08,pt09, pt10, pt11, pt12,pt13, pt14, pt15;
static FLOATUNION_t pulse1,pulse2,pulse3;
static FLOATUNION_t flowcnt1a,flowcnt1b,flowcnt2a,flowcnt2b,flowcnt3a,flowcnt3b;
static FLOATUNION_t intensifier_s1_actual_cpm, s1_min_cpm,s1_max_cpm,intensifier_s2_actual_cpm,s2_min_cpm,s2_max_cpm,intensifier_s3_actual_cpm,s3_min_cpm,s3_max_cpm;
static FLOATUNION_t pt405_STOP_min, pt405_IDLE_min, pt405_RUN_min, pt405_RUN_max, pt405_IDLE_max, pt405_STOP_max;    // PT-405
static FLOATUNION_t pt426_STOP_min, pt426_IDLE_min, pt426_RUN_min, pt426_RUN_max, pt426_IDLE_max, pt426_STOP_max;    // PT-426
static FLOATUNION_t pt442_STOP_min, pt442_IDLE_min, pt442_RUN_min, pt442_RUN_max, pt442_IDLE_max, pt442_STOP_max;    // PT-442
static FLOATUNION_t pt410_STOP_min, pt410_IDLE_min, pt410_RUN_min, pt410_RUN_max, pt410_IDLE_max, pt410_STOP_max;    // PT-410
static FLOATUNION_t pt457_STOP_min, pt457_IDLE_min, pt457_RUN_min, pt457_RUN_max, pt457_IDLE_max, pt457_STOP_max;    // PT-457
static FLOATUNION_t pt416_STOP_min, pt416_IDLE_min, pt416_RUN_min, pt416_RUN_max, pt416_IDLE_max, pt416_STOP_max;    // PT-416
static FLOATUNION_t pt468_STOP_min, pt468_IDLE_min, pt468_RUN_min, pt468_RUN_max, pt468_IDLE_max, pt468_STOP_max;    // PT-468
static FLOATUNION_t pt487_STOP_min, pt487_IDLE_min, pt487_RUN_min, pt487_RUN_max, pt487_IDLE_max, pt487_STOP_max;    // PT-487
static FLOATUNION_t pt496_STOP_min, pt496_IDLE_min, pt496_RUN_min, pt496_RUN_max, pt496_IDLE_max, pt496_STOP_max;    // PT-496
static FLOATUNION_t pt531_STOP_min, pt531_IDLE_min, pt531_RUN_min, pt531_RUN_max, pt531_IDLE_max, pt531_STOP_max;    // PT-531
static FLOATUNION_t pt576_STOP_min, pt576_IDLE_min, pt576_RUN_min, pt576_RUN_max, pt576_IDLE_max, pt576_STOP_max;    // PT-576

static FLOATUNION_t tt432_STOP_min, tt432_IDLE_min, tt432_RUN_min, tt432_RUN_max, tt432_IDLE_max, tt432_STOP_max;    // TT-432
static FLOATUNION_t tt439_STOP_min, tt439_IDLE_min, tt439_RUN_min, tt439_RUN_max, tt439_IDLE_max, tt439_STOP_max;    // TT-439
static FLOATUNION_t tt435_STOP_min, tt435_IDLE_min, tt435_RUN_min, tt435_RUN_max, tt435_IDLE_max, tt435_STOP_max;    // TT-435
static FLOATUNION_t tt436_STOP_min, tt436_IDLE_min, tt436_RUN_min, tt436_RUN_max, tt436_IDLE_max, tt436_STOP_max;    // TT-436
static FLOATUNION_t tt447_STOP_min, tt447_IDLE_min, tt447_RUN_min, tt447_RUN_max, tt447_IDLE_max, tt447_STOP_max;    // TT-447
static FLOATUNION_t tt454_STOP_min, tt454_IDLE_min, tt454_RUN_min, tt454_RUN_max, tt454_IDLE_max, tt454_STOP_max;    // TT-454
static FLOATUNION_t tt450_STOP_min, tt450_IDLE_min, tt450_RUN_min, tt450_RUN_max, tt450_IDLE_max, tt450_STOP_max;    // TT-450
static FLOATUNION_t tt451_STOP_min, tt451_IDLE_min, tt451_RUN_min, tt451_RUN_max, tt451_IDLE_max, tt451_STOP_max;    // TT-451
static FLOATUNION_t tt462_STOP_min, tt462_IDLE_min, tt462_RUN_min, tt462_RUN_max, tt462_IDLE_max, tt462_STOP_max;    // TT-462
static FLOATUNION_t tt474_STOP_min, tt474_IDLE_min, tt474_RUN_min, tt474_RUN_max, tt474_IDLE_max, tt474_STOP_max;    // TT-474
static FLOATUNION_t tt465_STOP_min, tt465_IDLE_min, tt465_RUN_min, tt465_RUN_max, tt465_IDLE_max, tt465_STOP_max;    // TT-465
static FLOATUNION_t tt471_STOP_min, tt471_IDLE_min, tt471_RUN_min, tt471_RUN_max, tt471_IDLE_max, tt471_STOP_max;    // TT-471
static FLOATUNION_t tt532_STOP_min, tt532_IDLE_min, tt532_RUN_min, tt532_RUN_max, tt532_IDLE_max, tt532_STOP_max;    // TT-532
static FLOATUNION_t tt557_STOP_min, tt557_IDLE_min, tt557_RUN_min, tt557_RUN_max, tt557_IDLE_max, tt557_STOP_max;    // TT-557


/*
    Name:         init_config
    Arguments:    none
    Returns:      nil
    Description:  This function must be called prior to any other in this class. 

                  Initialize any variables to a default state. Shortly after power up the USB class may reset some of these variables according 
                  to saved values, however without a USB flash drive or a valid config file on said flash drive default values here will take 
                  priority. 
                  
                  Some of the default values come from #defines in config_c200.ch
*/
void init_config()
{
  slave_id.number = 1; 
  serial_number.number = 1;
  sw1_bits.number = 0;                  // Switch state bits - SW1
  error_bits.number = 0;                // Default to no errors
  error2_bits.number = 0;               // Default to no errors2
  opstate.number = 0;
  relays.number = 0;
  inputs.number = 0;  
  idle_s1_bits.number=0,idle_s2_bits.number=0,idle_s3_bits.number=0; 
  pv1_cl.number = 50; pv2_cl.number = 50; pv3_cl.number = 50; pv4_cl.number = 50;
  tt01.number = 0,tt02.number = 0,tt03.number = 0,tt04.number = 0,tt05.number = 0,tt06.number = 0,tt07.number = 0;
  tt08.number = 0,tt09.number = 0,tt10.number = 0,tt11.number = 0,tt12.number = 0,tt13.number = 0,tt14.number = 0;
  pt01.number = 0,pt02.number = 0,pt03.number = 0,pt04.number = 0,pt05.number = 0,pt06.number = 0,pt07.number = 0;
  pt08.number = 0,pt09.number = 0,pt10.number = 0,pt11.number = 0,pt12.number = 0,pt13.number = 0,pt14.number = 0, pt15.number = 0;
  pt405_STOP_min.number = SETPOINT_PT405_ENTER_STOP_MIN;    // Initial setpoint values 
  pt405_IDLE_min.number = SETPOINT_PT405_ENTER_IDLE_MIN; 
  pt405_RUN_min.number = SETPOINT_PT405_ENTER_RUN_MIN;
  pt405_RUN_max.number = SETPOINT_PT405_ENTER_RUN_MAX; 
  pt405_IDLE_max.number = SETPOINT_PT405_ENTER_IDLE_MAX;
  pt405_STOP_max.number = SETPOINT_PT405_ENTER_STOP_MAX;

  pt426_STOP_min.number = SETPOINT_PT426_ENTER_STOP_MIN;    // Initial setpoint values 
  pt426_IDLE_min.number = SETPOINT_PT426_ENTER_IDLE_MIN; 
  pt426_RUN_min.number = SETPOINT_PT426_ENTER_RUN_MIN;
  pt426_RUN_max.number = SETPOINT_PT426_ENTER_RUN_MAX; 
  pt426_IDLE_max.number = SETPOINT_PT426_ENTER_IDLE_MAX;
  pt426_STOP_max.number = SETPOINT_PT426_ENTER_STOP_MAX;

  pt442_STOP_min.number = SETPOINT_PT442_ENTER_STOP_MIN;    // Initial setpoint values 
  pt442_IDLE_min.number = SETPOINT_PT442_ENTER_IDLE_MIN; 
  pt442_RUN_min.number = SETPOINT_PT442_ENTER_RUN_MIN;
  pt442_RUN_max.number = SETPOINT_PT442_ENTER_RUN_MAX; 
  pt442_IDLE_max.number = SETPOINT_PT442_ENTER_IDLE_MAX;
  pt442_STOP_max.number = SETPOINT_PT442_ENTER_STOP_MAX;

  pt410_STOP_min.number = SETPOINT_PT410_ENTER_STOP_MIN;    // Initial setpoint values 
  pt410_IDLE_min.number = SETPOINT_PT410_ENTER_IDLE_MIN; 
  pt410_RUN_min.number = SETPOINT_PT410_ENTER_RUN_MIN;
  pt410_RUN_max.number = SETPOINT_PT410_ENTER_RUN_MAX; 
  pt410_IDLE_max.number = SETPOINT_PT410_ENTER_IDLE_MAX;
  pt410_STOP_max.number = SETPOINT_PT410_ENTER_STOP_MAX;

  pt457_STOP_min.number = SETPOINT_PT457_ENTER_STOP_MIN;    // Initial setpoint values 
  pt457_IDLE_min.number = SETPOINT_PT457_ENTER_IDLE_MIN; 
  pt457_RUN_min.number = SETPOINT_PT457_ENTER_RUN_MIN;
  pt457_RUN_max.number = SETPOINT_PT457_ENTER_RUN_MAX; 
  pt457_IDLE_max.number = SETPOINT_PT457_ENTER_IDLE_MAX;
  pt457_STOP_max.number = SETPOINT_PT457_ENTER_STOP_MAX;

  pt416_STOP_min.number = SETPOINT_PT416_ENTER_STOP_MIN;    // Initial setpoint values 
  pt416_IDLE_min.number = SETPOINT_PT416_ENTER_IDLE_MIN; 
  pt416_RUN_min.number = SETPOINT_PT416_ENTER_RUN_MIN;
  pt416_RUN_max.number = SETPOINT_PT416_ENTER_RUN_MAX; 
  pt416_IDLE_max.number = SETPOINT_PT416_ENTER_IDLE_MAX;
  pt416_STOP_max.number = SETPOINT_PT416_ENTER_STOP_MAX;

  pt468_STOP_min.number = SETPOINT_PT468_ENTER_STOP_MIN;    // Initial setpoint values 
  pt468_IDLE_min.number = SETPOINT_PT468_ENTER_IDLE_MIN; 
  pt468_RUN_min.number = SETPOINT_PT468_ENTER_RUN_MIN;
  pt468_RUN_max.number = SETPOINT_PT468_ENTER_RUN_MAX; 
  pt468_IDLE_max.number = SETPOINT_PT468_ENTER_IDLE_MAX;
  pt468_STOP_max.number = SETPOINT_PT468_ENTER_STOP_MAX;

  pt487_STOP_min.number = SETPOINT_PT487_ENTER_STOP_MIN;    // Initial setpoint values 
  pt487_IDLE_min.number = SETPOINT_PT487_ENTER_IDLE_MIN; 
  pt487_RUN_min.number = SETPOINT_PT487_ENTER_RUN_MIN;
  pt487_RUN_max.number = SETPOINT_PT487_ENTER_RUN_MAX; 
  pt487_IDLE_max.number = SETPOINT_PT487_ENTER_IDLE_MAX;
  pt487_STOP_max.number = SETPOINT_PT487_ENTER_STOP_MAX;

  pt496_STOP_min.number = SETPOINT_PT496_ENTER_STOP_MIN;    // Initial setpoint values 
  pt496_IDLE_min.number = SETPOINT_PT496_ENTER_IDLE_MIN; 
  pt496_RUN_min.number = SETPOINT_PT496_ENTER_RUN_MIN;
  pt496_RUN_max.number = SETPOINT_PT496_ENTER_RUN_MAX; 
  pt496_IDLE_max.number = SETPOINT_PT496_ENTER_IDLE_MAX;
  pt496_STOP_max.number = SETPOINT_PT496_ENTER_STOP_MAX;

  pt531_STOP_min.number = SETPOINT_PT531_ENTER_STOP_MIN;    // Initial setpoint values 
  pt531_IDLE_min.number = SETPOINT_PT531_ENTER_IDLE_MIN; 
  pt531_RUN_min.number = SETPOINT_PT531_ENTER_RUN_MIN;
  pt531_RUN_max.number = SETPOINT_PT531_ENTER_RUN_MAX; 
  pt531_IDLE_max.number = SETPOINT_PT531_ENTER_IDLE_MAX;
  pt531_STOP_max.number = SETPOINT_PT531_ENTER_STOP_MAX;

  pt576_STOP_min.number = SETPOINT_PT576_ENTER_STOP_MIN;    // Initial setpoint values 
  pt576_IDLE_min.number = SETPOINT_PT576_ENTER_IDLE_MIN; 
  pt576_RUN_min.number = SETPOINT_PT576_ENTER_RUN_MIN;
  pt576_RUN_max.number = SETPOINT_PT576_ENTER_RUN_MAX; 
  pt576_IDLE_max.number = SETPOINT_PT576_ENTER_IDLE_MAX;
  pt576_STOP_max.number = SETPOINT_PT576_ENTER_STOP_MAX;

  tt432_STOP_min.number = SETPOINT_TT432_ENTER_STOP_MIN;    // Initial setpoint values 
  tt432_IDLE_min.number = SETPOINT_TT432_ENTER_IDLE_MIN; 
  tt432_RUN_min.number = SETPOINT_TT432_ENTER_RUN_MIN;
  tt432_RUN_max.number = SETPOINT_TT432_ENTER_RUN_MAX; 
  tt432_IDLE_max.number = SETPOINT_TT432_ENTER_IDLE_MAX;
  tt432_STOP_max.number = SETPOINT_TT432_ENTER_STOP_MAX;

  tt439_STOP_min.number = SETPOINT_TT439_ENTER_STOP_MIN;    // Initial setpoint values 
  tt439_IDLE_min.number = SETPOINT_TT439_ENTER_IDLE_MIN; 
  tt439_RUN_min.number = SETPOINT_TT439_ENTER_RUN_MIN;
  tt439_RUN_max.number = SETPOINT_TT439_ENTER_RUN_MAX; 
  tt439_IDLE_max.number = SETPOINT_TT439_ENTER_IDLE_MAX;
  tt439_STOP_max.number = SETPOINT_TT439_ENTER_STOP_MAX;

  tt435_STOP_min.number = SETPOINT_TT435_ENTER_STOP_MIN;    // Initial setpoint values 
  tt435_IDLE_min.number = SETPOINT_TT435_ENTER_IDLE_MIN; 
  tt435_RUN_min.number = SETPOINT_TT435_ENTER_RUN_MIN;
  tt435_RUN_max.number = SETPOINT_TT435_ENTER_RUN_MAX; 
  tt435_IDLE_max.number = SETPOINT_TT435_ENTER_IDLE_MAX;
  tt435_STOP_max.number = SETPOINT_TT435_ENTER_STOP_MAX;

  tt436_STOP_min.number = SETPOINT_TT436_ENTER_STOP_MIN;    // Initial setpoint values 
  tt436_IDLE_min.number = SETPOINT_TT436_ENTER_IDLE_MIN; 
  tt436_RUN_min.number = SETPOINT_TT436_ENTER_RUN_MIN;
  tt436_RUN_max.number = SETPOINT_TT436_ENTER_RUN_MAX; 
  tt436_IDLE_max.number = SETPOINT_TT436_ENTER_IDLE_MAX;
  tt436_STOP_max.number = SETPOINT_TT436_ENTER_STOP_MAX;

  tt447_STOP_min.number = SETPOINT_TT447_ENTER_STOP_MIN;    // Initial setpoint values 
  tt447_IDLE_min.number = SETPOINT_TT447_ENTER_IDLE_MIN; 
  tt447_RUN_min.number = SETPOINT_TT447_ENTER_RUN_MIN;
  tt447_RUN_max.number = SETPOINT_TT447_ENTER_RUN_MAX; 
  tt447_IDLE_max.number = SETPOINT_TT447_ENTER_IDLE_MAX;
  tt447_STOP_max.number = SETPOINT_TT447_ENTER_STOP_MAX;

  tt454_STOP_min.number = SETPOINT_TT454_ENTER_STOP_MIN;    // Initial setpoint values 
  tt454_IDLE_min.number = SETPOINT_TT454_ENTER_IDLE_MIN; 
  tt454_RUN_min.number = SETPOINT_TT454_ENTER_RUN_MIN;
  tt454_RUN_max.number = SETPOINT_TT454_ENTER_RUN_MAX; 
  tt454_IDLE_max.number = SETPOINT_TT454_ENTER_IDLE_MAX;
  tt454_STOP_max.number = SETPOINT_TT454_ENTER_STOP_MAX;

  tt450_STOP_min.number = SETPOINT_TT450_ENTER_STOP_MIN;    // Initial setpoint values 
  tt450_IDLE_min.number = SETPOINT_TT450_ENTER_IDLE_MIN; 
  tt450_RUN_min.number = SETPOINT_TT450_ENTER_RUN_MIN;
  tt450_RUN_max.number = SETPOINT_TT450_ENTER_RUN_MAX; 
  tt450_IDLE_max.number = SETPOINT_TT450_ENTER_IDLE_MAX;
  tt450_STOP_max.number = SETPOINT_TT450_ENTER_STOP_MAX;

  tt451_STOP_min.number = SETPOINT_TT451_ENTER_STOP_MIN;    // Initial setpoint values 
  tt451_IDLE_min.number = SETPOINT_TT451_ENTER_IDLE_MIN; 
  tt451_RUN_min.number = SETPOINT_TT451_ENTER_RUN_MIN;
  tt451_RUN_max.number = SETPOINT_TT451_ENTER_RUN_MAX; 
  tt451_IDLE_max.number = SETPOINT_TT451_ENTER_IDLE_MAX;
  tt451_STOP_max.number = SETPOINT_TT451_ENTER_STOP_MAX;

  tt462_STOP_min.number = SETPOINT_TT462_ENTER_STOP_MIN;    // Initial setpoint values 
  tt462_IDLE_min.number = SETPOINT_TT462_ENTER_IDLE_MIN; 
  tt462_RUN_min.number = SETPOINT_TT462_ENTER_RUN_MIN;
  tt462_RUN_max.number = SETPOINT_TT462_ENTER_RUN_MAX; 
  tt462_IDLE_max.number = SETPOINT_TT462_ENTER_IDLE_MAX;
  tt462_STOP_max.number = SETPOINT_TT462_ENTER_STOP_MAX;

  tt474_STOP_min.number = SETPOINT_TT474_ENTER_STOP_MIN;    // Initial setpoint values 
  tt474_IDLE_min.number = SETPOINT_TT474_ENTER_IDLE_MIN; 
  tt474_RUN_min.number = SETPOINT_TT474_ENTER_RUN_MIN;
  tt474_RUN_max.number = SETPOINT_TT474_ENTER_RUN_MAX; 
  tt474_IDLE_max.number = SETPOINT_TT474_ENTER_IDLE_MAX;
  tt474_STOP_max.number = SETPOINT_TT474_ENTER_STOP_MAX;

  tt465_STOP_min.number = SETPOINT_TT465_ENTER_STOP_MIN;    // Initial setpoint values 
  tt465_IDLE_min.number = SETPOINT_TT465_ENTER_IDLE_MIN; 
  tt465_RUN_min.number = SETPOINT_TT465_ENTER_RUN_MIN;
  tt465_RUN_max.number = SETPOINT_TT465_ENTER_RUN_MAX; 
  tt465_IDLE_max.number = SETPOINT_TT465_ENTER_IDLE_MAX;
  tt465_STOP_max.number = SETPOINT_TT465_ENTER_STOP_MAX;

  tt471_STOP_min.number = SETPOINT_TT471_ENTER_STOP_MIN;    // Initial setpoint values 
  tt471_IDLE_min.number = SETPOINT_TT471_ENTER_IDLE_MIN; 
  tt471_RUN_min.number = SETPOINT_TT471_ENTER_RUN_MIN;
  tt471_RUN_max.number = SETPOINT_TT471_ENTER_RUN_MAX; 
  tt471_IDLE_max.number = SETPOINT_TT471_ENTER_IDLE_MAX;
  tt471_STOP_max.number = SETPOINT_TT471_ENTER_STOP_MAX;

  tt532_STOP_min.number = SETPOINT_TT532_ENTER_STOP_MIN;    // Initial setpoint values 
  tt532_IDLE_min.number = SETPOINT_TT532_ENTER_IDLE_MIN; 
  tt532_RUN_min.number = SETPOINT_TT532_ENTER_RUN_MIN;
  tt532_RUN_max.number = SETPOINT_TT532_ENTER_RUN_MAX; 
  tt532_IDLE_max.number = SETPOINT_TT532_ENTER_IDLE_MAX;
  tt532_STOP_max.number = SETPOINT_TT532_ENTER_STOP_MAX;

  tt557_STOP_min.number = SETPOINT_TT557_ENTER_STOP_MIN;    // Initial setpoint values 
  tt557_IDLE_min.number = SETPOINT_TT557_ENTER_IDLE_MIN; 
  tt557_RUN_min.number = SETPOINT_TT557_ENTER_RUN_MIN;
  tt557_RUN_max.number = SETPOINT_TT557_ENTER_RUN_MAX; 
  tt557_IDLE_max.number = SETPOINT_TT557_ENTER_IDLE_MAX;
  tt557_STOP_max.number = SETPOINT_TT557_ENTER_STOP_MAX;

//flowcnt1 is pulse limit for intensifier 1 DirectionA
//flowcnt2 is pulse limit for intensifier 1 DirectionB
////flowcnt3 not defined yet,??
  flowcnt1a.number = 3000,flowcnt1b.number = 3000,flowcnt2a.number = 3000,flowcnt2b.number = 3000,flowcnt3a.number = 3000,flowcnt3b.number = 3000;
  intensifier_s1_actual_cpm.number = 0, s1_min_cpm.number= 7, s1_max_cpm.number=12,intensifier_s2_actual_cpm.number = 0, s2_min_cpm.number= 7, s2_max_cpm.number=12,intensifier_s3_actual_cpm.number = 0, s3_min_cpm.number=5, s3_max_cpm.number=10;
  eeprom_setup();  
}

/*
    Name:         get_config_parameter
    Arguments:    The unique identifier of the parameter to be returned - as defined in config_c200.h
    Returns:      float - All variables are stored as a float for compatability, however not all variables are used as a float. It is up to the 
                  caller to decide if it is appropriate for the variable being requested to be returned as a float. For example, OP_STATE_ALL
                  is a 32-bit binary field indicator. This variable (OP_STATE_ALL) would have no purpose as a float. 
    Description:  Return the parameter specifed by 'param' as a 32-bit float. 
*/
float get_config_parameter(int param){
  switch(param){
    case CONFIG_PARAM_RELAYS:       // Relays
       return(relays.number); 
    case CONFIG_PARAM_OP_STATE:
       return(opstate.number);
    case CONFIG_PARAM_INPUTS:       // Inputs
      return(inputs.number);
    case SWITCH_STATE_ALL:         //  DIP switches
      return(sw1_bits.number);  
    case CONFIG_PARAM_SLAVE_ID:       // Modbus slave ID
      return(slave_id.number);
    case CONFIG_PARAM_SERIAL_NUM:     // Unique device serial number
      return(serial_number.number);
    case CONFIG_PARAM_PV1_CL:
      return(pv1_cl.number);
    case CONFIG_PARAM_PV2_CL:
      return(pv2_cl.number);
    case CONFIG_PARAM_PV3_CL:
      return(pv3_cl.number);
    case CONFIG_PARAM_PV4_CL:
      return(pv4_cl.number);      
    case CONFIG_PARAM_TC1:
      return(tt01.number); 
    case CONFIG_PARAM_TC2:
      return(tt02.number); 
    case CONFIG_PARAM_TC3:
      return(tt03.number); 
    case CONFIG_PARAM_TC4:
      return(tt04.number); 
    case CONFIG_PARAM_TC5:
      return(tt05.number); 
    case CONFIG_PARAM_TC6:
      return(tt06.number); 
    case CONFIG_PARAM_TC7:
      return(tt07.number); 
    case CONFIG_PARAM_TC8:
      return(tt08.number); 
    case CONFIG_PARAM_TC9:
      return(tt09.number); 
    case CONFIG_PARAM_TC10:
      return(tt10.number); 
    case CONFIG_PARAM_TC11:
      return(tt11.number); 
    case CONFIG_PARAM_TC12:
      return(tt12.number); 
    case CONFIG_PARAM_TC13:
      return(tt13.number); 
    case CONFIG_PARAM_TC14:
      return(tt14.number); 
    case CONFIG_PARAM_COOLANT1_PT:
      return(pt01.number); 
    case CONFIG_PARAM_COOLANT2_PT:
      return(pt02.number); 
    case CONFIG_PARAM_SUCTION_PT:
      return(pt03.number); 
    case CONFIG_PARAM_SUCTION_TANK:
      return(pt04.number); 
    case CONFIG_PARAM_STAGE1_PT:
     // return(pt05.number); 
      return(-5000); 
    case CONFIG_PARAM_STAGE1_TANK:
      return(pt06.number); 
    case CONFIG_PARAM_STAGE2_PT:
      // return(pt07.number); 
       return(-5000); 
    case CONFIG_PARAM_STAGE2_TANK:
      return(pt08.number); 
    case CONFIG_PARAM_STAGE3_PT:
      return(pt09.number);   
    case CONFIG_PARAM_STAGE3_TANK:
      return(pt10.number);
    case CONFIG_PARAM_HFL_PUMP1:
      return(pt11.number);     
    case CONFIG_PARAM_PT405_STOP_MIN:
      return(pt405_STOP_min.number); 
    case CONFIG_PARAM_PT405_IDLE_MIN:
      return(pt405_IDLE_min.number); 
    case CONFIG_PARAM_PT405_RUN_MIN:
      return(pt405_RUN_min.number); 
    case CONFIG_PARAM_PT405_RUN_MAX:
      return(pt405_RUN_max.number); 
    case CONFIG_PARAM_PT405_IDLE_MAX:
      return(pt405_IDLE_max.number); 
    case CONFIG_PARAM_PT405_STOP_MAX:
      return(pt405_STOP_max.number);
    //PT-426 RUN,IDLE,STOP
    case CONFIG_PARAM_PT426_STOP_MIN:
      return(pt426_STOP_min.number); 
    case CONFIG_PARAM_PT426_IDLE_MIN:
      return(pt426_IDLE_min.number); 
    case CONFIG_PARAM_PT426_RUN_MIN:
      return(pt426_RUN_min.number); 
    case CONFIG_PARAM_PT426_RUN_MAX:
      return(pt426_RUN_max.number); 
    case CONFIG_PARAM_PT426_IDLE_MAX:
      return(pt426_IDLE_max.number); 
    case CONFIG_PARAM_PT426_STOP_MAX:
      return(pt426_STOP_max.number);  
    //PT-442 RUN,IDLE,STOP
    case CONFIG_PARAM_PT442_STOP_MIN:
      return(pt442_STOP_min.number); 
    case CONFIG_PARAM_PT442_IDLE_MIN:
      return(pt442_IDLE_min.number); 
    case CONFIG_PARAM_PT442_RUN_MIN:
      return(pt442_RUN_min.number); 
    case CONFIG_PARAM_PT442_RUN_MAX:
      return(pt442_RUN_max.number); 
    case CONFIG_PARAM_PT442_IDLE_MAX:
      return(pt442_IDLE_max.number); 
    case CONFIG_PARAM_PT442_STOP_MAX:
      return(pt442_STOP_max.number);   
    //PT-410     
    case CONFIG_PARAM_PT410_STOP_MIN:
      return(pt410_STOP_min.number); 
    case CONFIG_PARAM_PT410_IDLE_MIN:
      return(pt410_IDLE_min.number); 
    case CONFIG_PARAM_PT410_RUN_MIN:
      return(pt410_RUN_min.number); 
    case CONFIG_PARAM_PT410_RUN_MAX:
      return(pt410_RUN_max.number); 
    case CONFIG_PARAM_PT410_IDLE_MAX:
      return(pt410_IDLE_max.number); 
    case CONFIG_PARAM_PT410_STOP_MAX:
      return(pt410_STOP_max.number);         

    //PT-457   
    case CONFIG_PARAM_PT457_STOP_MIN:
      return(pt457_STOP_min.number); 
    case CONFIG_PARAM_PT457_IDLE_MIN:
      return(pt457_IDLE_min.number); 
    case CONFIG_PARAM_PT457_RUN_MIN:
      return(pt457_RUN_min.number); 
    case CONFIG_PARAM_PT457_RUN_MAX:
      return(pt457_RUN_max.number); 
    case CONFIG_PARAM_PT457_IDLE_MAX:
      return(pt457_IDLE_max.number); 
    case CONFIG_PARAM_PT457_STOP_MAX:
      return(pt457_STOP_max.number);         

    //PT-416   
    case CONFIG_PARAM_PT416_STOP_MIN:
      return(pt416_STOP_min.number); 
    case CONFIG_PARAM_PT416_IDLE_MIN:
      return(pt416_IDLE_min.number); 
    case CONFIG_PARAM_PT416_RUN_MIN:
      return(pt416_RUN_min.number); 
    case CONFIG_PARAM_PT416_RUN_MAX:
      return(pt416_RUN_max.number); 
    case CONFIG_PARAM_PT416_IDLE_MAX:
      return(pt416_IDLE_max.number); 
    case CONFIG_PARAM_PT416_STOP_MAX:
      return(pt416_STOP_max.number);         

   //PT-468
    case CONFIG_PARAM_PT468_STOP_MIN:
      return(pt468_STOP_min.number); 
    case CONFIG_PARAM_PT468_IDLE_MIN:
      return(pt468_IDLE_min.number); 
    case CONFIG_PARAM_PT468_RUN_MIN:
      return(pt468_RUN_min.number); 
    case CONFIG_PARAM_PT468_RUN_MAX:
      return(pt468_RUN_max.number); 
    case CONFIG_PARAM_PT468_IDLE_MAX:
      return(pt468_IDLE_max.number); 
    case CONFIG_PARAM_PT468_STOP_MAX:
      return(pt468_STOP_max.number);            
  
    //PT-487
    case CONFIG_PARAM_PT487_STOP_MIN:
      return(pt487_STOP_min.number); 
    case CONFIG_PARAM_PT487_IDLE_MIN:
      return(pt487_IDLE_min.number); 
    case CONFIG_PARAM_PT487_RUN_MIN:
      return(pt487_RUN_min.number); 
    case CONFIG_PARAM_PT487_RUN_MAX:
      return(pt487_RUN_max.number); 
    case CONFIG_PARAM_PT487_IDLE_MAX:
      return(pt487_IDLE_max.number); 
    case CONFIG_PARAM_PT487_STOP_MAX:
      return(pt487_STOP_max.number);            

    //PT-496
    case CONFIG_PARAM_PT496_STOP_MIN:
      return(pt496_STOP_min.number); 
    case CONFIG_PARAM_PT496_IDLE_MIN:
      return(pt496_IDLE_min.number); 
    case CONFIG_PARAM_PT496_RUN_MIN:
      return(pt496_RUN_min.number); 
    case CONFIG_PARAM_PT496_RUN_MAX:
      return(pt496_RUN_max.number); 
    case CONFIG_PARAM_PT496_IDLE_MAX:
      return(pt496_IDLE_max.number); 
    case CONFIG_PARAM_PT496_STOP_MAX:
      return(pt496_STOP_max.number);            

    //PT-531
    case CONFIG_PARAM_PT531_STOP_MIN:
      return(pt531_STOP_min.number); 
    case CONFIG_PARAM_PT531_IDLE_MIN:
      return(pt531_IDLE_min.number); 
    case CONFIG_PARAM_PT531_RUN_MIN:
      return(pt531_RUN_min.number); 
    case CONFIG_PARAM_PT531_RUN_MAX:
      return(pt531_RUN_max.number); 
    case CONFIG_PARAM_PT531_IDLE_MAX:
      return(pt531_IDLE_max.number); 
    case CONFIG_PARAM_PT531_STOP_MAX:
      return(pt531_STOP_max.number);               

    //PT-576
    case CONFIG_PARAM_PT576_STOP_MIN:
      return(pt576_STOP_min.number); 
    case CONFIG_PARAM_PT576_IDLE_MIN:
      return(pt576_IDLE_min.number); 
    case CONFIG_PARAM_PT576_RUN_MIN:
      return(pt576_RUN_min.number); 
    case CONFIG_PARAM_PT576_RUN_MAX:
      return(pt576_RUN_max.number); 
    case CONFIG_PARAM_PT576_IDLE_MAX:
      return(pt576_IDLE_max.number); 
    case CONFIG_PARAM_PT576_STOP_MAX:
      return(pt576_STOP_max.number); 

    //TT-432
    case CONFIG_PARAM_TT432_STOP_MIN:
      return(tt432_STOP_min.number); 
    case CONFIG_PARAM_TT432_IDLE_MIN:
      return(tt432_IDLE_min.number); 
    case CONFIG_PARAM_TT432_RUN_MIN:
      return(tt432_RUN_min.number); 
    case CONFIG_PARAM_TT432_RUN_MAX:
      return(tt432_RUN_max.number); 
    case CONFIG_PARAM_TT432_IDLE_MAX:
      return(tt432_IDLE_max.number); 
    case CONFIG_PARAM_TT432_STOP_MAX:
      return(tt432_STOP_max.number); 

       //TT-439
    case CONFIG_PARAM_TT439_STOP_MIN:
      return(tt439_STOP_min.number); 
    case CONFIG_PARAM_TT439_IDLE_MIN:
      return(tt439_IDLE_min.number); 
    case CONFIG_PARAM_TT439_RUN_MIN:
      return(tt439_RUN_min.number); 
    case CONFIG_PARAM_TT439_RUN_MAX:
      return(tt439_RUN_max.number); 
    case CONFIG_PARAM_TT439_IDLE_MAX:
      return(tt439_IDLE_max.number); 
    case CONFIG_PARAM_TT439_STOP_MAX:
      return(tt439_STOP_max.number);   

    //TT-435
    case CONFIG_PARAM_TT435_STOP_MIN:
      return(tt435_STOP_min.number); 
    case CONFIG_PARAM_TT435_IDLE_MIN:
      return(tt435_IDLE_min.number); 
    case CONFIG_PARAM_TT435_RUN_MIN:
      return(tt435_RUN_min.number); 
    case CONFIG_PARAM_TT435_RUN_MAX:
      return(tt435_RUN_max.number); 
    case CONFIG_PARAM_TT435_IDLE_MAX:
      return(tt435_IDLE_max.number); 
    case CONFIG_PARAM_TT435_STOP_MAX:
      return(tt435_STOP_max.number);     

    //TT-436
    case CONFIG_PARAM_TT436_STOP_MIN:
      return(tt436_STOP_min.number); 
    case CONFIG_PARAM_TT436_IDLE_MIN:
      return(tt436_IDLE_min.number); 
    case CONFIG_PARAM_TT436_RUN_MIN:
      return(tt436_RUN_min.number); 
    case CONFIG_PARAM_TT436_RUN_MAX:
      return(tt436_RUN_max.number); 
    case CONFIG_PARAM_TT436_IDLE_MAX:
      return(tt436_IDLE_max.number); 
    case CONFIG_PARAM_TT436_STOP_MAX:
      return(tt436_STOP_max.number);     

    //TT-447
    case CONFIG_PARAM_TT447_STOP_MIN:
      return(tt447_STOP_min.number); 
    case CONFIG_PARAM_TT447_IDLE_MIN:
      return(tt447_IDLE_min.number); 
    case CONFIG_PARAM_TT447_RUN_MIN:
      return(tt447_RUN_min.number); 
    case CONFIG_PARAM_TT447_RUN_MAX:
      return(tt447_RUN_max.number); 
    case CONFIG_PARAM_TT447_IDLE_MAX:
      return(tt447_IDLE_max.number); 
    case CONFIG_PARAM_TT447_STOP_MAX:
      return(tt447_STOP_max.number);     

     //TT-454
    case CONFIG_PARAM_TT454_STOP_MIN:
      return(tt454_STOP_min.number); 
    case CONFIG_PARAM_TT454_IDLE_MIN:
      return(tt454_IDLE_min.number); 
    case CONFIG_PARAM_TT454_RUN_MIN:
      return(tt454_RUN_min.number); 
    case CONFIG_PARAM_TT454_RUN_MAX:
      return(tt454_RUN_max.number); 
    case CONFIG_PARAM_TT454_IDLE_MAX:
      return(tt454_IDLE_max.number); 
    case CONFIG_PARAM_TT454_STOP_MAX:
      return(tt454_STOP_max.number);       

    //TT-450
    case CONFIG_PARAM_TT450_STOP_MIN:
      return(tt450_STOP_min.number); 
    case CONFIG_PARAM_TT450_IDLE_MIN:
      return(tt450_IDLE_min.number); 
    case CONFIG_PARAM_TT450_RUN_MIN:
      return(tt450_RUN_min.number); 
    case CONFIG_PARAM_TT450_RUN_MAX:
      return(tt450_RUN_max.number); 
    case CONFIG_PARAM_TT450_IDLE_MAX:
      return(tt450_IDLE_max.number); 
    case CONFIG_PARAM_TT450_STOP_MAX:
      return(tt450_STOP_max.number);         

    //TT-451
    case CONFIG_PARAM_TT451_STOP_MIN:
      return(tt451_STOP_min.number); 
    case CONFIG_PARAM_TT451_IDLE_MIN:
      return(tt451_IDLE_min.number); 
    case CONFIG_PARAM_TT451_RUN_MIN:
      return(tt451_RUN_min.number); 
    case CONFIG_PARAM_TT451_RUN_MAX:
      return(tt451_RUN_max.number); 
    case CONFIG_PARAM_TT451_IDLE_MAX:
      return(tt451_IDLE_max.number); 
    case CONFIG_PARAM_TT451_STOP_MAX:
      return(tt451_STOP_max.number);     

    //TT-462
    case CONFIG_PARAM_TT462_STOP_MIN:
      return(tt462_STOP_min.number); 
    case CONFIG_PARAM_TT462_IDLE_MIN:
      return(tt462_IDLE_min.number); 
    case CONFIG_PARAM_TT462_RUN_MIN:
      return(tt462_RUN_min.number); 
    case CONFIG_PARAM_TT462_RUN_MAX:
      return(tt462_RUN_max.number); 
    case CONFIG_PARAM_TT462_IDLE_MAX:
      return(tt462_IDLE_max.number); 
    case CONFIG_PARAM_TT462_STOP_MAX:
      return(tt462_STOP_max.number);       

    //TT-474
    case CONFIG_PARAM_TT474_STOP_MIN:
      return(tt474_STOP_min.number); 
    case CONFIG_PARAM_TT474_IDLE_MIN:
      return(tt474_IDLE_min.number); 
    case CONFIG_PARAM_TT474_RUN_MIN:
      return(tt474_RUN_min.number); 
    case CONFIG_PARAM_TT474_RUN_MAX:
      return(tt474_RUN_max.number); 
    case CONFIG_PARAM_TT474_IDLE_MAX:
      return(tt474_IDLE_max.number); 
    case CONFIG_PARAM_TT474_STOP_MAX:
      return(tt474_STOP_max.number);    

    //TT-465
    case CONFIG_PARAM_TT465_STOP_MIN:
      return(tt465_STOP_min.number); 
    case CONFIG_PARAM_TT465_IDLE_MIN:
      return(tt465_IDLE_min.number); 
    case CONFIG_PARAM_TT465_RUN_MIN:
      return(tt465_RUN_min.number); 
    case CONFIG_PARAM_TT465_RUN_MAX:
      return(tt465_RUN_max.number); 
    case CONFIG_PARAM_TT465_IDLE_MAX:
      return(tt465_IDLE_max.number); 
    case CONFIG_PARAM_TT465_STOP_MAX:
      return(tt465_STOP_max.number);         

    //TT-471
    case CONFIG_PARAM_TT471_STOP_MIN:
      return(tt471_STOP_min.number); 
    case CONFIG_PARAM_TT471_IDLE_MIN:
      return(tt471_IDLE_min.number); 
    case CONFIG_PARAM_TT471_RUN_MIN:
      return(tt471_RUN_min.number); 
    case CONFIG_PARAM_TT471_RUN_MAX:
      return(tt471_RUN_max.number); 
    case CONFIG_PARAM_TT471_IDLE_MAX:
      return(tt471_IDLE_max.number); 
    case CONFIG_PARAM_TT471_STOP_MAX:
      return(tt471_STOP_max.number);           

    //TT-532
    case CONFIG_PARAM_TT532_STOP_MIN:
      return(tt532_STOP_min.number); 
    case CONFIG_PARAM_TT532_IDLE_MIN:
      return(tt532_IDLE_min.number); 
    case CONFIG_PARAM_TT532_RUN_MIN:
      return(tt532_RUN_min.number); 
    case CONFIG_PARAM_TT532_RUN_MAX:
      return(tt532_RUN_max.number); 
    case CONFIG_PARAM_TT532_IDLE_MAX:
      return(tt532_IDLE_max.number); 
    case CONFIG_PARAM_TT532_STOP_MAX:
      return(tt532_STOP_max.number);        

     //TT-557
    case CONFIG_PARAM_TT557_STOP_MIN:
      return(tt557_STOP_min.number); 
    case CONFIG_PARAM_TT557_IDLE_MIN:
      return(tt557_IDLE_min.number); 
    case CONFIG_PARAM_TT557_RUN_MIN:
      return(tt557_RUN_min.number); 
    case CONFIG_PARAM_TT557_RUN_MAX:
      return(tt557_RUN_max.number); 
    case CONFIG_PARAM_TT557_IDLE_MAX:
      return(tt557_IDLE_max.number); 
    case CONFIG_PARAM_TT557_STOP_MAX:
      return(tt557_STOP_max.number);               

    case IDLE_STATE_S1_ALL:
      return(idle_s1_bits.number); 
    case IDLE_STATE_S2_ALL:
      return(idle_s2_bits.number); 
    case IDLE_STATE_S3_ALL:
      return(idle_s3_bits.number);   
    case CONFIG_PARAM_PULSE1:
      return(pulse1.number); 
    case CONFIG_PARAM_PULSE2:
      return(pulse2.number);   
    case CONFIG_PARAM_PULSE3:
      return(pulse3.number);             
    case CONFIG_PARAM_S1A_FLOW_COUNT:
      return(flowcnt1a.number); 
    case CONFIG_PARAM_S1B_FLOW_COUNT:
      return(flowcnt1b.number); 
    case CONFIG_PARAM_S2A_FLOW_COUNT:
      return(flowcnt2a.number); 
    case CONFIG_PARAM_S2B_FLOW_COUNT:
      return(flowcnt2b.number);    
    case CONFIG_PARAM_S3A_FLOW_COUNT:
      return(flowcnt3a.number); 
    case CONFIG_PARAM_S3B_FLOW_COUNT:
      return(flowcnt3b.number);   
    case CONFIG_PARAM_S1_ACTUAL_CPM:
      return(intensifier_s1_actual_cpm.number);
    case CONFIG_PARAM_S1_MIN_CPM:
      return(s1_min_cpm.number);  
    case CONFIG_PARAM_S1_MAX_CPM:
      return(s1_max_cpm.number);    
    case CONFIG_PARAM_S2_ACTUAL_CPM:
      return(intensifier_s2_actual_cpm.number); 
    case CONFIG_PARAM_S2_MIN_CPM:
      return(s2_min_cpm.number); 
    case CONFIG_PARAM_S2_MAX_CPM:
      return(s2_max_cpm.number); 
    case CONFIG_PARAM_S3_ACTUAL_CPM:
      return(intensifier_s3_actual_cpm.number); 
    case CONFIG_PARAM_S3_MIN_CPM:
      //return(s3_min_cpm.number); 
      return(135)
    case CONFIG_PARAM_S3_MAX_CPM:
      return(s3_max_cpm.number); 
    case ERROR_PARAM_ALL:
      return(error_bits.number);       
    case ERROR_PARAM2_ALL:
      return(error2_bits.number);                        
    default:
      return(-1);
      break;
  }
}

/*
    Name:         test_config_parameter
    Arguments:    int - The unique identifier of the parameter to be returned - as defined in config_c200.h
                  int - A 32-bit mask against which the parameter referenced by the previous integer is to be tested against. 
    Returns:      Bool -  True if the bit indicated by mask is set in the parameter referenced by param
                          False if the bit indicated by mask is clear in the parameter referenced by param
    Description:  Rather that having bit field test code all over the place it is more conveniant to have a single place in which to test specific 
                  bit fields. Only a small number of configuration parameters are bit field indicators. Others such as a sensor variable or threshold 
                  set point are not and will not pass the switch case statement below and return a defualt 0. 
*/
bool test_config_parameter(int param, int mask){
  switch(param){
    case SWITCH_STATE_ALL:
          return(sw1_bits.uint_value & (1 << mask));
    case CONFIG_PARAM_RELAYS:
          return(relays.uint_value & (1 << mask));
    case CONFIG_PARAM_INPUTS:
          return(inputs.uint_value & (1 << mask)); 
    case CONFIG_PARAM_OP_STATE:
          return(opstate.uint_value & (1 << mask));   
    case ERROR_PARAM_ALL:
          return(error_bits.uint_value & (1 << mask));         
    case IDLE_STATE_S1_ALL:
          return(idle_s1_bits.uint_value & (1 << mask));         
    case IDLE_STATE_S2_ALL:
          return(idle_s2_bits.uint_value & (1 << mask));         
    case IDLE_STATE_S3_ALL:
          return(idle_s3_bits.uint_value & (1 << mask));                           
    default:
      return(0);
  }  
}

/*
    Name:         get_config_parameter
    Arguments:    int - The unique identifier of the parameter to be returned - as defined in config_c200.h
                  int - A tri-state variable according to those values outlined in config_c200.h
                  MSB - The most significant byte of the 32-bit variable
                  LSB - The most significant byte of the 32-bit variable
                  AS_INT - Converts the 32-bit float to an 16-bit integer and returns the result. 
    Returns:      uin16_t - Value and meaning specifed by the arguments above
    Description:  For compatability with Modbus which works on a 16-bit architecture (words) it is sometimes useful to have a method to access 
                  the high and low words of each 32-bit variable. This enable eacy packing of a 32-bit field into 2x Modbus registers 
                  Alternatively AS_INT will simply convert the 32-bit value to a uint16_t for easy use as an integer during operation.
*/
uint16_t get_config_parameter(int param, int byte_n){
  uint16_t return_param = 0;
  FLOATUNION_t return_float;
  return_float.number = get_config_parameter(param);

  if (byte_n == MSB){
    return_param = return_float.words[0];
  }else if (byte_n == LSB){
    return_param = return_float.words[1];
  }else if (byte_n == AS_INT){    // As int
    return_param =  int(return_float.number);
  }else if (byte_n == AS_INT_32){//Added as sai has done in E100 32bit param write code
    return_param = return_float.uint_value;
  }
  return(return_param);
}

/*
    Name:         set_config_parameter
    Arguments:    int - The unique identifier of the parameter to be returned - as defined in config_c200.h
                  float - the new value of the specified parameter
    Returns:      nil
    Description:  Update the value of the float specified by param (According to config_c200.h)
*/
void set_config_parameter(int param, float param_value){

  if (isnan(param_value))  // Check if the param_value is NaN (Not a Number)
   return;  // Exit the function immediately if param_value is invalid (NaN)
  switch(param){
    case CONFIG_PARAM_SLAVE_ID:
      // Only slave ID numbers between 1 and 254 are valid. All others are to be ignored
      if ((param_value > 0) && (param_value < 255)){
        slave_id.number = param_value;
        // Any change to the SLAVE ID must trigger an EEPROM write, otherwise we may loose track
      }
      eeprom_save();
      break;    

    case CONFIG_PARAM_SERIAL_NUM:
      serial_number.number = param_value;
      eeprom_save();
      break; 
    case CONFIG_PARAM_PV1_CL:
      if (param_value > 100){
        param_value = 100; 
      }
      if (param_value < 0){
        param_value = 0;
      }
      pv1_cl.number = param_value;
      break; 
    case CONFIG_PARAM_PV2_CL:
      if (param_value > 100){
        param_value = 100; 
      }
      if (param_value < 0){
        param_value = 0;
      }
      pv2_cl.number = param_value;
      break;
    case CONFIG_PARAM_PV3_CL:
      if (param_value > 100){
        param_value = 100; 
      }
      if (param_value < 0){
        param_value = 0;
      }
      pv3_cl.number = param_value;
      break;
    case CONFIG_PARAM_PV4_CL:
      if (param_value > 100){
        param_value = 100; 
      }
      if (param_value < 0){
        param_value = 0;
      }
      pv4_cl.number = param_value;
      break;      
    case CONFIG_PARAM_RELAYS:               // Relays
      relays.number = param_value;
      break;
    case CONFIG_PARAM_INPUTS:               // Inputs
      inputs.number = param_value;
      break;  
    case SWITCH_STATE_ALL:               // DIP Switch
      sw1_bits.number = param_value;
      break;  
    case CONFIG_PARAM_TC1:               // TT-01
      tt01.number = param_value;
      break;
    case CONFIG_PARAM_TC2:               // TT-02
      tt02.number = param_value;
      break;
    case CONFIG_PARAM_TC3:               // TT-432
      tt03.number = param_value;
      break;
    case CONFIG_PARAM_TC4:               // TT-04
      tt04.number = param_value;
      break;
    case CONFIG_PARAM_TC5:               // TT-05
      tt05.number = param_value;
      break;  
    case CONFIG_PARAM_TC6:               // TT-06
      tt06.number = param_value;
      break;
    case CONFIG_PARAM_TC7:               // TT-07
      tt07.number = param_value;
      break;
    case CONFIG_PARAM_TC8:               // TT-08
      tt08.number = param_value;
      break;
    case CONFIG_PARAM_TC9:               // TT-09
      tt09.number = param_value;
      break;
    case CONFIG_PARAM_TC10:               // TT-10
      tt10.number = param_value;
      break;
    case CONFIG_PARAM_TC11:               // TT-11
      tt11.number = param_value;
      break;  
    case CONFIG_PARAM_TC12:               // TT-12
      tt12.number = param_value;
      break;
    case CONFIG_PARAM_TC13:               // TT-13
      tt13.number = param_value;
      break;  
    case CONFIG_PARAM_TC14:               // TT-14
      tt14.number = param_value;
      break;
    case CONFIG_PARAM_COOLANT1_PT:               // PT-01
      pt01.number = param_value;
      break;
    case CONFIG_PARAM_COOLANT2_PT:               // PT-02
      pt02.number = param_value;
      break;
    case CONFIG_PARAM_SUCTION_PT:               // PT-03
      pt03.number = param_value;
      break;
    case CONFIG_PARAM_SUCTION_TANK:               // PT-04
      pt04.number = param_value;
      break;
    case CONFIG_PARAM_STAGE1_PT:               // PT-05 -UNUSED
      pt05.number = param_value;
      break;  
    case CONFIG_PARAM_STAGE1_TANK:               // PT-06
      pt06.number = param_value;
      break;
    case CONFIG_PARAM_STAGE2_PT:               // PT-07  -UNUSED
      pt07.number = param_value;
      break;
    case CONFIG_PARAM_STAGE2_TANK:               // PT-08
      pt08.number = param_value;
      break;
    case CONFIG_PARAM_STAGE3_PT:                // PT-09
      pt09.number = param_value;
      break;  
    case CONFIG_PARAM_STAGE3_TANK:              // PT-10
      pt10.number = param_value;
      break; 
    case CONFIG_PARAM_HFL_PUMP1:                // PT-11
      pt11.number = param_value;
      break;     
    case CONFIG_PARAM_S1A_FLOW_COUNT:        //Intensifier 1-DirectionA Flowcount, Default:3000
      flowcnt1a.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_S1B_FLOW_COUNT:        //Intensifier 1-DirectionB Flowcount, Default:3000
      flowcnt1b.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_S2A_FLOW_COUNT:        //Intensifier 2-DirectionA Flowcount, Default:3000
      flowcnt2a.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_S2B_FLOW_COUNT:        //Intensifier 2-DirectionB Flowcount, Default:3000
      flowcnt2b.number = param_value;
      eeprom_save();
      break;    
    case CONFIG_PARAM_S3A_FLOW_COUNT:        //Intensifier 3-DirectionA Flowcount, Default:3000
      flowcnt3a.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_S3B_FLOW_COUNT:        //Intensifier 3-DirectionB Flowcount, Default:3000
      flowcnt3b.number = param_value;
      eeprom_save();
      break;      
    case CONFIG_PARAM_PT405_STOP_MIN:               // 405 STOP MIN
      pt405_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT405_IDLE_MIN:               // 405 IDLE MIN
      pt405_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT405_RUN_MIN:               // 405 RUN MIN
      pt405_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_PT405_RUN_MAX:               // 405 RUN MAX
      pt405_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT405_IDLE_MAX:               // 405 IDLE MAX
      pt405_IDLE_max.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_PT405_STOP_MAX:               // 405 STOP MAX
      pt405_STOP_max.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_PT426_STOP_MIN:               // 426 STOP MIN
      pt426_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT426_IDLE_MIN:               // 426 IDLE MIN
      pt426_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT426_RUN_MIN:               // 426 RUN MIN
      pt426_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_PT426_RUN_MAX:               // 426 RUN MAX
      pt426_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT426_IDLE_MAX:               // 426 IDLE MAX
      pt426_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_PT426_STOP_MAX:               // 426 STOP MAX
      pt426_STOP_max.number = param_value;
      eeprom_save();
      break;    
    case CONFIG_PARAM_PT442_STOP_MIN:               // 442 STOP MIN
      pt442_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT442_IDLE_MIN:               // 442 IDLE MIN
      pt442_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT442_RUN_MIN:               // 442 RUN MIN
      pt442_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_PT442_RUN_MAX:               // 442 RUN MAX
      pt442_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT442_IDLE_MAX:               // 442 IDLE MAX
      pt442_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_PT442_STOP_MAX:               // 442 STOP MAX
      pt442_STOP_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT410_STOP_MIN:               // 410 STOP MIN
      pt410_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT410_IDLE_MIN:               // 410 IDLE MIN
      pt410_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT410_RUN_MIN:               // 410 RUN MIN
      pt410_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_PT410_RUN_MAX:               // 410 RUN MAX
      pt410_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT410_IDLE_MAX:               // 410 IDLE MAX
      pt410_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_PT410_STOP_MAX:               // 410 STOP MAX
      pt410_STOP_max.number = param_value;
      eeprom_save();  
      break;      
    case CONFIG_PARAM_PT457_STOP_MIN:               // 457 STOP MIN
      pt457_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT457_IDLE_MIN:               // 457 IDLE MIN
      pt457_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT457_RUN_MIN:               // 457 RUN MIN
      pt457_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_PT457_RUN_MAX:               // 457 RUN MAX
      pt457_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT457_IDLE_MAX:               // 457 IDLE MAX
      pt457_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_PT457_STOP_MAX:               // 457 STOP MAX
      pt457_STOP_max.number = param_value;
      eeprom_save();  
      break;   
    case CONFIG_PARAM_PT416_STOP_MIN:               // 416 STOP MIN
      pt416_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT416_IDLE_MIN:               // 416 IDLE MIN
      pt416_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT416_RUN_MIN:               // 416 RUN MIN
      pt416_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_PT416_RUN_MAX:               // 416 RUN MAX
      pt416_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT416_IDLE_MAX:               // 416 IDLE MAX
      pt416_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_PT416_STOP_MAX:               // 416 STOP MAX
      pt416_STOP_max.number = param_value;
      eeprom_save();  
      break;  

    case CONFIG_PARAM_PT468_STOP_MIN:               // 468 STOP MIN
      pt468_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT468_IDLE_MIN:               // 468 IDLE MIN
      pt468_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT468_RUN_MIN:               // 468 RUN MIN
      pt468_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_PT468_RUN_MAX:               // 468 RUN MAX
      pt468_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT468_IDLE_MAX:               // 468 IDLE MAX
      pt468_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_PT468_STOP_MAX:               // 468 STOP MAX
      pt468_STOP_max.number = param_value;
      eeprom_save();  
      break;      

    case CONFIG_PARAM_PT487_STOP_MIN:               // 487 STOP MIN
      pt487_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT487_IDLE_MIN:               // 487 IDLE MIN
      pt487_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT487_RUN_MIN:               // 487 RUN MIN
      pt487_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_PT487_RUN_MAX:               // 487 RUN MAX
      pt487_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT487_IDLE_MAX:               // 487 IDLE MAX
      pt487_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_PT487_STOP_MAX:               // 487 STOP MAX
      pt487_STOP_max.number = param_value;
      eeprom_save();  
      break;      

    case CONFIG_PARAM_PT496_STOP_MIN:              // 496 STOP MIN
      pt496_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT496_IDLE_MIN:              // 496 IDLE MIN
      pt496_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT496_RUN_MIN:               // 496 RUN MIN
      pt496_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_PT496_RUN_MAX:               // 496 RUN MAX
      pt496_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT496_IDLE_MAX:              // 496 IDLE MAX
      pt496_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_PT496_STOP_MAX:              // 496 STOP MAX
      pt496_STOP_max.number = param_value;
      eeprom_save();  
      break;          

    case CONFIG_PARAM_PT531_STOP_MIN:               // 531 STOP MIN
      pt531_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT531_IDLE_MIN:               // 531 IDLE MIN
      pt531_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT531_RUN_MIN:               // 531 RUN MIN
      pt531_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_PT531_RUN_MAX:               // 531 RUN MAX
      pt531_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT531_IDLE_MAX:               // 531 IDLE MAX
      pt531_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_PT531_STOP_MAX:               // 531 STOP MAX
      pt531_STOP_max.number = param_value;
      eeprom_save();  
      break;  

    case CONFIG_PARAM_PT576_STOP_MIN:               // 576 STOP MIN
      pt576_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT576_IDLE_MIN:               // 576 IDLE MIN
      pt576_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT576_RUN_MIN:               // 576 RUN MIN
      pt576_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_PT576_RUN_MAX:               // 576 RUN MAX
      pt576_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_PT576_IDLE_MAX:               // 576 IDLE MAX
      pt576_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_PT576_STOP_MAX:               // 576 STOP MAX
      pt576_STOP_max.number = param_value;
      eeprom_save();  
      break;                

    case CONFIG_PARAM_TT432_STOP_MIN:               // TT432 STOP MIN
      tt432_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT432_IDLE_MIN:               // TT432 IDLE MIN
      tt432_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT432_RUN_MIN:               // TT432 RUN MIN
      tt432_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_TT432_RUN_MAX:               // TT432 RUN MAX
      tt432_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT432_IDLE_MAX:               // TT432 IDLE MAX
      tt432_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_TT432_STOP_MAX:               // TT432 STOP MAX
      tt432_STOP_max.number = param_value;
      eeprom_save();  
      break;                  

    case CONFIG_PARAM_TT439_STOP_MIN:               // TT439 STOP MIN
      tt439_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT439_IDLE_MIN:               // TT439 IDLE MIN
      tt439_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT439_RUN_MIN:               // TT439 RUN MIN
      tt439_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_TT439_RUN_MAX:               // TT439 RUN MAX
      tt439_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT439_IDLE_MAX:               // TT439 IDLE MAX
      tt439_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_TT439_STOP_MAX:               // TT439 STOP MAX
      tt439_STOP_max.number = param_value;
      eeprom_save();  
      break;            

    case CONFIG_PARAM_TT435_STOP_MIN:               // TT435 STOP MIN
      tt435_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT435_IDLE_MIN:               // TT435 IDLE MIN
      tt435_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT435_RUN_MIN:               // TT435 RUN MIN
      tt435_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_TT435_RUN_MAX:               // TT435 RUN MAX
      tt435_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT435_IDLE_MAX:               // TT435 IDLE MAX
      tt435_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_TT435_STOP_MAX:               // TT435 STOP MAX
      tt435_STOP_max.number = param_value;
      eeprom_save();  
      break;       

    case CONFIG_PARAM_TT436_STOP_MIN:               // TT436 STOP MIN
      tt436_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT436_IDLE_MIN:               // TT436 IDLE MIN
      tt436_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT436_RUN_MIN:               // TT436 RUN MIN
      tt436_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_TT436_RUN_MAX:               // TT436 RUN MAX
      tt436_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT436_IDLE_MAX:               // TT436 IDLE MAX
      tt436_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_TT436_STOP_MAX:               // TT436 STOP MAX
      tt436_STOP_max.number = param_value;
      eeprom_save();  
      break;         

    case CONFIG_PARAM_TT447_STOP_MIN:               // TT447 STOP MIN
      tt447_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT447_IDLE_MIN:               // TT447 IDLE MIN
      tt447_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT447_RUN_MIN:               // TT447 RUN MIN
      tt447_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_TT447_RUN_MAX:               // TT447 RUN MAX
      tt447_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT447_IDLE_MAX:               // TT447 IDLE MAX
      tt447_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_TT447_STOP_MAX:               // TT447 STOP MAX
      tt447_STOP_max.number = param_value;
      eeprom_save();  
      break;              

    case CONFIG_PARAM_TT454_STOP_MIN:               // TT454 STOP MIN
      tt454_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT454_IDLE_MIN:               // TT454 IDLE MIN
      tt454_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT454_RUN_MIN:               // TT454 RUN MIN
      tt454_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_TT454_RUN_MAX:               // TT454 RUN MAX
      tt454_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT454_IDLE_MAX:               // TT454 IDLE MAX
      tt454_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_TT454_STOP_MAX:               // TT454 STOP MAX
      tt454_STOP_max.number = param_value;
      eeprom_save();  
      break;                   

    case CONFIG_PARAM_TT450_STOP_MIN:               // TT450 STOP MIN
      tt450_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT450_IDLE_MIN:               // TT450 IDLE MIN
      tt450_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT450_RUN_MIN:               // TT450 RUN MIN
      tt450_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_TT450_RUN_MAX:               // TT450 RUN MAX
      tt450_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT450_IDLE_MAX:               // TT450 IDLE MAX
      tt450_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_TT450_STOP_MAX:               // TT450 STOP MAX
      tt450_STOP_max.number = param_value;
      eeprom_save();  
      break;                                           

    case CONFIG_PARAM_TT451_STOP_MIN:               // TT451 STOP MIN
      tt451_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT451_IDLE_MIN:               // TT451 IDLE MIN
      tt451_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT451_RUN_MIN:               // TT451 RUN MIN
      tt451_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_TT451_RUN_MAX:               // TT451 RUN MAX
      tt451_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT451_IDLE_MAX:               // TT451 IDLE MAX
      tt451_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_TT451_STOP_MAX:               // TT451 STOP MAX
      tt451_STOP_max.number = param_value;
      eeprom_save();  
      break;                       

    case CONFIG_PARAM_TT462_STOP_MIN:               // TT462 STOP MIN
      tt462_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT462_IDLE_MIN:               // TT462 IDLE MIN
      tt462_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT462_RUN_MIN:               // TT462 RUN MIN
      tt462_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_TT462_RUN_MAX:               // TT462 RUN MAX
      tt462_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT462_IDLE_MAX:               // TT462 IDLE MAX
      tt462_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_TT462_STOP_MAX:               // TT462 STOP MAX
      tt462_STOP_max.number = param_value;
      eeprom_save();  
      break;         
      
    case CONFIG_PARAM_TT474_STOP_MIN:               // TT474 STOP MIN
      tt474_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT474_IDLE_MIN:               // TT474 IDLE MIN
      tt474_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT474_RUN_MIN:               // TT474 RUN MIN
      tt474_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_TT474_RUN_MAX:               // TT474 RUN MAX
      tt474_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT474_IDLE_MAX:               // TT474 IDLE MAX
      tt474_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_TT474_STOP_MAX:               // TT474 STOP MAX
      tt474_STOP_max.number = param_value;
      eeprom_save();  
      break;                                                           

    case CONFIG_PARAM_TT465_STOP_MIN:               // TT465 STOP MIN
      tt465_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT465_IDLE_MIN:               // TT465 IDLE MIN
      tt465_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT465_RUN_MIN:               // TT465 RUN MIN
      tt465_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_TT465_RUN_MAX:               // TT465 RUN MAX
      tt465_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT465_IDLE_MAX:               // TT465 IDLE MAX
      tt465_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_TT465_STOP_MAX:               // TT465 STOP MAX
      tt465_STOP_max.number = param_value;
      eeprom_save();  
      break; 

    case CONFIG_PARAM_TT471_STOP_MIN:               // TT471 STOP MIN
      tt471_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT471_IDLE_MIN:               // TT471 IDLE MIN
      tt471_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT471_RUN_MIN:               // TT471 RUN MIN
      tt471_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_TT471_RUN_MAX:               // TT471 RUN MAX
      tt471_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT471_IDLE_MAX:               // TT471 IDLE MAX
      tt471_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_TT471_STOP_MAX:               // TT471 STOP MAX
      tt471_STOP_max.number = param_value;
      eeprom_save();  
      break;   

    case CONFIG_PARAM_TT532_STOP_MIN:               // TT532 STOP MIN
      tt532_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT532_IDLE_MIN:               // TT532 IDLE MIN
      tt532_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT532_RUN_MIN:               // TT532 RUN MIN
      tt532_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_TT532_RUN_MAX:               // TT532 RUN MAX
      tt532_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT532_IDLE_MAX:               // TT532 IDLE MAX
      tt532_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_TT532_STOP_MAX:               // TT532 STOP MAX
      tt532_STOP_max.number = param_value;
      eeprom_save();  
      break;     

    case CONFIG_PARAM_TT557_STOP_MIN:               // TT557 STOP MIN
      tt557_STOP_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT557_IDLE_MIN:               // TT557 IDLE MIN
      tt557_IDLE_min.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT557_RUN_MIN:               // TT557 RUN MIN
      tt557_RUN_min.number = param_value;
      eeprom_save();
      break;  
    case CONFIG_PARAM_TT557_RUN_MAX:               // TT557 RUN MAX
      tt557_RUN_max.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_TT557_IDLE_MAX:               // TT557 IDLE MAX
      tt557_IDLE_max.number = param_value;
      eeprom_save(); 
      break;  
    case CONFIG_PARAM_TT557_STOP_MAX:               // TT557 STOP MAX
      tt557_STOP_max.number = param_value;
      eeprom_save();  
      break;       

    case CONFIG_PARAM_PULSE1:               // Pulse1
      pulse1.number = param_value;
      break;
    case CONFIG_PARAM_PULSE2:               // Pulse2
      pulse2.number = param_value;
      break;
    case CONFIG_PARAM_PULSE3:               // Pulse3
      pulse3.number = param_value;
      break;    
    case ERROR_PARAM_ALL:
      error_bits.number = param_value;
      break;  
    case ERROR_PARAM2_ALL:
      error2_bits.number = param_value;
      break;   
    case CONFIG_PARAM_S1_ACTUAL_CPM:
      intensifier_s1_actual_cpm.number = param_value;
      break;
    case CONFIG_PARAM_S1_MIN_CPM:
      s1_min_cpm.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_S1_MAX_CPM:
      s1_max_cpm.number = param_value;
      eeprom_save();
      break;
    case CONFIG_PARAM_S2_ACTUAL_CPM:
      intensifier_s2_actual_cpm.number = param_value;
      break;    
    case CONFIG_PARAM_S2_MIN_CPM:
      s2_min_cpm.number = param_value;
      eeprom_save();
      break; 
    case CONFIG_PARAM_S2_MAX_CPM:
      s2_max_cpm.number = param_value;
      eeprom_save();
      break; 
    case CONFIG_PARAM_S3_ACTUAL_CPM:
      intensifier_s3_actual_cpm.number = param_value;
      break;
    case CONFIG_PARAM_S3_MIN_CPM:
      s3_min_cpm.number = param_value;
      eeprom_save();
      break; 
    case CONFIG_PARAM_S3_MAX_CPM:
      s3_max_cpm.number = param_value;
      eeprom_save();
      break;  

    default:
      break;
  }
}

/*
    Name:         set_config_parameter
    Arguments:    int - The unique identifier of the parameter to be returned - as defined in config_c200.h
                  uint16_t - The new value to be set 
                  int - A tri-state variable according to those values outlined in config_c200.h
                  MSB - The most significant byte of the 32-bit variable
                  LSB - The most significant byte of the 32-bit variable
                  AS_INT - Converts the 32-bit float to an 16-bit integer and returns the result. 
    Returns:      nil
    Description:  Because Modbus is built on 16-bit architecture (words) it is sometimes useful if using a 32-bit 
                  variable to update each word separatly because this is how Modbus will do it - in 2x writes, one to 
                  each word. Not all variables operate in this way and so only a sub-set is available here. 
*/
void set_config_parameter(int param, uint16_t param_value, int byte_n){
  switch(param){
    case CONFIG_PARAM_RELAYS:
      if (byte_n == MSB){
        relays.words[0] = param_value;
      }else{
        relays.words[1] = param_value;
      }
      break;

    case CONFIG_PARAM_OP_STATE:
      if (byte_n == MSB){
        opstate.words[0] = param_value;
      }else{
        opstate.words[1] = param_value;
      }
      break;

    case CONFIG_PARAM_INPUTS:
      if (byte_n == MSB){
        inputs.words[0] = param_value;
      }else{
        inputs.words[1] = param_value;
      }
      break;

    case CONFIG_PARAM_SERIAL_NUM:
      if (byte_n == MSB){
        serial_number.words[0] = param_value;
      }else{
        serial_number.words[1] = param_value;
      }
      break;
  
    case ERROR_PARAM_ALL:
      if (byte_n == MSB){
        error_bits.words[0] = param_value;
      }else{
        error_bits.words[1] = param_value;
      }
      break;

    case ERROR_PARAM2_ALL:
      if (byte_n == MSB){
        error2_bits.words[0] = param_value;
      }else{
        error2_bits.words[1] = param_value;
      }
      break;

    case IDLE_STATE_S1_ALL:
      if (byte_n == MSB){
        idle_s1_bits.words[0] = param_value;
      }else{
        idle_s1_bits.words[1] = param_value;
      }
      break;

    case IDLE_STATE_S2_ALL:
      if (byte_n == MSB){
        idle_s2_bits.words[0] = param_value;
      }else{
        idle_s2_bits.words[1] = param_value;
      }
      break;

    case IDLE_STATE_S3_ALL:
      if (byte_n == MSB){
        idle_s3_bits.words[0] = param_value;
      }else{
        idle_s3_bits.words[1] = param_value;
      }
      break;      

    case CONFIG_PARAM_S1_ACTUAL_CPM:
      if (byte_n == MSB){
        intensifier_s1_actual_cpm.words[0] = param_value;
      }else{
        intensifier_s1_actual_cpm.words[1] = param_value;
      }
      break;  
    case CONFIG_PARAM_S1_MIN_CPM:
      if (byte_n == MSB){
        s1_min_cpm.words[0] = param_value;
      }else{
        s1_min_cpm.words[1] = param_value;
      }
      eeprom_save();
      break; 
    case CONFIG_PARAM_S1_MAX_CPM:
      if (byte_n == MSB){
        s1_max_cpm.words[0] = param_value;
      }else{
        s1_max_cpm.words[1] = param_value;
      }
      eeprom_save();
      break;  

    case CONFIG_PARAM_S2_ACTUAL_CPM:
      if (byte_n == MSB){
        intensifier_s2_actual_cpm.words[0] = param_value;
      }else{
        intensifier_s2_actual_cpm.words[1] = param_value;
      }
      break;   
    case CONFIG_PARAM_S2_MIN_CPM:
      if (byte_n == MSB){
        s2_min_cpm.words[0] = param_value;
      }else{
        s2_min_cpm.words[1] = param_value;
      }
      eeprom_save();
      break;  
    case CONFIG_PARAM_S2_MAX_CPM:
      if (byte_n == MSB){
        s2_max_cpm.words[0] = param_value;
      }else{
        s2_max_cpm.words[1] = param_value;
      }
      eeprom_save();
      break;  

    case CONFIG_PARAM_S3_ACTUAL_CPM:
      if (byte_n == MSB){
        intensifier_s3_actual_cpm.words[0] = param_value;
      }else{
        intensifier_s3_actual_cpm.words[1] = param_value;
      }
      break;  
    case CONFIG_PARAM_S3_MIN_CPM:
      if (byte_n == MSB){
        s3_min_cpm.words[0] = param_value;
      }else{
        s3_min_cpm.words[1] = param_value;
      }
      eeprom_save();
      break;  
    case CONFIG_PARAM_S3_MAX_CPM:
      if (byte_n == MSB){
        s3_max_cpm.words[0] = param_value;
      }else{
        s3_max_cpm.words[1] = param_value;
      }
      eeprom_save();
      break;          
    default:
      break;
  }
}

void set_config_bit(int param, uint16_t param_value, int bit_n){
  switch(param){
    case CONFIG_PARAM_RELAYS:
      if (param_value){
        relays.uint_value |= 1 << bit_n;
      }else{
        relays.uint_value &= ~(1 << bit_n);
      }
      break;    
    case CONFIG_PARAM_INPUTS:
      if (param_value){
        inputs.uint_value |= 1 << bit_n;
      }else{
        inputs.uint_value &= ~(1 << bit_n);
      }
      break;
    case CONFIG_PARAM_OP_STATE:
      if (param_value){
        opstate.uint_value |= 1 << bit_n;
      }else{
        opstate.uint_value &= ~(1 << bit_n);
      }
      break;
    case ERROR_PARAM_ALL:
      if (param_value){
        error_bits.uint_value |= 1 << bit_n;
      }else{
        error_bits.uint_value &= ~(1 << bit_n);
      }
      break;       
    case ERROR_PARAM2_ALL:
      if (param_value){
        error2_bits.uint_value |= 1 << bit_n;
      }else{
        error2_bits.uint_value &= ~(1 << bit_n);
      }
      break;            
    case IDLE_STATE_S1_ALL:
      if (param_value){
        idle_s1_bits.uint_value |= 1 << bit_n;
      }else{
        idle_s1_bits.uint_value &= ~(1 << bit_n);
      }
      break;         
    case IDLE_STATE_S2_ALL:
      if (param_value){
        idle_s2_bits.uint_value |= 1 << bit_n;
      }else{
        idle_s2_bits.uint_value &= ~(1 << bit_n);
      }
      break;         
    case IDLE_STATE_S3_ALL:
      if (param_value){
        idle_s3_bits.uint_value |= 1 << bit_n;
      }else{
        idle_s3_bits.uint_value &= ~(1 << bit_n);
      }
      break;   
    case SWITCH_STATE_ALL:
      if (param_value){
        sw1_bits.uint_value |= 1 << bit_n;
      }else{
        sw1_bits.uint_value &= ~(1 << bit_n);
      }
      break;  

  }
}

