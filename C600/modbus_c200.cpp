/*
    Author: Christopher Glanville
    Description:    This function handles all of the Modbus communications associated with the Global485, BLE & WiFi. Although Arduino does have libraries for this
                    have I found that most of them accept a serial only connection which prevents those libraries from being used on TCP and BLE connections. I have also 
                    found that finer control over timeouts and processing blocks (Read/write) is not available and so it was more convenient for me to use my own raw code.
                    This class accepts input from 3x interfaces and stores the data into a buffer for each. This allows quick and non-blocking reception of new data bytes 
                    that can later be processed in their own loop call functions. 
   Date: 26/5/24
   updated : 26/5/24
*/

#include "C200.h"
#include "io_c200.h"
#include "config_c200.h"
#include "modbus_c200.h"

#include <WiFi.h>

#define TCP_SERIAL_BUFFER_LIMIT       256
#define SLAVE_ID                      1
#define PACKET_CHECK_ONLY             1
#define PACKET_CRC_INSERT             0
uint8_t TCPSerialBufferRx[TCP_SERIAL_BUFFER_LIMIT];
uint8_t TCPSerialBufferTx[TCP_SERIAL_BUFFER_LIMIT];
uint8_t RSSerialBufferRx[TCP_SERIAL_BUFFER_LIMIT];
uint8_t RSSerialBufferTx[TCP_SERIAL_BUFFER_LIMIT];
uint8_t BLESerialBufferRx[TCP_SERIAL_BUFFER_LIMIT];
uint8_t BLESerialBufferTx[TCP_SERIAL_BUFFER_LIMIT];
int TCPSerialBufferPt = 0; 
int RSSerialBufferPt = 0; 
int BLESerialBufferPt = 0; 
int tcp_tx_bytes = 0;
int rs_tx_bytes = 0;
int ble_tx_bytes = 0;

/*
    Name:         CRC16
    Arguments:    uint8_t*    A poiner to the data being investigation - A MODBUS packet for either transmission (CRC insertion) or reception 
                              (CRC checking)
                  int         An integer length of the data packet. This length is the data length only and does not include any CRC length
                  char        A character indicator as to if this function should check for a valid CRC (receive) or insert the calculated CRC
                              (transmit) 
    Returns:      bool        Always true for transmit when the CRC is calculated and inserted 
                              In receive this will be TRUE if the CRC that was calculated matched the final two bytes after the specified integer 
                              data length or FALSE if those bytes do NOT match the CRC that was calculated matched the final two bytes.
    Description:  Calculate the CRC of a packet and either test it against the CRC received in said packet (For receive) or insert the CRC into the 
                  packet for sending (Used in transmission of packets). 
*/
bool CRC16(uint8_t * packet, int dataLength, char checkElseInsert) //CRC 16 for modbus checksum
{
    unsigned int CheckSum;
    unsigned int j;
    unsigned char lowCRC, highCRC;
    unsigned short i;

    CheckSum = 0xFFFF;
    for (j = 0; j < dataLength; j++)
    {
        CheckSum = CheckSum ^ (unsigned int)packet[j];
        for(i = 8; i > 0; i--)
            if((CheckSum) & 0x0001)
                CheckSum = (CheckSum >> 1) ^ 0xa001;
            else
                CheckSum >>= 1;
    }
    highCRC = (CheckSum >> 8) & 0xFF;
    lowCRC = (CheckSum & 0xFF);

    if (checkElseInsert == 1){
        if ((packet[dataLength+1] == highCRC) & (packet[dataLength] == lowCRC ))
            return 1;
        else
            return 0;
    }else{
        packet[dataLength] = lowCRC;
        packet[dataLength+1] = highCRC;
        return 1;
    }  
}

/*
    Name:         readRegResponse
    Arguments:    int       The MODBUS register that is no be read from
    Returns:      uint16_t  The corresponding value of the register that was requested in the argument integer i
    Description:  Return the data requested in register i. Most data is accessed from the config_c200 class. This function will be 
                  called from the modbus_loop function once a valid packet is received. When reading multiple registers this function 
                  is called in succession with an increasing register integer i.
*/
uint16_t readRegResponse(int i){
  float temp_val;
  uint16_t reg_response_val = 0;

  switch(i){
    case MODBUS_REG_RELAYS:
      temp_val = get_config_parameter(CONFIG_PARAM_RELAYS, MSB);
      reg_response_val = (uint16_t)temp_val;
      break;
    case MODBUS_REG_RELAYS+1:
      temp_val = get_config_parameter(CONFIG_PARAM_RELAYS, LSB);
      reg_response_val = (uint16_t)temp_val;
      break;  
    case MODBUS_REG_SWITCH_ALL:  
      temp_val = get_config_parameter(SWITCH_STATE_ALL, MSB);
      reg_response_val = (uint16_t)temp_val;
      break;  
    case MODBUS_REG_IDLE_STATE_S1_ALL:
      temp_val = get_config_parameter(IDLE_STATE_S1_ALL, MSB);
      reg_response_val = (uint16_t)temp_val;
      break;
    case MODBUS_REG_IDLE_STATE_S1_ALL+1:
      temp_val = get_config_parameter(IDLE_STATE_S1_ALL, LSB);
      reg_response_val = (uint16_t)temp_val;
      break;    
    case MODBUS_REG_IDLE_STATE_S2_ALL:
      temp_val = get_config_parameter(IDLE_STATE_S2_ALL, MSB);
      reg_response_val = (uint16_t)temp_val;
      break;
    case MODBUS_REG_IDLE_STATE_S2_ALL+1:
      temp_val = get_config_parameter(IDLE_STATE_S2_ALL, LSB);
      reg_response_val = (uint16_t)temp_val;
      break; 
    case MODBUS_REG_IDLE_STATE_S3_ALL:
      temp_val = get_config_parameter(IDLE_STATE_S3_ALL, MSB);
      reg_response_val = (uint16_t)temp_val;
      break;
    case MODBUS_REG_IDLE_STATE_S3_ALL+1:
      temp_val = get_config_parameter(IDLE_STATE_S3_ALL, LSB);
      reg_response_val = (uint16_t)temp_val;
      break;     
    case MODBUS_REG_INPUTS:  
      temp_val = get_config_parameter(CONFIG_PARAM_INPUTS, MSB);
      reg_response_val = (uint16_t)temp_val;
      break; 
    case MODBUS_REG_INPUTS+1:  
      temp_val = get_config_parameter(CONFIG_PARAM_INPUTS, LSB);
      reg_response_val = (uint16_t)temp_val;
      break;   
    case MODBUS_REG_SLAVE_ID:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_SLAVE_ID);
      break;  
    case MODBUS_REG_SERIAL_NUM:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_SERIAL_NUM, MSB);
      break;
    case MODBUS_REG_SERIAL_NUM+1:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_SERIAL_NUM, LSB);
      break;
     case MODBUS_REG_PV1_CL:
      reg_response_val = get_config_parameter(CONFIG_PARAM_PV1_CL);
      break;
    case MODBUS_REG_PV2_CL:
      reg_response_val = get_config_parameter(CONFIG_PARAM_PV2_CL);
      break;
    case MODBUS_REG_PV3_CL:
      reg_response_val = get_config_parameter(CONFIG_PARAM_PV3_CL);
      break;
    case MODBUS_REG_PV4_CL:
      reg_response_val = get_config_parameter(CONFIG_PARAM_PV4_CL);
      break;
    case MODBUS_REG_COOLANT1_PT:       
      reg_response_val = get_config_parameter(CONFIG_PARAM_COOLANT1_PT, AS_INT);
      break;
    case MODBUS_REG_COOLANT2_PT:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_COOLANT2_PT, AS_INT);
      break;    
    case MODBUS_REG_SUCTION_PT:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_SUCTION_PT, AS_INT);
      break;
    case MODBUS_REG_SUCTION_TANK:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_SUCTION_TANK, AS_INT);
      break;
    case MODBUS_REG_STAGE1_PT:  //UNUSED
      reg_response_val = get_config_parameter(CONFIG_PARAM_STAGE1_PT, AS_INT);
      break;
    case MODBUS_REG_STAGE1_TANK:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_STAGE1_TANK, AS_INT);
      break;  
    case MODBUS_REG_STAGE2_PT:   //UNUSED
      reg_response_val = get_config_parameter(CONFIG_PARAM_STAGE2_PT, AS_INT);
      break;
    case MODBUS_REG_STAGE2_TANK:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_STAGE2_TANK, AS_INT);
      break;
    case MODBUS_REG_STAGE3_PT:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_STAGE3_PT, AS_INT);
      break;  
    case MODBUS_REG_STAGE3_TANK:
      reg_response_val = get_config_parameter(CONFIG_PARAM_STAGE3_TANK, AS_INT);
      break;  
    case MODBUS_REG_HFL_PUMP1:
      reg_response_val = get_config_parameter(CONFIG_PARAM_HFL_PUMP1, AS_INT);
      break;  
    case MODBUS_REG_TC1:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TC1);
      break;  
    case MODBUS_REG_TC2:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TC2);
      break; 
    case MODBUS_REG_TC3:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TC3);
      break; 
    case MODBUS_REG_TC4:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TC4);
      break; 
    case MODBUS_REG_TC5:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TC5);
      break; 
    case MODBUS_REG_TC6:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TC6);
      break;          
    case MODBUS_REG_TC7:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TC7);
      break;  
    case MODBUS_REG_TC8:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TC8);
      break; 
    case MODBUS_REG_TC9:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TC9);
      break; 
    case MODBUS_REG_TC10:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TC10);
      break; 
    case MODBUS_REG_TC11:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TC11);
      break; 
    case MODBUS_REG_TC12:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TC12);
      break;
    case MODBUS_REG_TC13:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TC13);
      break; 
    case MODBUS_REG_TC14:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TC14);
      break;                
    case MODBUS_REG_PULSE1:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PULSE1);
      break;
    case MODBUS_REG_PULSE2:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PULSE2);
      break; 
    case MODBUS_REG_PULSE3:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PULSE3);
      break;                
    //PT-405  
    case MODBUS_REG_PT405_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT405_STOP_MIN, AS_INT);
      break;  
    case MODBUS_REG_PT405_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT405_IDLE_MIN, AS_INT);
      break; 
    case MODBUS_REG_PT405_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT405_RUN_MIN, AS_INT);
      break; 
    case MODBUS_REG_PT405_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT405_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_PT405_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT405_IDLE_MAX, AS_INT);
      break; 
    case MODBUS_REG_PT405_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT405_STOP_MAX, AS_INT);
      break;          
    //PT-426   
    case MODBUS_REG_PT426_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT426_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_PT426_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT426_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_PT426_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT426_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_PT426_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT426_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_PT426_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT426_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_PT426_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT426_STOP_MAX, AS_INT);
      break;            
     //PT-442   
    case MODBUS_REG_PT442_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT442_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_PT442_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT442_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_PT442_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT442_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_PT442_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT442_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_PT442_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT442_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_PT442_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT442_STOP_MAX, AS_INT);
      break;       
    //PT-410  
    case MODBUS_REG_PT410_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT410_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_PT410_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT410_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_PT410_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT410_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_PT410_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT410_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_PT410_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT410_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_PT410_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT410_STOP_MAX, AS_INT);
      break;             
    //PT-457
    case MODBUS_REG_PT457_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT457_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_PT457_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT457_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_PT457_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT457_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_PT457_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT457_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_PT457_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT457_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_PT457_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT457_STOP_MAX, AS_INT);
      break;             
      //PT-416
    case MODBUS_REG_PT416_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT416_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_PT416_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT416_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_PT416_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT416_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_PT416_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT416_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_PT416_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT416_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_PT416_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT416_STOP_MAX, AS_INT);
      break;           
    //PT-468
    case MODBUS_REG_PT468_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT468_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_PT468_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT468_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_PT468_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT468_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_PT468_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT468_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_PT468_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT468_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_PT468_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT468_STOP_MAX, AS_INT);
      break; 
    //PT-487
    case MODBUS_REG_PT487_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT487_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_PT487_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT487_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_PT487_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT487_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_PT487_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT487_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_PT487_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT487_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_PT487_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT487_STOP_MAX, AS_INT);
      break;                       
      //PT-531
    case MODBUS_REG_PT531_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT531_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_PT531_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT531_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_PT531_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT531_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_PT531_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT531_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_PT531_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT531_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_PT531_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT531_STOP_MAX, AS_INT);
      break; 
      //PT-576
    case MODBUS_REG_PT576_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT576_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_PT576_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT576_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_PT576_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT576_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_PT576_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT576_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_PT576_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT576_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_PT576_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT576_STOP_MAX, AS_INT);
      break; 
    //PT-496
    case MODBUS_REG_PT496_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT496_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_PT496_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT496_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_PT496_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT496_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_PT496_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT496_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_PT496_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT496_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_PT496_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT496_STOP_MAX, AS_INT);
      break;      

     //TT-432
    case MODBUS_REG_TT432_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT432_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_TT432_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT432_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_TT432_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT432_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_TT432_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT432_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_TT432_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT432_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_TT432_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT432_STOP_MAX, AS_INT);
      break;   

     //TT-439
    case MODBUS_REG_TT439_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT439_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_TT439_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT439_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_TT439_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT439_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_TT439_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT439_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_TT439_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT439_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_TT439_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT439_STOP_MAX, AS_INT);
      break;  

     //TT-435
    case MODBUS_REG_TT435_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT435_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_TT435_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT435_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_TT435_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT435_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_TT435_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT435_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_TT435_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT435_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_TT435_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT435_STOP_MAX, AS_INT);
      break;        

     //TT-436
    case MODBUS_REG_TT436_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT436_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_TT436_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT436_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_TT436_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT436_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_TT436_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT436_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_TT436_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT436_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_TT436_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT436_STOP_MAX, AS_INT);
      break;        

    //TT-447
    case MODBUS_REG_TT447_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT447_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_TT447_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT447_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_TT447_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT447_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_TT447_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT447_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_TT447_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT447_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_TT447_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT447_STOP_MAX, AS_INT);
      break;              

    //TT-454
    case MODBUS_REG_TT454_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT454_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_TT454_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT454_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_TT454_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT454_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_TT454_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT454_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_TT454_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT454_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_TT454_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT454_STOP_MAX, AS_INT);
      break;        

    //TT-450
    case MODBUS_REG_TT450_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT450_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_TT450_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT450_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_TT450_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT450_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_TT450_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT450_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_TT450_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT450_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_TT450_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT450_STOP_MAX, AS_INT);
      break;        

     //TT-451
    case MODBUS_REG_TT451_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT451_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_TT451_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT451_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_TT451_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT451_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_TT451_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT451_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_TT451_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT451_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_TT451_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT451_STOP_MAX, AS_INT);
      break;          

    //TT-462
    case MODBUS_REG_TT462_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT462_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_TT462_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT462_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_TT462_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT462_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_TT462_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT462_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_TT462_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT462_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_TT462_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT462_STOP_MAX, AS_INT);
      break;       

    //TT-474
    case MODBUS_REG_TT474_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT474_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_TT474_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT474_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_TT474_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT474_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_TT474_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT474_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_TT474_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT474_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_TT474_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT474_STOP_MAX, AS_INT);
      break;              

    //TT-465
    case MODBUS_REG_TT465_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT465_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_TT465_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT465_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_TT465_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT465_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_TT465_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT465_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_TT465_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT465_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_TT465_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT465_STOP_MAX, AS_INT);
      break; 

    //TT-471
    case MODBUS_REG_TT471_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT471_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_TT471_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT471_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_TT471_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT471_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_TT471_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT471_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_TT471_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT471_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_TT471_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT471_STOP_MAX, AS_INT);
      break;   

    //TT-532
    case MODBUS_REG_TT532_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT532_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_TT532_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT532_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_TT532_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT532_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_TT532_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT532_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_TT532_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT532_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_TT532_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT532_STOP_MAX, AS_INT);
      break;     

    //TT-557
    case MODBUS_REG_TT557_STOP_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT557_STOP_MIN, AS_INT);
      break;          
    case MODBUS_REG_TT557_IDLE_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT557_IDLE_MIN, AS_INT);
      break;    
    case MODBUS_REG_TT557_RUN_MIN:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT557_RUN_MIN, AS_INT);
      break;  
    case MODBUS_REG_TT557_RUN_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT557_RUN_MAX, AS_INT);
      break; 
    case MODBUS_REG_TT557_IDLE_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT557_IDLE_MAX, AS_INT);
      break;     
    case MODBUS_REG_TT557_STOP_MAX:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_TT557_STOP_MAX, AS_INT);
      break;       

    case MODBUS_REG_S1A_FLOW_COUNT:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_S1A_FLOW_COUNT);
      break;
    case MODBUS_REG_S1B_FLOW_COUNT:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_S1B_FLOW_COUNT);
      break; 
    case MODBUS_REG_S2A_FLOW_COUNT:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_S2A_FLOW_COUNT);
      break;
    case MODBUS_REG_S2B_FLOW_COUNT:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_S2B_FLOW_COUNT);
      break; 
    case MODBUS_REG_S3A_FLOW_COUNT:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_S3A_FLOW_COUNT);
      break;
    case MODBUS_REG_S3B_FLOW_COUNT:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_S3B_FLOW_COUNT);
      break;   
    case MODBUS_REG_S1_ACTUAL_CPM:
      reg_response_val = get_config_parameter(CONFIG_PARAM_S1_ACTUAL_CPM, LSB);
      break; 
    case MODBUS_REG_S1_ACTUAL_CPM+1:
      reg_response_val = get_config_parameter(CONFIG_PARAM_S1_ACTUAL_CPM, MSB);
      break; 
    case MODBUS_REG_S1_MIN_CPM:
      reg_response_val = get_config_parameter(CONFIG_PARAM_S1_MIN_CPM, LSB);
      break; 
    case MODBUS_REG_S1_MIN_CPM+1:
      reg_response_val = get_config_parameter(CONFIG_PARAM_S1_MIN_CPM, MSB);
      break;
    case MODBUS_REG_S1_MAX_CPM:
      reg_response_val = get_config_parameter(CONFIG_PARAM_S1_MAX_CPM, LSB);
      break; 
    case MODBUS_REG_S1_MAX_CPM+1:
      reg_response_val = get_config_parameter(CONFIG_PARAM_S1_MAX_CPM, MSB);
      break;
    case MODBUS_REG_S2_ACTUAL_CPM:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_S2_ACTUAL_CPM, LSB);
      break;
    case MODBUS_REG_S2_ACTUAL_CPM+1: 
      reg_response_val = get_config_parameter(CONFIG_PARAM_S2_ACTUAL_CPM, MSB);
      break;
    case MODBUS_REG_S2_MIN_CPM:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_S2_MIN_CPM, LSB);
      break;
    case MODBUS_REG_S2_MIN_CPM+1: 
      reg_response_val = get_config_parameter(CONFIG_PARAM_S2_MIN_CPM, MSB);
      break;
    case MODBUS_REG_S2_MAX_CPM:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_S2_MAX_CPM, LSB);
      break;
    case MODBUS_REG_S2_MAX_CPM+1: 
      reg_response_val = get_config_parameter(CONFIG_PARAM_S2_MAX_CPM, MSB);
      break;
    case MODBUS_REG_S3_ACTUAL_CPM:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_S3_ACTUAL_CPM, LSB);
      break; 
    case MODBUS_REG_S3_ACTUAL_CPM+1:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_S3_ACTUAL_CPM, MSB);
      break; 
    case MODBUS_REG_S3_MIN_CPM:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_S3_MIN_CPM, LSB);
      break; 
    case MODBUS_REG_S3_MIN_CPM+1:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_S3_MIN_CPM, MSB);
      break;
    case MODBUS_REG_S3_MAX_CPM:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_S3_MAX_CPM, LSB);
      break; 
    case MODBUS_REG_S3_MAX_CPM+1:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_S3_MAX_CPM, MSB);
      break;
    case MODBUS_REG_PT442_RATIO_LOW_PSI:
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT442_RATIO_LOW_PSI, LSB);
      break;
    case MODBUS_REG_PT442_RATIO_LOW_PSI+1:
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT442_RATIO_LOW_PSI, MSB);
      break;
    case MODBUS_REG_PT442_RATIO_HIGH_PSI:
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT442_RATIO_HIGH_PSI, LSB);
      break;
    case MODBUS_REG_PT442_RATIO_HIGH_PSI+1:
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT442_RATIO_HIGH_PSI, MSB);
      break;
    case MODBUS_REG_PT457_TARGET_PSI:
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT457_TARGET_PSI, LSB);
      break;
    case MODBUS_REG_PT457_TARGET_PSI+1:
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT457_TARGET_PSI, MSB);
      break;
    case MODBUS_REG_PT487_TARGET_PSI:
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT487_TARGET_PSI, LSB);
      break;
    case MODBUS_REG_PT487_TARGET_PSI+1:
      reg_response_val = get_config_parameter(CONFIG_PARAM_PT487_TARGET_PSI, MSB);
      break;
    case MODBUS_REG_ERROR_PARAM:  
      reg_response_val = get_config_parameter(ERROR_PARAM_ALL,MSB);
      break;      
    case MODBUS_REG_ERROR_PARAM+1: 
      reg_response_val = get_config_parameter(ERROR_PARAM_ALL,LSB);
      break;      
    case MODBUS_REG_ERROR2_PARAM:  
      reg_response_val = get_config_parameter(ERROR_PARAM2_ALL,MSB);
      break;      
    case MODBUS_REG_ERROR2_PARAM+1: 
      reg_response_val = get_config_parameter(ERROR_PARAM2_ALL,LSB);
      break;        
    case MODBUS_REG_OP_STATE:  
      reg_response_val = get_config_parameter(CONFIG_PARAM_OP_STATE, MSB);
      break;                            
    default:
      return(0xABCD); 
  }
  return(reg_response_val);
}

/*
    Name:         writeRegResponse
    Arguments:    int       The MODBUS register that is no be read from
                  uint16_t  The corresponding value of the register that is to be updated.
    Description:  Update the data requested in register i to the valuye specified in the uint16_t.
*/
int writeRegResponse(int register_n, int16_t value){
  #if defined(DEBUG_MODE)
    Serial.print(F("Write register: "));
    Serial.print(String(register_n));
    Serial.print(F(", value: "));
    Serial.println(String(value));
  #endif
  
  switch(register_n){   
    case MODBUS_REG_RELAYS:
      set_config_parameter(CONFIG_PARAM_RELAYS, value, MSB);
      break; 
    case MODBUS_REG_RELAYS+1:
      set_config_parameter(CONFIG_PARAM_RELAYS, value, LSB);
      break;   
    case MODBUS_REG_OP_STATE:
      set_config_parameter(CONFIG_PARAM_OP_STATE, value, MSB);
      break;    
    case MODBUS_REG_SLAVE_ID:
      set_config_parameter(CONFIG_PARAM_SLAVE_ID, float(value));
      break;  
    case MODBUS_REG_SERIAL_NUM:
      set_config_parameter(CONFIG_PARAM_SERIAL_NUM, value, MSB);
      break;
    case MODBUS_REG_SERIAL_NUM+1:  
      set_config_parameter(CONFIG_PARAM_SERIAL_NUM, value, LSB);
      break; 
    case MODBUS_REG_IDLE_STATE_S1_ALL:  //Clear button pressed then reset Idle state S1 to 0
    case MODBUS_REG_IDLE_STATE_S1_ALL+1:  //Clear button pressed then reset Idle state S1 to 0
      set_config_parameter(IDLE_STATE_S1_ALL, 0);
      break;  
    case MODBUS_REG_IDLE_STATE_S2_ALL:  //Clear button pressed then reset Idle state S2 to 0
    case MODBUS_REG_IDLE_STATE_S2_ALL+1:  //Clear button pressed then reset Idle state S2 to 0
      set_config_parameter(IDLE_STATE_S2_ALL, 0);
      break; 
    case MODBUS_REG_IDLE_STATE_S3_ALL:  //Clear button pressed then reset Idle state S3 to 0
    case MODBUS_REG_IDLE_STATE_S3_ALL+1:  //Clear button pressed then reset Idle state S3 to 0
      set_config_parameter(IDLE_STATE_S3_ALL, 0);
      break;     

    case MODBUS_REG_PV1_CL:
      set_config_parameter(CONFIG_PARAM_PV1_CL, float(value));
      break;
    case MODBUS_REG_PV2_CL:
      set_config_parameter(CONFIG_PARAM_PV2_CL, float(value));
      break;
    case MODBUS_REG_PV3_CL:
      set_config_parameter(CONFIG_PARAM_PV3_CL, float(value));
      break;
    case MODBUS_REG_PV4_CL:
      set_config_parameter(CONFIG_PARAM_PV4_CL, float(value));
      break;  
    //PT-405 IDLE,STOP,RUN  
    case MODBUS_REG_PT405_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_PT405_STOP_MIN, float(value));
      break;
    case MODBUS_REG_PT405_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_PT405_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_PT405_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_PT405_RUN_MIN, float(value));
      break;  
    case MODBUS_REG_PT405_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_PT405_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_PT405_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_PT405_IDLE_MAX, float(value));
      break;  
    case MODBUS_REG_PT405_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_PT405_STOP_MAX, float(value));
      break; 
    //PT-426 IDLE,STOP,RUN  
    case MODBUS_REG_PT426_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_PT426_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_PT426_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_PT426_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_PT426_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_PT426_RUN_MIN, float(value));
      break;
    case MODBUS_REG_PT426_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_PT426_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_PT426_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_PT426_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_PT426_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_PT426_STOP_MAX, float(value));
      break;   
    //PT-442 IDLE,STOP,RUN  
    case MODBUS_REG_PT442_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_PT442_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_PT442_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_PT442_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_PT442_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_PT442_RUN_MIN, float(value));
      break;
    case MODBUS_REG_PT442_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_PT442_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_PT442_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_PT442_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_PT442_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_PT442_STOP_MAX, float(value));
      break;     
    //PT-410 IDLE,STOP,RUN  
    case MODBUS_REG_PT410_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_PT410_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_PT410_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_PT410_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_PT410_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_PT410_RUN_MIN, float(value));
      break;
    case MODBUS_REG_PT410_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_PT410_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_PT410_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_PT410_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_PT410_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_PT410_STOP_MAX, float(value));
      break;         
    //PT-457 IDLE,STOP,RUN  
    case MODBUS_REG_PT457_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_PT457_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_PT457_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_PT457_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_PT457_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_PT457_RUN_MIN, float(value));
      break;
    case MODBUS_REG_PT457_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_PT457_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_PT457_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_PT457_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_PT457_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_PT457_STOP_MAX, float(value));
      break;    
    //PT-416 IDLE,STOP,RUN  
    case MODBUS_REG_PT416_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_PT416_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_PT416_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_PT416_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_PT416_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_PT416_RUN_MIN, float(value));
      break;
    case MODBUS_REG_PT416_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_PT416_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_PT416_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_PT416_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_PT416_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_PT416_STOP_MAX, float(value));
      break;  
    //PT-468 IDLE,STOP,RUN  
    case MODBUS_REG_PT468_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_PT468_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_PT468_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_PT468_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_PT468_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_PT468_RUN_MIN, float(value));
      break;
    case MODBUS_REG_PT468_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_PT468_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_PT468_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_PT468_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_PT468_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_PT468_STOP_MAX, float(value));
      break;        
    //PT-487 IDLE,STOP,RUN  
    case MODBUS_REG_PT487_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_PT487_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_PT487_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_PT487_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_PT487_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_PT487_RUN_MIN, float(value));
      break;
    case MODBUS_REG_PT487_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_PT487_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_PT487_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_PT487_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_PT487_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_PT487_STOP_MAX, float(value));
      break;        
    //PT-496 IDLE,STOP,RUN  
    case MODBUS_REG_PT496_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_PT496_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_PT496_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_PT496_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_PT496_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_PT496_RUN_MIN, float(value));
      break;
    case MODBUS_REG_PT496_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_PT496_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_PT496_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_PT496_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_PT496_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_PT496_STOP_MAX, float(value));
      break;         
    //PT-531 IDLE,STOP,RUN  
    case MODBUS_REG_PT531_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_PT531_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_PT531_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_PT531_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_PT531_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_PT531_RUN_MIN, float(value));
      break;
    case MODBUS_REG_PT531_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_PT531_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_PT531_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_PT531_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_PT531_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_PT531_STOP_MAX, float(value));
      break;     
    //PT-576 IDLE,STOP,RUN  
    case MODBUS_REG_PT576_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_PT576_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_PT576_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_PT576_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_PT576_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_PT576_RUN_MIN, float(value));
      break;
    case MODBUS_REG_PT576_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_PT576_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_PT576_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_PT576_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_PT576_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_PT576_STOP_MAX, float(value));
      break;     
          

    //TT-432 IDLE,STOP,RUN  
    case MODBUS_REG_TT432_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_TT432_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_TT432_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_TT432_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_TT432_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_TT432_RUN_MIN, float(value));
      break;
    case MODBUS_REG_TT432_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_TT432_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_TT432_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_TT432_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_TT432_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_TT432_STOP_MAX, float(value));
      break;             

     //TT-439 IDLE,STOP,RUN  
    case MODBUS_REG_TT439_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_TT439_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_TT439_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_TT439_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_TT439_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_TT439_RUN_MIN, float(value));
      break;
    case MODBUS_REG_TT439_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_TT439_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_TT439_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_TT439_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_TT439_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_TT439_STOP_MAX, float(value));
      break;               

    //TT-435 IDLE,STOP,RUN  
    case MODBUS_REG_TT435_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_TT435_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_TT435_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_TT435_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_TT435_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_TT435_RUN_MIN, float(value));
      break;
    case MODBUS_REG_TT435_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_TT435_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_TT435_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_TT435_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_TT435_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_TT435_STOP_MAX, float(value));
      break;                     

    //TT-436 IDLE,STOP,RUN  
    case MODBUS_REG_TT436_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_TT436_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_TT436_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_TT436_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_TT436_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_TT436_RUN_MIN, float(value));
      break;
    case MODBUS_REG_TT436_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_TT436_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_TT436_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_TT436_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_TT436_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_TT436_STOP_MAX, float(value));
      break;            

    //TT-447 IDLE,STOP,RUN  
    case MODBUS_REG_TT447_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_TT447_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_TT447_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_TT447_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_TT447_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_TT447_RUN_MIN, float(value));
      break;
    case MODBUS_REG_TT447_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_TT447_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_TT447_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_TT447_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_TT447_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_TT447_STOP_MAX, float(value));
      break;           

    //TT-454 IDLE,STOP,RUN  
    case MODBUS_REG_TT454_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_TT454_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_TT454_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_TT454_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_TT454_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_TT454_RUN_MIN, float(value));
      break;
    case MODBUS_REG_TT454_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_TT454_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_TT454_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_TT454_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_TT454_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_TT454_STOP_MAX, float(value));
      break;              

     //TT-450 IDLE,STOP,RUN  
    case MODBUS_REG_TT450_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_TT450_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_TT450_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_TT450_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_TT450_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_TT450_RUN_MIN, float(value));
      break;
    case MODBUS_REG_TT450_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_TT450_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_TT450_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_TT450_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_TT450_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_TT450_STOP_MAX, float(value));
      break;       

    //TT-451 IDLE,STOP,RUN  
    case MODBUS_REG_TT451_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_TT451_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_TT451_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_TT451_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_TT451_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_TT451_RUN_MIN, float(value));
      break;
    case MODBUS_REG_TT451_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_TT451_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_TT451_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_TT451_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_TT451_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_TT451_STOP_MAX, float(value));
      break;                

    //TT-462 IDLE,STOP,RUN  
    case MODBUS_REG_TT462_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_TT462_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_TT462_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_TT462_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_TT462_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_TT462_RUN_MIN, float(value));
      break;
    case MODBUS_REG_TT462_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_TT462_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_TT462_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_TT462_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_TT462_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_TT462_STOP_MAX, float(value));
      break;  

    //TT-474 IDLE,STOP,RUN  
    case MODBUS_REG_TT474_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_TT474_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_TT474_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_TT474_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_TT474_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_TT474_RUN_MIN, float(value));
      break;
    case MODBUS_REG_TT474_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_TT474_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_TT474_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_TT474_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_TT474_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_TT474_STOP_MAX, float(value));
      break;                     

    //TT-465 IDLE,STOP,RUN  
    case MODBUS_REG_TT465_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_TT465_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_TT465_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_TT465_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_TT465_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_TT465_RUN_MIN, float(value));
      break;
    case MODBUS_REG_TT465_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_TT465_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_TT465_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_TT465_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_TT465_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_TT465_STOP_MAX, float(value));
      break;                       

    //TT-471 IDLE,STOP,RUN  
    case MODBUS_REG_TT471_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_TT471_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_TT471_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_TT471_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_TT471_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_TT471_RUN_MIN, float(value));
      break;
    case MODBUS_REG_TT471_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_TT471_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_TT471_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_TT471_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_TT471_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_TT471_STOP_MAX, float(value));
      break; 

    //TT-532 IDLE,STOP,RUN  
    case MODBUS_REG_TT532_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_TT532_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_TT532_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_TT532_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_TT532_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_TT532_RUN_MIN, float(value));
      break;
    case MODBUS_REG_TT532_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_TT532_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_TT532_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_TT532_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_TT532_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_TT532_STOP_MAX, float(value));
      break; 

    //TT-557 IDLE,STOP,RUN  
    case MODBUS_REG_TT557_STOP_MIN:
      set_config_parameter(CONFIG_PARAM_TT557_STOP_MIN, float(value));
      break;    
    case MODBUS_REG_TT557_IDLE_MIN:
      set_config_parameter(CONFIG_PARAM_TT557_IDLE_MIN, float(value));
      break;
    case MODBUS_REG_TT557_RUN_MIN:
      set_config_parameter(CONFIG_PARAM_TT557_RUN_MIN, float(value));
      break;
    case MODBUS_REG_TT557_RUN_MAX:
      set_config_parameter(CONFIG_PARAM_TT557_RUN_MAX, float(value));
      break;  
    case MODBUS_REG_TT557_IDLE_MAX:
      set_config_parameter(CONFIG_PARAM_TT557_IDLE_MAX, float(value));
      break;   
    case MODBUS_REG_TT557_STOP_MAX:
      set_config_parameter(CONFIG_PARAM_TT557_STOP_MAX, float(value));
      break;                             

    case MODBUS_REG_S1A_FLOW_COUNT:  
      set_config_parameter(CONFIG_PARAM_S1A_FLOW_COUNT, float(value));
      break;
    case MODBUS_REG_S1B_FLOW_COUNT:  
      set_config_parameter(CONFIG_PARAM_S1B_FLOW_COUNT, float(value));
      break;
    case MODBUS_REG_S2A_FLOW_COUNT:  
      set_config_parameter(CONFIG_PARAM_S2A_FLOW_COUNT, float(value));
      break;
    case MODBUS_REG_S2B_FLOW_COUNT:  
      set_config_parameter(CONFIG_PARAM_S2B_FLOW_COUNT, float(value));
      break;
    case MODBUS_REG_S3A_FLOW_COUNT:  
      set_config_parameter(CONFIG_PARAM_S3A_FLOW_COUNT, float(value));
      break;
    case MODBUS_REG_S3B_FLOW_COUNT:  
      set_config_parameter(CONFIG_PARAM_S3B_FLOW_COUNT, float(value));
      break;  
    case MODBUS_REG_S1_MIN_CPM:
      set_config_parameter(CONFIG_PARAM_S1_MIN_CPM, value, LSB);
      break;
    case MODBUS_REG_S1_MIN_CPM+1:
      set_config_parameter(CONFIG_PARAM_S1_MIN_CPM, value, MSB);
      break;
    case MODBUS_REG_S1_MAX_CPM:
      set_config_parameter(CONFIG_PARAM_S1_MAX_CPM, value, LSB);
      break;
    case MODBUS_REG_S1_MAX_CPM+1:
      set_config_parameter(CONFIG_PARAM_S1_MAX_CPM, value, MSB);
      break; 
    case MODBUS_REG_S2_MIN_CPM:
      set_config_parameter(CONFIG_PARAM_S2_MIN_CPM, value, LSB);
      break;
    case MODBUS_REG_S2_MIN_CPM+1:
      set_config_parameter(CONFIG_PARAM_S2_MIN_CPM, value, MSB);
      break; 
    case MODBUS_REG_S2_MAX_CPM:
      set_config_parameter(CONFIG_PARAM_S2_MAX_CPM, value, LSB);
      break;
    case MODBUS_REG_S2_MAX_CPM+1:
      set_config_parameter(CONFIG_PARAM_S2_MAX_CPM, value, MSB);
      break; 
    case MODBUS_REG_S3_MIN_CPM:
      set_config_parameter(CONFIG_PARAM_S3_MIN_CPM, value, LSB);
      break;
    case MODBUS_REG_S3_MIN_CPM+1:
      set_config_parameter(CONFIG_PARAM_S3_MIN_CPM, value, MSB);
      break;
    case MODBUS_REG_S3_MAX_CPM:
      set_config_parameter(CONFIG_PARAM_S3_MAX_CPM, value, LSB);
      break;
    case MODBUS_REG_S3_MAX_CPM+1:
      set_config_parameter(CONFIG_PARAM_S3_MAX_CPM, value, MSB);
      break; 
    case MODBUS_REG_ERROR_PARAM:  //Clear button pressed then reset errors to 0
    case MODBUS_REG_ERROR_PARAM+1:  //Clear button pressed then reset errors to 0
    case MODBUS_REG_ERROR2_PARAM:  //Clear button pressed then reset errors2 to 0
    case MODBUS_REG_ERROR2_PARAM+1:  //Clear button pressed then reset errors2 to 0
      set_config_parameter(ERROR_PARAM_ALL, 0);    
      set_config_parameter(ERROR_PARAM2_ALL, 0);
      break;  

    case MODBUS_REG_PULSE1:  
    case MODBUS_REG_PULSE2:  
    case MODBUS_REG_PULSE3:  
      set_config_parameter(CONFIG_PARAM_PULSE1, 0);    
      set_config_parameter(CONFIG_PARAM_PULSE2, 0);
      set_config_parameter(CONFIG_PARAM_PULSE3, 0);
      break;         
    default:
      break;
  }
}


/*
    Name:         modbuxRxData
    Arguments:    uint8_t   The new data bytes that has been received. 
                  int       The buffer to which the new data bytes is to be stored for later processing
    Description:  This function receives new data from any interface and stores it into its own unique buffer for processing later on. 
*/
void modbuxRxData(uint8_t new_byte, int buffer)
{
  int * bufferPtr;

  if (buffer == TCP_BUFFER){
    TCPSerialBufferRx[TCPSerialBufferPt] = new_byte;
    bufferPtr = &TCPSerialBufferPt;
  }else if (buffer == RS_485_BUFFER){
    RSSerialBufferRx[RSSerialBufferPt] = new_byte;
    bufferPtr = &RSSerialBufferPt;
  }else if (buffer == BLE_BUFFER){
    BLESerialBufferRx[BLESerialBufferPt] = new_byte;
    bufferPtr = &BLESerialBufferPt;
  }else{
    #if defined(DEBUG_MODE)
      Serial.print(F("Unknown buffer in modbuxRxData modbus_c200.cpp"));
    #endif
    return;
  }
  // Increment whichever pointer goes with this buffer
  if (*bufferPtr < (TCP_SERIAL_BUFFER_LIMIT - 1)){
    *bufferPtr = *bufferPtr + 1;    // The normal *buggerPtr++ will not work here. For some reason this needs to be explicit!
  }else{
    *bufferPtr = 0;
  }
  return;
}

/*
    Name:         get_tx_bytes
    Arguments:    int       The buffer to which the tx data length is referring to.
    Returns:      int       The length of the data to be transmitted on the specified interface in bytes. 
    Description:  After a specific interface buffer has been processed, if there is any resposse to be made then said response will be placed into the 
                  corresponding tx data buffer for that interface and a transmission data length set. The caller can quickly determine if a response needs
                  to be sent by calling this function and looking for any length > 0. 
*/
int get_tx_bytes(int buffer)
{
  int temp_tx_bytes = 0;
  if (buffer == TCP_BUFFER){
    temp_tx_bytes = tcp_tx_bytes;
    tcp_tx_bytes = 0; 
  }else if (buffer == RS_485_BUFFER){
    temp_tx_bytes = rs_tx_bytes;
    rs_tx_bytes = 0; 
  }else if (buffer == BLE_BUFFER){
    temp_tx_bytes = ble_tx_bytes;
    ble_tx_bytes = 0; 
  }else{
    #if defined(DEBUG_MODE)
      Serial.print(F("Unknown buffer in modbus_loop modbus_c200.cpp"));
    #endif
    return(0);
  }

  #if defined(DEBUG_MODE)
    Serial.print(F("Responding with n bytes: "));
    Serial.println(String(temp_tx_bytes));
  #endif
  return(temp_tx_bytes);
}

/*
    Name:         get_tx_buffer
    Arguments:    int       The buffer to which the tx data length is referring to.
    Returns:      uint8_t   A pointer to the data buffer for said interface specified in the call. 
    Description:  Once data has been processed if any response is requried then the data length returned in get_tx_bytes 
                  will be > 0. If this happens then th specific bytes in the tx-Buffer for transmission can be gathered 
                  by calling this function on that same interface. 
*/
uint8_t * get_tx_buffer(int buffer)
{
  if (buffer == TCP_BUFFER){
    return(TCPSerialBufferTx);
  }else if (buffer == RS_485_BUFFER){
    return(RSSerialBufferTx);
  }else if (buffer == BLE_BUFFER){
    return(BLESerialBufferTx);
  }else{
    #if defined(DEBUG_MODE)
      Serial.print(F("Unknown buffer in modbus_loop modbus_c200.cpp"));
    #endif
    return(0);
  }
}

/*
    Name:         modbus_loop
    Arguments:    int      The buffer to which the tx data length is referring to.
    Returns:      int      The number of data bytes that need to be sent in response to a valid packet that was found in
                            the specified interfaces buffer. 
    Description:  This function is callde continually for each interface until a valid packet is detected in the receive 
                  buffer for said interface. This function will return 0 while no valid packet could be found or any number 
                  > 0 when a response from a valid packet is required. 
*/
int modbus_loop(int buffer) {
  int dataLength = 8;   // All register reads are 8 bytes in length. ID, Fn, 2x REGISTER, 2x length req, 2x CRC = 8
  int * bufferPtr;
  uint8_t * processingBufferRx;
  uint8_t * processingBufferTx;
  int * tx_bytes;

  if (buffer == TCP_BUFFER){
    processingBufferRx = &TCPSerialBufferRx[0];
    processingBufferTx = &TCPSerialBufferTx[0];
    bufferPtr = &TCPSerialBufferPt;
    tx_bytes = &tcp_tx_bytes;
  }else if (buffer == RS_485_BUFFER){
    processingBufferRx = &RSSerialBufferRx[0];
    processingBufferTx = &RSSerialBufferTx[0];
    bufferPtr = &RSSerialBufferPt;
    tx_bytes = &rs_tx_bytes;
  }else if (buffer == BLE_BUFFER){
    processingBufferRx = &BLESerialBufferRx[0];
    processingBufferTx = &BLESerialBufferTx[0];
    bufferPtr = &BLESerialBufferPt;
    tx_bytes = &ble_tx_bytes;
  }else{
    #if defined(DEBUG_MODE)
      Serial.print(F("Unknown buffer in modbus_loop modbus_c600.cpp"));
    #endif
    return(0);
  }
  
  for(int i = 0; i < (TCP_SERIAL_BUFFER_LIMIT - dataLength); i++){  // Look through the entire buffer
    // For faster processing, don't bother doing anything unless the first byte under investigation is at least
    // the MOSBUS slave ID
    if (buffer != BLE_BUFFER){    // EXCEPTION! On BLE the connection can only be 1:1 and therefore just work whatever the slave ID is!
      if (processingBufferRx[i] != get_config_parameter(CONFIG_PARAM_SLAVE_ID)){
        continue; 
      }
    }

    int requestID = processingBufferRx[i]; // The ID in the packet from the master
    int functionCode = processingBufferRx[i+1]; // The function code. 
    int nRegisters = (processingBufferRx[i+4] << 8) | processingBufferRx[i+5];   // How many registers to read?
    int functionRegister = (processingBufferRx[i+2] << 8) | processingBufferRx[i+3];   // MB Register

    // The data length will depend on the function code we are operating with at this time
    if ((functionCode == 3) || (functionCode == 6)){
      dataLength = 6;   // All register reads are 8 bytes in length. ID, Fn, 2x REGISTER, 2x length req, 2x CRC = 8
    }else if (functionCode == 16){
      dataLength = 7 + (2*nRegisters);  // Total length 9 + 2*nR, however we do not include the CRC in the data length
    }
    
    if (i <= (TCP_SERIAL_BUFFER_LIMIT - dataLength)){         // If we have the entire packet
      if(CRC16(&processingBufferRx[i], dataLength, PACKET_CHECK_ONLY)){        // CRC Checking    
        // The tx buffer can only handle 255 bytes. So limit the number of registers we are allowed to read to a MAX 100
        if (nRegisters > 100){
            // Now that this packet has been processed - and is not valid for this device! We can destroy it so that it will not be processed again
            processingBufferRx[i] = 0;    // Wipe the RTU ID 
            processingBufferRx[i+1] = 0;  // Wipte the function code
            TCPSerialBufferPt = 0;   // We can return the pointer to the beginning of the buffer since a valid packet has been detected
            #if defined(DEBUG_MODE)
              Serial.println(F("Unable to return more than 100 registers at once"));
            #endif
        }else{ 
          if (functionCode == 3){
            int index = 0;    // Index the number of registers returned
            
            processingBufferTx[0] = requestID;    // Slave ID
            processingBufferTx[1] = 3;    // Function code
            processingBufferTx[2] = 2 * nRegisters;    // Number of bytes being returned
            
            for (int j = nRegisters; j > 0; j--){
              uint16_t modbus_response_val = readRegResponse(functionRegister++);
              processingBufferTx[3+index] = (uint8_t)((modbus_response_val >> 8) & 0xFF);    // MSB data being returned
              processingBufferTx[4+index] = (uint8_t)(modbus_response_val & 0xFF);    // LSB data being returned, byte only 
              index = index + 2;  // Increment to the next point in the buffer where the register response will be stored for sending
            }
            // The data length is 3 (ID, FN, Length) + 2 bytes for every register!
            CRC16(processingBufferTx, (3 + (2*nRegisters)), PACKET_CRC_INSERT); // Insert the CRC
            
            // Finally, send the data
            //SerialBT.write(TCPSerialBufferTx, 7);  // Respond to the MODBUS request by sending data out of the BT Serial port
            *tx_bytes = 5 + (2 * nRegisters);
          }else if(functionCode == 16){
            int16_t value_n = (processingBufferRx[i+7] << 8) | processingBufferRx[i+8];   // MB Register
            Serial.print(F("Function code 16 CRC Pass for nRegisters = "));
            Serial.println(nRegisters);            
            
            processingBufferTx[0] = requestID;    // Slave ID
            processingBufferTx[1] = 16;    // Function code
            processingBufferTx[2] = processingBufferRx[i+2];    // MB register MSB
            processingBufferTx[3] = processingBufferRx[i+3];   // MB Register LSB
            processingBufferTx[4] = processingBufferRx[i+4];    // n registers MSB
            processingBufferTx[5] = processingBufferRx[i+5];    // n registers LSB
            CRC16(processingBufferTx, 6, PACKET_CRC_INSERT); // Insert the CRC

            while(nRegisters > 0){
              writeRegResponse(functionRegister, value_n);    // Write the value
              nRegisters--;   // Remove one from the number of registers we need to process
              functionRegister++;   // Move to the next function register to be written
              i = i + 2;    // Move our buffer indexing to the next word (below) that will be written
              value_n = (processingBufferRx[i+7] << 8) | processingBufferRx[i+8];   // Next value to be written
            }

            // Finally, send the data
            //SerialBT.write(TCPSerialBufferTx, 7);  // Respond to the MODBUS request by sending data out of the BT Serial port
            *tx_bytes = 8;
          }else{   // End if read holding registers
            // Unsupported function code
            Serial.print(F("Unsupported function code: "));
            Serial.println(String(functionCode));
          }// End if test function code
          // Wipe the data bytes just processed in the buffer to avoid double processing
          memset(processingBufferRx, '\0', TCP_SERIAL_BUFFER_LIMIT);    // We know that dataLength is safe because it was tested above
          *bufferPtr = 0; // Reset the buffer data pointer to the beginning of the buffer
        } // End if slave ID matches our own
      } // end CRC checking
    }
  } // End buffer itteration
  return(*tx_bytes);
}
