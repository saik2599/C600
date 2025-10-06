/*
    Author: Christopher Glanville
    Description:    IO class to handle all external connections (Both analog and digital) to and from the Giga. Debounce functions 
                    for button inputs are also handled here. 
   Date: 26/5/24
   updated : 26/5/24
*/
#include <PI4IOE5V6534Q.h>
#include "ADS7828.h"
#include <Wire.h>
#include <stdint.h>
#include <movingAvg.h>
#include <Ewma.h>
#include "Adafruit_MCP9600.h"
#include "Adafruit_MCP9601.h"
#include <Adafruit_MCP4728.h>
#include "MCP4728.h"

#include "watchdog_c200.h"
#include "config_c200.h"
#include "io_c200.h"
#include "ble_c200.h"  // Required to update the name of the C200 PCB in the event of a change to SW1.3

// ************ Pin definitions *****************
//Exp1
#define COOLANT_PT1 0            // TB30, ADS-7828 Addr 0x48, channel 0
#define COOLANT_PT1_EXP PT_Exp1  // Connect each parameter to the correct I2C Expander
#define COOLANT_PT2 1            // TB31, ADS-7828 Addr 0x48, channel 0
#define COOLANT_PT2_EXP PT_Exp1  // Connect each parameter to the correct I2C Expander
#define S3_TANK 4            // TB25, ADS-7828 Addr 0x48, channel 0
#define S3_TANK_EXP PT_Exp1  // Connect each parameter to the correct I2C Expander
//Exp2
#define SUCTION_PT 2            // TB19, ADS-7828 Addr 0x48, channel 0
#define SUCTION_PT_EXP PT_Exp2  // Connect each parameter to the correct I2C Expander
#define SCT_TANK_PT 3            // TB20, ADS-7828 Addr 0x48, channel 0
#define SCT_TANK_PT_EXP PT_Exp2  // Connect each parameter to the correct I2C Expander
#define HFL_PUMP 4            // TB21, ADS-7828 Addr 0x48, channel 0
#define HFL_PUMP_EXP PT_Exp2  // Connect each parameter to the correct I2C Expander
#define S1_TANK 5            // TB22, ADS-7828 Addr 0x48, channel 0
#define S1_TANK_EXP PT_Exp2  // Connect each parameter to the correct I2C Expander
#define S3_PT 6            // TB23, ADS-7828 Addr 0x48, channel 0
#define S3_PT_EXP PT_Exp2  // Connect each parameter to the correct I2C Expander
#define S2_TANK 7            // TB24, ADS-7828 Addr 0x48, channel 0
#define S2_TANK_EXP PT_Exp2  // Connect each parameter to the correct I2C Expander

#define DEBOUNCE_TIMER 1000

#define EXP1_IO_ADDR 0x48
#define EXP2_IO_ADDR 0x49
#define EXP4_IO_ADDR 0x22
#define EXP5_IO_ADDR 0x23
#define EXP7_IO_ADDR 0x60

#define TT_SIZE 14
TwoWire i2c(20, 21);

PI4IOE5V6534Q exp4_io(EXP4_IO_ADDR, i2c);  // For some reason address is multipled by 2 on the schematic
PI4IOE5V6534Q exp5_io(EXP5_IO_ADDR, i2c);

ADS7828 PT_Exp1, PT_Exp2;
MCP4728 dac_Exp7;

Adafruit_MCP9601 mcp1;
Adafruit_MCP9601 mcp2;
Adafruit_MCP9601 mcp3;
Adafruit_MCP9601 mcp4;
Adafruit_MCP9601 mcp5;
Adafruit_MCP9601 mcp6;
Adafruit_MCP9601 mcp7;
Adafruit_MCP9601 mcp8;
Adafruit_MCP9601 mcp9;
Adafruit_MCP9601 mcp10;
Adafruit_MCP9601 mcp11;
Adafruit_MCP9601 mcp12;
Adafruit_MCP9601 mcp13;
Adafruit_MCP9601 mcp14;
uint8_t mcpExist = 0;
int errCnt = 0;
unsigned long lastpulse1readTime = 0, lastpulse2readTime = 0, lastpulse3readTime = 0;
float coolant1_pt, coolant2_pt;


bool PressgreenButton = false;
bool pauseButton = false;
bool OldValue_Redbutton = false, OldValue_yellowbutton = false, OldValue_greenbutton = false, OldValue_HFLSwitch = false, OldValue_estop = false;
int tracker = 0, trackerRed = 0, trackerGreen = 0, trackerAmber = 0, trackerHFLSwitch = 0, estop_tracker = 0;
bool temp_Redbutton, temp_yellowbutton, temp_greenbutton, temp_Flow, temp_HFLSwitch, temp_estop;

Ewma avgTT1(1);     // Previously no averaging
Ewma avgTT2(1);     // Previous from PCB v7.5
Ewma avgTT3(1);
Ewma avgTT4(1);
Ewma avgTT5(1);
Ewma avgTT6(1);
Ewma avgTT7(1);
Ewma avgTT8(1);     // Previously no averaging
Ewma avgTT9(1);     // Previous from PCB v7.5
Ewma avgTT10(1);
Ewma avgTT11(1);
Ewma avgTT12(1);
Ewma avgTT13(1);
Ewma avgTT14(1);

Ewma avgPT1(0.02);     
Ewma avgPT2(0.02);     
Ewma avgPT3(0.02);
Ewma avgPT4(0.02);
Ewma avgPT5(0.02);
Ewma avgPT6(0.02);
Ewma avgPT7(0.02);
Ewma avgPT8(0.02);     
Ewma avgPT9(0.02); 
Ewma avgPT10(0.02);
Ewma avgPT11(0.02);
Ewma avgPT12(0.02);
Ewma avgPT13(0.02);
Ewma avgPT14(0.02);
Ewma avgPT15(0.02);

  double AI_01_TC = 0;
  double AI_02_TC = 0;
  double AI_03_TC = 0;
  double AI_04_TC = 0;
  double AI_05_TC = 0;
  double AI_06_TC = 0;
  double AI_07_TC = 0;
  double AI_08_TC = 0;
  double AI_09_TC = 0;
  double AI_10_TC = 0;
  double AI_11_TC = 0;
  double AI_12_TC = 0;
  double AI_13_TC = 0;
  double AI_14_TC = 0;

  // Ewma avgPT911(0.02);
  // Ewma avgPT407(0.02);
  // Ewma avgPT410(0.02);
  // The following deals with the MCP-9601 TCs
  struct tt {
    String name;
    String key;
    double raw;
    double rawTemp;
    Ewma avg;
    double* value;
    double failCtr;
    int min;
    int minRecovery;
    int minPause;
    int max;
    int maxRecovery;
    int maxPause;
    double coef;
    int mcp;
    int channel;
    int pause;
    unsigned long overheat;
    unsigned long underheat;
  } TTdata[] = {
    { "TC1", "TT01", 0, 0, avgTT1, &AI_01_TC, 0, -1, -1, 0, 140, 130, 0, 1, 1, -1, 0, 0, 0 },
    { "TC2", "TT02", 0, 0, avgTT2, &AI_02_TC, 0, -1, -1, 0, 140, 130, 0, 1, 2, -1, 0, 0, 0 },
    { "TC3", "TT03", 0, 0, avgTT3, &AI_03_TC, 0, -1, -1, 0, 140, 130, 0, 1, 3, -1, 0, 0, 0 },
    { "TC4", "TT04", 0, 0, avgTT4, &AI_04_TC, 0, -1, -1, 0, 140, 130, 0, 1, 4, -1, 0, 0, 0 },
    { "TC5", "TT05", 0, 0, avgTT5, &AI_05_TC, 0, -1, -1, 0, 140, 130, 0, 1, 5, -1, 0, 0, 0 },
    { "TC6", "TT06", 0, 0, avgTT6, &AI_06_TC, 0, -1, -1, 0, 140, 130, 0, 1, 6, -1, 0, 0, 0 },
    { "TC7", "TT07", 0, 0, avgTT7, &AI_07_TC, 0, -1, -1, 0, 140, 130, 0, 1, 7, -1, 0, 0, 0 },
    { "TC8", "TT08", 0, 0, avgTT8, &AI_08_TC, 0, -1, -1, 0, 140, 130, 0, 1, 8, -1, 0, 0, 0 },
    { "TC9", "TT09", 0, 0, avgTT9, &AI_09_TC, 0, -1, -1, 0, 140, 130, 0, 1, 9, -1, 0, 0, 0 },
    { "TC10", "TT10", 0, 0, avgTT9, &AI_10_TC, 0, -1, -1, 0, 140, 130, 0, 1, 10, -1, 0, 0, 0 },
    { "TC11", "TT11", 0, 0, avgTT7, &AI_11_TC, 0, -1, -1, 0, 140, 130, 0, 1, 11, -1, 0, 0, 0 },
    { "TC12", "TT12", 0, 0, avgTT8, &AI_12_TC, 0, -1, -1, 0, 140, 130, 0, 1, 12, -1, 0, 0, 0 },
    { "TC13", "TT13", 0, 0, avgTT9, &AI_13_TC, 0, -1, -1, 0, 140, 130, 0, 1, 13, -1, 0, 0, 0 },
    { "TC14", "TT14", 0, 0, avgTT9, &AI_14_TC, 0, -1, -1, 0, 140, 130, 0, 1, 14, -1, 0, 0, 0 },
  };
  String errMsg[30] = { "" };

float fmap(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
    Name:         debounce_RedButton
    Arguments:    bool    New button input value
    Returns:      bool    The current button input value which is valid for use. 
    Description:  This function accepts continually updated values form the input pin of the red button. Changes to 
                  the value of the input pin are not transferred to the operational boolean value until a period 
                  of stable values has been observed. This prevents false state changes from being reported during 
                  a press or release of the button. 
*/
bool debounce_RedButton(bool InputVal) {
  if (OldValue_Redbutton != InputVal) {
    trackerRed = 0;
    OldValue_Redbutton = InputVal;
    return temp_Redbutton;
  } else {
    trackerRed++;
    if (trackerRed > 3) {
      temp_Redbutton = OldValue_Redbutton;
      return InputVal;
    }
  }
}


/*
    Name:         debounce_AmberButton
    Arguments:    bool    New button input value
    Returns:      bool    The current button input value which is valid for use. 
    Description:  This function accepts continually updated values form the input pin of the amber button. Changes to 
                  the value of the input pin are not transferred to the operational boolean value until a period 
                  of stable values has been observed. This prevents false state changes from being reported during 
                  a press or release of the button. 
*/
bool debounce_AmberButton(bool InputVal) {
  if (OldValue_yellowbutton != InputVal) {
    trackerAmber = 0;
    OldValue_yellowbutton = InputVal;
    return temp_yellowbutton;
  } else {
    trackerAmber++;
    if (trackerAmber > 3) {
      temp_yellowbutton = OldValue_yellowbutton;
      return InputVal;
    }
  }
}

/*
    Name:         debounce_GreenButton
    Arguments:    bool    New button input value
    Returns:      bool    The current button input value which is valid for use. 
    Description:  This function accepts continually updated values form the input pin of the green button. Changes to 
                  the value of the input pin are not transferred to the operational boolean value until a period 
                  of stable values has been observed. This prevents false state changes from being reported during 
                  a press or release of the button. 
*/
bool debounce_GreenButton(bool InputVal) {

  if (OldValue_greenbutton != InputVal) {
    trackerGreen = 0;
    OldValue_greenbutton = InputVal;
    return temp_greenbutton;
  } else {
    trackerGreen++;
    if (trackerGreen > 3) {
      temp_greenbutton = OldValue_greenbutton;
      return InputVal;
    }
  }
}

/*
    Name:         debounce_EstopButton
    Arguments:    bool    New button input value
    Returns:      bool    The current button input value which is valid for use. 
    Description:  This function accepts continually updated values form the input pin of the E-STOP button. Changes to 
                  the value of the input pin are not transferred to the operational boolean value until a period 
                  of stable values has been observed. This prevents false state changes from being reported during 
                  a press or release of the button. 
*/
bool debounce_EstopButton(bool InputVal) {

  if (OldValue_estop != InputVal) {
    estop_tracker = 0;
    OldValue_estop = InputVal;
    return temp_estop;
  } else {
    estop_tracker++;
    if (estop_tracker > 5) {
      temp_estop = OldValue_estop;
      return InputVal;
    }
  }
}


/*
    Name:         io_setup
    Arguments:    void
    Returns:      nil
    Description:  io_setup sets all of the initial conditions for reading and writing to external IO, including configuration of the 
                  IO expanders. This function must be called at power up before any others in this class. 
*/
void io_setup() {
  Serial.println("Beginning IO Setup...");

  pinMode(48, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(48), flowMeter1Interrupt, FALLING); // Or FALLING, CHANGE
  pinMode(50, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(50), flowMeter2Interrupt, FALLING); // Or FALLING, CHANGE
  pinMode(52, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(52), flowMeter3Interrupt, FALLING); // Or FALLING, CHANGE

  i2c.setClock(400000UL);  // Default clock speed is 100kHz. Increase to 400kHz.
  exp4_io.begin();
  exp5_io.begin();

 // exp4_io.portMode(IO_EXP_PORT3, INPUT);
  Serial.println("EXP4 Initialization...");
  exp4_io.pinMode(P0_0, INPUT_PULLUP, true);
  exp4_io.pinMode(P0_1, INPUT_PULLUP, true);
  exp4_io.pinMode(P0_2, INPUT_PULLUP, true);
  exp4_io.pinMode(P0_3, INPUT_PULLUP, true);
  exp4_io.pinMode(P0_4, INPUT_PULLUP, true);
  exp4_io.pinMode(P0_5, INPUT_PULLUP, true);
  exp4_io.pinMode(P0_6, INPUT_PULLUP, true);
  exp4_io.pinMode(P0_7, INPUT_PULLUP, true);
  exp4_io.pinMode(P1_0, INPUT_PULLUP, true);
  exp4_io.pinMode(P1_1, INPUT_PULLUP, true);
  exp4_io.pinMode(P1_2, INPUT_PULLUP, true);
  exp4_io.pinMode(P1_3, INPUT_PULLUP, true);
  exp4_io.pinMode(P1_4, INPUT_PULLUP, true);
  exp4_io.pinMode(P1_5, INPUT_PULLUP, true);
  exp4_io.pinMode(P1_6, INPUT_PULLUP, true);
  exp4_io.pinMode(P1_7, INPUT_PULLUP, true);
  exp4_io.pinMode(P2_0, INPUT_PULLUP, true);
  exp4_io.pinMode(P2_1, INPUT_PULLUP, true);
  exp4_io.pinMode(P2_3, INPUT_PULLUP, true);
  exp4_io.pinMode(P2_5, INPUT_PULLUP, true);
  exp4_io.pinMode(P2_7, INPUT_PULLUP, true);    // Coolant flow switch
  exp4_io.pinMode(P3_1, INPUT_PULLUP, true);
  exp4_io.pinMode(P3_2, INPUT_PULLUP, true);
  exp4_io.pinMode(P3_3, OUTPUT);
  exp5_io.digitalWrite(P3_3, LOW);      // As soon as an output is configured, set it to low. This prevents the initialization click-clack
  exp4_io.pinMode(P3_4, OUTPUT);
  exp5_io.digitalWrite(P3_4, LOW);
  exp4_io.pinMode(P3_6, OUTPUT);
  exp5_io.digitalWrite(P3_6, LOW);

  Serial.println("EXP5 Initialization...");
  exp5_io.pinMode(P0_0, OUTPUT);      // As soon as an output is configured, set it to low. This prevents the initialization click-clack
  exp5_io.digitalWrite(P0_0, LOW);
  exp5_io.pinMode(P0_1, OUTPUT); 
  exp5_io.digitalWrite(P0_1, LOW);
  exp5_io.pinMode(P0_2, OUTPUT);
  exp5_io.digitalWrite(P0_2, LOW); 
  exp5_io.pinMode(P0_3, OUTPUT); 
  exp5_io.digitalWrite(P0_3, LOW);
  exp5_io.pinMode(P0_4, OUTPUT); 
  exp5_io.digitalWrite(P0_4, LOW);
  exp5_io.pinMode(P0_5, OUTPUT);
  exp5_io.digitalWrite(P0_5, LOW);
  exp5_io.pinMode(P0_6, OUTPUT);
  exp5_io.digitalWrite(P0_6, LOW);
  exp5_io.pinMode(P0_7, OUTPUT);
  exp5_io.digitalWrite(P0_7, LOW);
  exp5_io.pinMode(P1_0, OUTPUT);
  exp5_io.digitalWrite(P1_0, LOW);
  exp5_io.pinMode(P1_1, OUTPUT);
  exp5_io.digitalWrite(P1_1, LOW);
  exp5_io.pinMode(P1_2, OUTPUT);
  exp5_io.digitalWrite(P1_2, LOW);
  exp5_io.pinMode(P1_3, OUTPUT);
  exp5_io.digitalWrite(P1_3, LOW);
  exp5_io.pinMode(P1_4, INPUT_PULLUP, true);
  exp5_io.pinMode(P1_5, OUTPUT);
  exp5_io.digitalWrite(P1_5, LOW);
  exp5_io.pinMode(P1_6, INPUT_PULLUP, true);
  exp5_io.pinMode(P1_7, INPUT_PULLUP, true);
  exp5_io.pinMode(P2_1, OUTPUT, true);
  exp5_io.digitalWrite(P2_2, LOW);
  exp5_io.pinMode(P2_2, OUTPUT);
  exp5_io.digitalWrite(P2_2, LOW);
  exp5_io.pinMode(P2_3, OUTPUT);
  exp5_io.digitalWrite(P2_3, LOW);
  exp5_io.pinMode(P2_4, INPUT_PULLUP, true);
  exp5_io.pinMode(P2_5, INPUT_PULLUP, true);


  // SW1 Switch state inputs
  exp5_io.pinMode(P2_6, INPUT_PULLUP, true);
  exp5_io.pinMode(P2_7, INPUT_PULLUP, true);
  exp5_io.pinMode(P3_0, INPUT_PULLUP, true);
  exp5_io.pinMode(P3_1, INPUT_PULLUP, true);
  exp5_io.pinMode(P3_2, INPUT_PULLUP, true);
  exp5_io.pinMode(P3_3, INPUT_PULLUP, true);
  exp5_io.pinMode(P3_4, INPUT_PULLUP, true);
  exp5_io.pinMode(P3_5, INPUT_PULLUP, true);
  exp5_io.pinMode(P3_6, INPUT_PULLUP, true);
  exp5_io.pinMode(P3_7, INPUT_PULLUP, true);
  exp5_io.pinMode(P4_0, INPUT_PULLUP, true);
  exp5_io.pinMode(P4_1, INPUT_PULLUP, true);

  Wire.begin();
  Wire1.begin();

  //Wire.setTimeout(250);    // // Although this function exists, it does fuck all for the I2C hanging issue
  //i2c.timeOut(100);  // milliseconds
  //Wire.setWireTimeout(250, true);    // 250ms timeout, reset I2C on timeout (TRUE) - It would be great if the function existed.

  Serial.println("EXP1 Initialization...");
  PT_Exp1.begin(EXP1_IO_ADDR, &i2c);
  Serial.println("EXP2 Initialization...");
  PT_Exp2.begin(EXP2_IO_ADDR, &i2c);

   dac_Exp7.attatch(Wire, 16);   // Pin 16 is not connected and on Exp7 DAC is connected to 0VDC so let's see if this works
   dac_Exp7.readRegisters();
   dac_Exp7.selectVref(MCP4728::VREF::VDD, MCP4728::VREF::VDD, MCP4728::VREF::VDD, MCP4728::VREF::VDD);
   dac_Exp7.selectPowerDown(MCP4728::PWR_DOWN::GND_100KOHM, MCP4728::PWR_DOWN::GND_100KOHM, MCP4728::PWR_DOWN::GND_100KOHM, MCP4728::PWR_DOWN::GND_100KOHM);
   dac_Exp7.selectGain(MCP4728::GAIN::X1, MCP4728::GAIN::X1, MCP4728::GAIN::X1, MCP4728::GAIN::X1);
   dac_Exp7.analogWrite(MCP4728::DAC_CH::A, 50);   
   dac_Exp7.analogWrite(MCP4728::DAC_CH::B, 50);   
   dac_Exp7.analogWrite(MCP4728::DAC_CH::C, 50);
   dac_Exp7.analogWrite(MCP4728::DAC_CH::D, 50);
   dac_Exp7.enable(true);
   dac_Exp7.readRegisters();
  
  Serial.println("MCP1 Initialization...");
  if (!mcp1.begin(0x61)) {
    Serial.println("WARNING!! Couldnt detect mcp1(0x61)");
    bitSet(mcpExist, 0);
  } else {
    mcp1.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp1.setThermocoupleType(MCP9600_TYPE_K);
    mcp1.enable(true);
  }

  Serial.println("MCP2 Initialization...");
  if (!mcp2.begin(0x62)) {
    Serial.println("WARNING!! Couldnt detect mcp2(0x62)");
    bitSet(mcpExist, 1);
  } else {
    mcp2.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp2.setThermocoupleType(MCP9600_TYPE_K);
    mcp2.enable(true);
  }

  Serial.println("MCP3 Initialization...");
  if (!mcp3.begin(0x63)) {
    Serial.println("WARNING!! Couldnt detect mcp3(0x63)");
    bitSet(mcpExist, 2);
  } else {
    mcp3.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp3.setThermocoupleType(MCP9600_TYPE_K);
    mcp3.enable(true);
  }

  Serial.println("MCP4 Initialization...");
  if (!mcp4.begin(0x64)) {
    Serial.println("WARNING!! Couldnt detect mcp4(0x64)");
    bitSet(mcpExist, 3);
  } else {
    mcp4.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp4.setThermocoupleType(MCP9600_TYPE_K);
    mcp4.enable(true);
  }

  Serial.println("MCP5 Initialization...");
  if (!mcp5.begin(0x65)) {
    Serial.println("WARNING!! Couldnt detect mcp5(0x65)");
    bitSet(mcpExist, 4);
  } else {
    mcp5.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp5.setThermocoupleType(MCP9600_TYPE_K);
    mcp5.enable(true);
  }

  Serial.println("MCP6 Initialization...");
  if (!mcp6.begin(0x66)) {
    Serial.println("WARNING!! Couldnt detect mcp6(0x66)");
    bitSet(mcpExist, 5);
  } else {
    mcp6.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp6.setThermocoupleType(MCP9600_TYPE_K);
    mcp6.enable(true);
  }

  Serial.println("MCP7 Initialization...");
  if (!mcp7.begin(0x67)) {
    Serial.println("WARNING!! Couldnt detect mcp7(0x67)");
    bitSet(mcpExist, 6);
  } else {
    mcp7.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp7.setThermocoupleType(MCP9600_TYPE_K);
    mcp7.enable(true);
  }

  Serial.println("MCP8 Initialization...");
  if (!mcp8.begin(0x61, &Wire1)) {
    Serial.println("WARNING!! Couldnt detect mcp8(0x61)");
    bitSet(mcpExist, 7);
  } else {
    mcp8.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp8.setThermocoupleType(MCP9600_TYPE_K);
    mcp8.enable(true);
  }

  // From here on WE MUST use the alternative I2C bus
  Serial.println("MCP9 Initialization...");
  if (!mcp9.begin(0x62, &Wire1)) {
    Serial.println("WARNING!! Couldnt detect mcp9(0x62)");
    bitSet(mcpExist, 8);
  } else {
    mcp9.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp9.setThermocoupleType(MCP9600_TYPE_K);
    mcp9.enable(true);
  }

  Serial.println("MCP10 Initialization...");
  if (!mcp10.begin(0x63, &Wire1)) {
    Serial.println("WARNING!! Couldnt detect mcp10(0x63)");
    bitSet(mcpExist, 9);
  } else {
    mcp10.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp10.setThermocoupleType(MCP9600_TYPE_K);
    mcp10.enable(true);
  }
  
 Serial.println("MCP11 Initialization...");
  if (!mcp11.begin(0x64, &Wire1)) {
    Serial.println("WARNING!! Couldnt detect mcp11(0x64)");
    bitSet(mcpExist, 10);
  } else {
    mcp11.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp11.setThermocoupleType(MCP9600_TYPE_K);
    mcp11.enable(true);
  }

 Serial.println("MCP12 Initialization...");
  if (!mcp12.begin(0x65, &Wire1)) {
    Serial.println("WARNING!! Couldnt detect mcp12(0x65)");
    bitSet(mcpExist, 11);
  } else {
    mcp12.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp12.setThermocoupleType(MCP9600_TYPE_K);
    mcp12.enable(true);
  }

  Serial.println("MCP13 Initialization...");
  if (!mcp13.begin(0x66, &Wire1)) {
    Serial.println("WARNING!! Couldnt detect mcp13(0x66)");
    bitSet(mcpExist, 12);
  } else {
    mcp13.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp13.setThermocoupleType(MCP9600_TYPE_K);
    mcp13.enable(true);
  }

  Serial.println("MCP14 Initialization...");
  if (!mcp14.begin(0x67, &Wire1)) {
    Serial.println("WARNING!! Couldnt detect mcp14(0x67)");
    bitSet(mcpExist, 13);
  } else {
    mcp14.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp14.setThermocoupleType(MCP9600_TYPE_K);
    mcp14.enable(true);
  }

  Serial.println("IO setup complete");
}

/*
    Name:         flowMeter1Interrupt
    Arguments:    void
    Returns:      nil
    Description:  interrupt called each time the pin is taken low
*/
// WARNING! No Serial print statements inside an interrupt!
void flowMeter1Interrupt() 
{
  if ((micros() - lastpulse1readTime) > 20){    // 20us = 5kHz
    // Increment the counter by 1. 
    if(!digitalRead(48)){
      set_config_parameter(CONFIG_PARAM_PULSE1, get_config_parameter(CONFIG_PARAM_PULSE1) + 1);
    }
  }
  lastpulse1readTime = micros();  // This line also resolves the cases where an overflow has occurred. 
}

void flowMeter2Interrupt() 
{
  if ((micros() - lastpulse2readTime) > 20){    // 20us = 5kHz
    // Increment the counter by 1. 
    if(!digitalRead(50)){
    set_config_parameter(CONFIG_PARAM_PULSE2, get_config_parameter(CONFIG_PARAM_PULSE2) + 1);
    }
  }
  lastpulse2readTime = micros();  // This line also resolves the cases where an overflow has occurred. 
}

void flowMeter3Interrupt() 
{
  if ((micros() - lastpulse3readTime) > 20){    // 20us = 5kHz
    // Increment the counter by 1. 
    if(!digitalRead(52)){
    set_config_parameter(CONFIG_PARAM_PULSE3, get_config_parameter(CONFIG_PARAM_PULSE3) + 1);
    }
  }
  lastpulse3readTime = micros();  // This line also resolves the cases where an overflow has occurred. 
}
/*
    Name:         digital_io_loop
    Arguments:    void
    Returns:      nil
    Description:  Update all of the digital IO values from all inputs and expanders connected to the Giga. All updated values are transferred to the 
                  config_c200 class for storage and retrieval elsewhere in the code. By storing data in the config_c200 class data can be 
                  accessed in a single place in all other classes inlcuding Modbus, CompressorCore, USB etc. 
*/
void digital_io_loop() {
  uint8_t exp_return_val = 0;
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_24_1)){
    exp4_io.digitalWrite(P3_3, HIGH);
  }else{
    exp4_io.digitalWrite(P3_3, LOW);
  }

  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_24_2)){
    exp4_io.digitalWrite(P3_4, HIGH);
  }else{
    exp4_io.digitalWrite(P3_4, LOW);
  }


  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_24_4)){
    exp4_io.digitalWrite(P3_6, HIGH);
  }else{
    exp4_io.digitalWrite(P3_6, LOW);
  }

  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_24_6)){
    exp5_io.digitalWrite(P1_2, HIGH);
  }else{
    exp5_io.digitalWrite(P1_2, LOW);
  }

  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_24_7)){
    exp5_io.digitalWrite(P1_3, HIGH);
  }else{
    exp5_io.digitalWrite(P1_3, LOW);
  }

  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_24_8)){
    exp5_io.digitalWrite(P0_0, HIGH);
  }else{
    exp5_io.digitalWrite(P0_0, LOW);
  }

  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_24_9)){
    exp5_io.digitalWrite(P0_1, HIGH);
  }else{
    exp5_io.digitalWrite(P0_1, LOW);
  }

  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_24_10)){
    exp5_io.digitalWrite(P0_2, HIGH);
  }else{
    exp5_io.digitalWrite(P0_2, LOW);
  }

  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_24_11)){
    exp5_io.digitalWrite(P0_3, HIGH);
  }else{
    exp5_io.digitalWrite(P0_3, LOW);
  }

  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_24_12)){
    exp5_io.digitalWrite(P0_4, HIGH);
  }else{
    exp5_io.digitalWrite(P0_4, LOW);
  }

  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_24_13)){
    exp5_io.digitalWrite(P0_5, HIGH);
  }else{
    exp5_io.digitalWrite(P0_5, LOW);
  }

  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_24_14)){
    exp5_io.digitalWrite(P0_6, HIGH);
  }else{
    exp5_io.digitalWrite(P0_6, LOW);
  }

  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_24_15)){ // Coolant pump
    exp5_io.digitalWrite(P0_7, HIGH);
  }else{
    exp5_io.digitalWrite(P0_7, LOW);
  }

  // if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_24_RS485)){
  //     exp5_io.digitalWrite(P2_1, HIGH);
  //   }else{
  //     exp5_io.digitalWrite(P2_1, LOW);
  //   }

  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_24_34)){
    exp5_io.digitalWrite(P2_2, HIGH);
  }else{
    exp5_io.digitalWrite(P2_2, LOW);
  }

  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_24_35)){
    exp5_io.digitalWrite(P2_3, HIGH);
  }else{
    exp5_io.digitalWrite(P2_3, LOW);
  }

  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_SSR21A)){
    exp5_io.digitalWrite(P1_0, HIGH);
  }else{
    exp5_io.digitalWrite(P1_0, LOW);
  }

 if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_SSR22A)){
    exp5_io.digitalWrite(P1_1, HIGH);
  }else{
    exp5_io.digitalWrite(P1_1, LOW);
  }

  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_SSR23A)){
    exp5_io.digitalWrite(P1_5, HIGH);
  }else{
    exp5_io.digitalWrite(P1_5, LOW);
  }

 // ******* INPUTS ******************
  exp4_io.digitalRead(P0_0, &exp_return_val);   // Global Interlock
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_0);
  exp4_io.digitalRead(P0_1, &exp_return_val);   // Local Interlock Loop Input
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_1); 
  exp4_io.digitalRead(P0_2, &exp_return_val);   // Remote Reset
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_2);
  exp4_io.digitalRead(P0_4, &exp_return_val);   // Amber Pilot Input
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_4);
  exp4_io.digitalRead(P0_6, &exp_return_val);   // Suction XV Input
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_6);
  exp4_io.digitalRead(P1_3, &exp_return_val);   // Fan 1
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_11);
  exp4_io.digitalRead(P1_4, &exp_return_val);   // Fan 2
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_12);
  exp4_io.digitalRead(P1_5, &exp_return_val);   // Pump 1
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_13);
  exp4_io.digitalRead(P1_6, &exp_return_val);   // Pump 2
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_14);
  exp4_io.digitalRead(P1_7, &exp_return_val);   // Coolant Pump Input
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_15);
  exp4_io.digitalRead(P2_0, &exp_return_val);   // Coolant Pump Latch
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_16);
  exp4_io.digitalRead(P2_1, &exp_return_val);   // Local Estop
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_18);
  exp4_io.digitalRead(P2_3, &exp_return_val);   // Amber button Input
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_20);
  exp4_io.digitalRead(P2_5, &exp_return_val);   // LSR Input
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_22);
  exp4_io.digitalRead(P2_7, &exp_return_val);   // Coolant Flow switch
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_24);
  exp4_io.digitalRead(P3_1, &exp_return_val);   // Contamination Sensor S1D
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_26);
  exp4_io.digitalRead(P3_2, &exp_return_val);   // Contamination Sensor S2D
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_27);
  exp5_io.digitalRead(P1_4, &exp_return_val);   //  Comparator
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_28);
  exp5_io.digitalRead(P1_6, &exp_return_val);   // Suction Flow Meter
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_30);
  exp5_io.digitalRead(P1_7, &exp_return_val);   // Discharge Flow Meter
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_31);
  exp4_io.digitalRead(P0_3, &exp_return_val);   // HYD 1 Ready
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_36);
  exp4_io.digitalRead(P0_5, &exp_return_val);   // HYD 2 Ready
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_37);
  exp4_io.digitalRead(P1_2, &exp_return_val);   // HYD 3 Ready
  set_config_bit(CONFIG_PARAM_INPUTS, exp_return_val, INPUT_BIT_24_38);
  

  // ble_init below it is called with exp_return_val inverted
  // Only update the BLE name on change of the switch state. This will keep out speed up and prevent continual
  // renaming of the BLE radio module without changes.
  exp5_io.digitalRead(P3_7, &exp_return_val);  // For some unknown reason P3-7 reads in reverse to P4_0 and P4_1 so when calling
  if ((exp_return_val && !test_config_parameter(SWITCH_STATE_ALL, SWITCH_STATE_BLE_NAME))
      || (!exp_return_val && test_config_parameter(SWITCH_STATE_ALL, SWITCH_STATE_BLE_NAME))) {
    set_config_bit(SWITCH_STATE_ALL, exp_return_val, SWITCH_STATE_BLE_NAME);
    // For some unknown reason P3_7 reads in reverse to P4_0 and P4_1 so when calling
    // ble_init below it is called with exp_return_val inverted (Use !exp_return_val instead)
    if (!test_config_parameter(SWITCH_STATE_ALL, SWITCH_STATE_BLE_NAME)){
      watchdog_disable();  // Dip switch #3 enable (ble_init) take more than 1.5 seconds which will cause a WDT reset
      ble_init();  // Re-initialize the BLE module to a new name
      watchdog_enable();  // Don't forget to turn the WDT on again.
    }
  }

  //DIP Switch Inputs
  exp5_io.digitalRead(P4_1, &exp_return_val);
  set_config_bit(SWITCH_STATE_ALL, exp_return_val, DIP_BIT_0);
  exp5_io.digitalRead(P4_0, &exp_return_val);
  set_config_bit(SWITCH_STATE_ALL, exp_return_val, DIP_BIT_1);
  exp5_io.digitalRead(P3_7, &exp_return_val);
  set_config_bit(SWITCH_STATE_ALL, exp_return_val, SWITCH_STATE_BLE_NAME);
  exp5_io.digitalRead(P3_6, &exp_return_val);
  set_config_bit(SWITCH_STATE_ALL, exp_return_val, SWITCH_STATE_WIFI_DISABLE);
  exp5_io.digitalRead(P3_5, &exp_return_val);
  set_config_bit(SWITCH_STATE_ALL, exp_return_val, DIP_BIT_4);
  exp5_io.digitalRead(P3_4, &exp_return_val);
  set_config_bit(SWITCH_STATE_ALL, exp_return_val, DIP_BIT_5);
  exp5_io.digitalRead(P3_3, &exp_return_val);
  set_config_bit(SWITCH_STATE_ALL, exp_return_val, DIP_BIT_6);
  exp5_io.digitalRead(P3_2, &exp_return_val);
  set_config_bit(SWITCH_STATE_ALL, exp_return_val, DIP_BIT_7);
  exp5_io.digitalRead(P3_1, &exp_return_val);
  set_config_bit(SWITCH_STATE_ALL, exp_return_val, DIP_BIT_8);
  exp5_io.digitalRead(P3_0, &exp_return_val);
  set_config_bit(SWITCH_STATE_ALL, exp_return_val, DIP_BIT_9);
  exp5_io.digitalRead(P2_7, &exp_return_val);
  set_config_bit(SWITCH_STATE_ALL, exp_return_val, DIP_BIT_10);
  exp5_io.digitalRead(P2_6, &exp_return_val);
  set_config_bit(SWITCH_STATE_ALL, exp_return_val, DIP_BIT_11);

}

float potToTemp(float potReading, float BCOEFFICIENT_VALUE) {
  float potResistance = 4096 / potReading - 1;
  potResistance = SERIESRESISTOR / potResistance;  //Convert Pot Reading to resistance
  //----------Apply steinhart equation to reading-----------//
  float steinhart = log(potResistance / THERMISTORNOMINAL);                                                                            // ln(R/Ro)
  steinhart /= BCOEFFICIENT_VALUE;                                                                                                     // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);                                                                                    // + (1/To)
  steinhart = 1.0 / steinhart;                                                                                                         // Invert
  float potTemperature = steinhart + .0000007318 * pow(steinhart, 3) - .001011 * pow(steinhart, 2) + .4318 * steinhart - 58 - 273.15;  // magic and convert absolute temp to C
  return potTemperature;
}

/*
    Name:         analog_io_loop
    Arguments:    void
    Returns:      nil
    Description:  Update all of the analog IO values from all inputs and expanders connected to the Giga. All updated values are transferred to the 
                  config_c200 class for storage and retrieval elsewhere in the code. By storing data in the config_c200 class data can be 
                  accessed in a single place in all other classes inlcuding Modbus, CompressorCore, USB etc. 
*/
void analog_io_loop() {
    coolant1_pt =  map(avgPT1.filter(COOLANT_PT1_EXP.read(COOLANT_PT1)), 819, 4095, 0, 300);  //COOLANT1
    if (coolant1_pt < -30) {  // Threshold for disconnected
     coolant1_pt = -1;
    } else if (coolant1_pt < 0) {  // A slightly negative pressure +/- 1% can be approximated to 0psi
      coolant1_pt = 0;
    }
    set_config_parameter(CONFIG_PARAM_COOLANT1_PT, coolant1_pt); //TB30
    set_config_parameter(CONFIG_PARAM_COOLANT2_PT, map(avgPT2.filter(COOLANT_PT2_EXP.read(COOLANT_PT2)), 819, 4095, 0, 300));  //TB31
    set_config_parameter(CONFIG_PARAM_SUCTION_PT, map(avgPT10.filter(PT_Exp2.read(SUCTION_PT)), 819, 4095, 0, 2000));  //TB19-PT405
    set_config_parameter(CONFIG_PARAM_SUCTION_TANK, map(avgPT11.filter(PT_Exp2.read(SCT_TANK_PT)), 819, 4095, 0, 2000)); //TB20-PT426
    set_config_parameter(CONFIG_PARAM_STAGE1_TANK, map(avgPT13.filter(PT_Exp2.read(S1_TANK)), 819, 4095, 0, 2000));  //TB22-PT442
    set_config_parameter(CONFIG_PARAM_STAGE2_TANK, map(avgPT15.filter(PT_Exp2.read(S2_TANK)), 819, 4095, 0, 5000));  //TB24-PT457 
    set_config_parameter(CONFIG_PARAM_STAGE3_TANK, map(avgPT5.filter(PT_Exp1.read(S3_TANK)), 819, 4095, 0, 20000));  //TB25-PT468
    set_config_parameter(CONFIG_PARAM_STAGE3_PT, map(avgPT14.filter(PT_Exp2.read(S3_PT)), 819, 4095, 0, 20000));  //TB23-PT487
    set_config_parameter(CONFIG_PARAM_HFL_PUMP1, map(avgPT12.filter(PT_Exp2.read(HFL_PUMP)), 819, 4095, 0, 3000));  //TB21-PT496

   
 errCnt = 0;
  for (int i = 0; i < 30; i++) {
    errMsg[i] = "";
  }
  for (int i = 0; i < TT_SIZE; i++) {
    if (TTdata[i].channel == -1) {
      Adafruit_MCP9601 *mcpSensor = nullptr;
      switch (TTdata[i].mcp) {
        case 1: mcpSensor = &mcp1; break;
        case 2: mcpSensor = &mcp2; break;
        case 3: mcpSensor = &mcp3; break;
        case 4: mcpSensor = &mcp4; break;
        case 5: mcpSensor = &mcp5; break;
        case 6: mcpSensor = &mcp6; break;
        case 7: mcpSensor = &mcp7; break;
        case 8: mcpSensor = &mcp8; break;
        case 9: mcpSensor = &mcp9; break;
        case 10: mcpSensor = &mcp10; break;
        case 11: mcpSensor = &mcp11; break;
        case 12: mcpSensor = &mcp12; break;
        case 13: mcpSensor = &mcp13; break;
        case 14: mcpSensor = &mcp14; break; 
        default: break;
      }

      if (mcpSensor && !bitRead(mcpExist, TTdata[i].mcp - 1)) {
        uint8_t status = mcpSensor->getStatus();

        // Check for open circuit
        if (status & MCP9601_STATUS_OPENCIRCUIT) {
          TTdata[i].rawTemp = 5000;  // Set to 5000 for open circuit
          errMsg[errCnt++] = "Thermocouple open!";
        } else {
          // No open circuit detected, proceed with temperature reading
          TTdata[i].rawTemp = mcpSensor->readThermocouple();
          
          // If a short circuit is detected, you still get the value from the sensor
          if (status & MCP9601_STATUS_SHORTCIRCUIT) {
            errMsg[errCnt++] = "Thermocouple shorted!";
          }
        }

        // Update the value regardless of the error if not in open state
        if (TTdata[i].rawTemp != 10 || (status & MCP9601_STATUS_OPENCIRCUIT) == 0) {
          // Do NOT add a sensor disconnect value to the sensor averaging!!
          if (TTdata[i].rawTemp < 5000){
            *TTdata[i].value = TTdata[i].avg.filter(TTdata[i].rawTemp);
            TTdata[i].failCtr = 0; // Reset the error/sensor fail counter
          }else{
            // If we have had more than 3 consequtive failed reads then place the raw value directly onto the 
            // sensor value (Do not use the averaging if the sensor has failed). If this railed read was a 
            // 1 off, then simply disregard it
            if (TTdata[i].failCtr > 5){
              // No averaging on crazy number!
              *TTdata[i].value = TTdata[i].rawTemp;
            }else{
              TTdata[i].failCtr++;
            }
          }

          // Set the configuration parameter based on the index
          switch (i) {
            case 0: set_config_parameter(CONFIG_PARAM_TC1, *TTdata[i].value); break;
            case 1: set_config_parameter(CONFIG_PARAM_TC2, *TTdata[i].value); break;
            case 2: set_config_parameter(CONFIG_PARAM_TC3, *TTdata[i].value); break;
            case 3: set_config_parameter(CONFIG_PARAM_TC4, *TTdata[i].value); break;
            case 4: set_config_parameter(CONFIG_PARAM_TC5, *TTdata[i].value); break;
            case 5: set_config_parameter(CONFIG_PARAM_TC6, *TTdata[i].value); break;
            case 6: set_config_parameter(CONFIG_PARAM_TC7, *TTdata[i].value); break;
            case 7: set_config_parameter(CONFIG_PARAM_TC8, *TTdata[i].value); break;
            case 8: set_config_parameter(CONFIG_PARAM_TC9, *TTdata[i].value); break;
            case 9: set_config_parameter(CONFIG_PARAM_TC10, *TTdata[i].value); break;
            case 10: set_config_parameter(CONFIG_PARAM_TC11, *TTdata[i].value); break;
            case 11: set_config_parameter(CONFIG_PARAM_TC12, *TTdata[i].value); break;
            case 12: set_config_parameter(CONFIG_PARAM_TC13, *TTdata[i].value); break;
            case 13: set_config_parameter(CONFIG_PARAM_TC14, *TTdata[i].value); break;
            default: break;
          }
        }
      }
    }  // end if(TTdata[i].channel == -1){
  }


   // *************** ANALOG OUTPUTS ****************************
  dac_Exp7.analogWrite(MCP4728::DAC_CH::A, fmap(get_config_parameter(CONFIG_PARAM_PV1_CL), 0, 100, 0, 4095));    //4-20mA Current source
  dac_Exp7.analogWrite(MCP4728::DAC_CH::B, fmap(get_config_parameter(CONFIG_PARAM_PV2_CL), 0, 100, 0, 4095));    //4-20mA Current source
  dac_Exp7.analogWrite(MCP4728::DAC_CH::C, fmap(get_config_parameter(CONFIG_PARAM_PV3_CL), 0, 100, 0, 4095));    //4-20mA Current source
  //  dac_Exp7.analogWrite(MCP4728::DAC_CH::D, fmap(get_config_parameter(CONFIG_PARAM_PV4_CL), 0, 100, 0, 4095));    //4-20mA Current source
 }