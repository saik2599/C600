/*
   Author: Christopher Glanville
   io_c200.h configuration header file. To acompany io_c200.cpp
   Date: 25/5/24
   updated : 25/5/24
*/

#if !defined(_IO_C200_H)
  #define  _IO_C200_H
  #include "C200.h"

  #define BUTTON_PRESS          true
  #define BUTTON_RELEASE        false

  void io_setup(void);
  void digital_io_loop(void);
  void analog_io_loop(void);
  bool get_estop_button_state(void);
  void set_green_pilot_state(bool);
  void set_amber_pilot_state(bool);
  void set_amber_pilot_state_xx(bool);
  float potToTemp(float,float);
  bool levelSwicthDebounce(bool);
  void flowMeter1Interrupt(void);
  void flowMeter2Interrupt(void);
  void flowMeter3Interrupt(void);
#endif
