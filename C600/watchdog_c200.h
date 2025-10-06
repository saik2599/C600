/*
   Author: Piyush Sharma
   Watchdog Timer Implementation 
   Date: 3/4/24
   updated : 3/4/24
   File to setup watchdog timer
*/

#ifndef WATCHDOG_C200_H
  #define  WATCHDOG_C200_H
void watchdog_setup(void);
void watchdog_enable(void);
void watchdog_disable(void);
#endif;