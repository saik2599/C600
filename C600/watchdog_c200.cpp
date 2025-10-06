
#include <stdint.h>
#include "watchdog_c200.h"
#include <Arduino.h>
 

void watchdog_setup(){
  pinMode(D26, OUTPUT);
  digitalWrite(D26, HIGH);
  delay(3000);
  digitalWrite(D26,LOW);
  // delay(2000);
}


void watchdog_enable(void){
  digitalWrite(D26, LOW);
  Serial.println("Watchdog Timer Enabled ");
  delay(1000);
}

void watchdog_disable(void){
  digitalWrite(D26, HIGH);
  Serial.println("Watchdog Timer Disabled");
  delay(1000);
}