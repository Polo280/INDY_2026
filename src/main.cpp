#include "ELYOS_DRIVER.h"

ELYOS_DRIVER driver;
// Blink aux 
uint32_t blink_aux = 0;

int main(){

  // Setup 
  pinMode(LED_BUILTIN, OUTPUT);
  driver.driver_Init();

  while(true){
    // Make it more deterministic
    uint32_t t0 = micros();
    driver.runFOC();
    while(micros() - t0 < 50);   // 20 kHz loop

    // Blink LED
    if(millis() - blink_aux >= 500){
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      blink_aux = millis();
    }
  }
  return 0;
}