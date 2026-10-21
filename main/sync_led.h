// Front-panel sync LED on AP_PIN_SYNC_LED (active high, LED + resistor to GND):
//   on          PTP locked to the leader and the media clock disciplined
//   slow blink  PTP locked, media clock not yet disciplined
//   fast blink  a leader is heard, PTP not locked yet
//   off         no PTP leader

#ifndef SYNC_LED_H
#define SYNC_LED_H

#include "esp_err.h"

esp_err_t sync_led_start(void);

#endif // SYNC_LED_H
