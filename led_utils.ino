/*
 * Drogon : LED Utilities
 *
 * This file is part of Drogon.
 *
 * Drogon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Drogon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Drogon.  If not, see <http://www.gnu.org/licenses/>. 
 *
 * Author: Joseph Monti <joe.monti@gmail.com>
 * Copyright (c) 2013 Joseph Monti All Rights Reserved, http://joemonti.org/
 */

const int LED_MIN_DELAY = 50;
const int LED_MAX_DELAY = 500;

const int NUM_LEDS = 4;

int ledPin[NUM_LEDS];
boolean lastLed[NUM_LEDS];
int nextLedMillis[NUM_LEDS];

void led_setup() {
  for ( int i = 0; i < NUM_LEDS; i++ ) {
    lastLed[i] = LOW;
    nextLedMillis[i] = millis();
  }
  
  ledPin[0] = MOTOR_LED_PIN0;
  ledPin[1] = MOTOR_LED_PIN1;
  ledPin[2] = MOTOR_LED_PIN2;
  ledPin[3] = MOTOR_LED_PIN3;
}

void led_blink( int value1, int value2, int value3, int value4 ) {
  led_blink_led( 0, value1 );
  led_blink_led( 1, value2 );
  led_blink_led( 2, value3 );
  led_blink_led( 3, value4 );
}

void led_blink_led( int index, int value ) {
  if ( millis() >= nextLedMillis[index] ) {
    lastLed[index] = !lastLed[index];
    digitalWrite( ledPin[index], lastLed[index] );
    
    nextLedMillis[index] = millis() + ( map( MAX_MOTOR_VALUE-value, 0, MAX_MOTOR_VALUE, LED_MIN_DELAY, LED_MAX_DELAY ) ) ;
  }
}

