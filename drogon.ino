/*
 * Drogon : Main
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

#include <Servo.h>

const int MOTOR_PIN0 = 2;
const int MOTOR_PIN1 = 3;
const int MOTOR_PIN2 = 4;
const int MOTOR_PIN3 = 5;

const int RECEIVER_PIN = 7;

const int MOTOR_LED_PIN0 = 8;
const int MOTOR_LED_PIN1 = 9;
const int MOTOR_LED_PIN2 = 10;
const int MOTOR_LED_PIN3 = 11;

const int READY_LED_PIN = 13;
const int ARMED_LED_PIN = 12;

const int ACCEL_PIN_X = 0;
const int ACCEL_PIN_Y = 1;
const int ACCEL_PIN_Z = 2;

const int IR_PIN = 3;

const int MIN_MOTOR_VALUE = 21;
const int MAX_MOTOR_VALUE = 180;

const long STATE_BUFFER_TIME = 500;

const int STATE_PENDING = 0;
const int STATE_READY = 0;
const int STATE_ARMED = 0;

int state;

Servo motor0;
Servo motor1;
Servo motor2;
Servo motor3;

int motorValue0;
int motorValue1;
int motorValue2;
int motorValue3;

long stateBufferExpires;

const long RECEIVER_ARMING_TIME = 2000;
boolean receiverArming = true;
long receiverArmingEnding;

void setup() {
  Serial.begin(9600);
  
  motor0.attach( MOTOR_PIN0 );
  motor1.attach( MOTOR_PIN1 );
  motor2.attach( MOTOR_PIN2 );
  motor3.attach( MOTOR_PIN3 );
  
  pinMode( MOTOR_LED_PIN0, OUTPUT );
  pinMode( MOTOR_LED_PIN1, OUTPUT );
  pinMode( MOTOR_LED_PIN2, OUTPUT );
  pinMode( MOTOR_LED_PIN3, OUTPUT );
  
  pinMode( READY_LED_PIN, OUTPUT );
  pinMode( ARMED_LED_PIN, OUTPUT );
  
  digitalWrite( MOTOR_LED_PIN0, LOW );
  digitalWrite( MOTOR_LED_PIN1, LOW );
  digitalWrite( MOTOR_LED_PIN2, LOW );
  digitalWrite( MOTOR_LED_PIN3, LOW );
  
  digitalWrite( READY_LED_PIN, LOW );
  digitalWrite( ARMED_LED_PIN, LOW );
  
  motorValue0 = MIN_MOTOR_VALUE;
  motorValue1 = MIN_MOTOR_VALUE;
  motorValue2 = MIN_MOTOR_VALUE;
  motorValue3 = MIN_MOTOR_VALUE;
  
  receiver_setup();
  
  led_setup();
  accel_setup();
  gyro_setup();
  
  state = STATE_PENDING;
  
  stateBufferExpires = millis();
  
  receiverArming = false;
  receiverArmingEnding = millis();
}

void loop() {
  accel_loop();
  gyro_loop();
  
  if ( millis() < stateBufferExpires ) return;
  
  if ( state == STATE_PENDING ) {
    if ( receiver_ready() && gyro_ready() && accel_ready() ) {
      state = STATE_READY;
      digitalWrite( READY_LED_PIN, HIGH );
      stateBufferExpires = millis() + STATE_BUFFER_TIME;
    }
  } else if ( state == STATE_READY ) {
    if ( receiverArming ) {
      if ( receiver_get_value( 5 ) < 80 || receiver_get_value( 6 ) < 80 ) {
        if ( millis() > receiverArmingEnding ) {
          state = STATE_ARMED;
          digitalWrite( ARMED_LED_PIN, HIGH );
          stateBufferExpires = millis() + STATE_BUFFER_TIME;
        }
        receiverArming = false;
      }
    } else {
      if ( receiver_get_value( 5 ) >= 80 && receiver_get_value( 6 ) >= 80 ) {
        receiverArming = true;
        receiverArmingEnding = millis() + RECEIVER_ARMING_TIME;
      }
    }
  } else if ( state == STATE_ARMED ) {
    if ( receiverArming ) {
      if ( receiver_get_value( 5 ) < 80 || receiver_get_value( 6 ) < 80 ) {
        if ( millis() > receiverArmingEnding ) {
          state = STATE_READY;
          digitalWrite( ARMED_LED_PIN, LOW );
          stateBufferExpires = millis() + STATE_BUFFER_TIME;
          
          zero_motors();
        } else {
          receiverArming = false;
        }
      }
    } else {
      if ( receiver_get_value( 5 ) >= 80 && receiver_get_value( 6 ) >= 80 ) {
        receiverArming = true;
        receiverArmingEnding = millis() + RECEIVER_ARMING_TIME;
      } else {
        control_loop();
      }
    }
  }
}

void control_loop() {
  map_reciever();
  //update_motors();
}

void map_reciever() {
}

void update_motors() {
  motor0.write( motorValue0 );
  motor1.write( motorValue1 );
  motor2.write( motorValue2 );
  motor3.write( motorValue3 );
  
  led_blink( motorValue0, motorValue1, motorValue2, motorValue3 );
}

void zero_motors() {
  motorValue0 = MIN_MOTOR_VALUE;
  motorValue1 = MIN_MOTOR_VALUE;
  motorValue2 = MIN_MOTOR_VALUE;
  motorValue3 = MIN_MOTOR_VALUE;
  
  update_motors();
}

