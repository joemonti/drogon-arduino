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

const boolean DEBUG = true;

const int MOTOR_PIN0 = 8;
const int MOTOR_PIN1 = 9;
const int MOTOR_PIN2 = 10;
const int MOTOR_PIN3 = 11;

const int RECEIVER_PIN = 44;

const int MOTOR_LED_PIN0 = 46;
const int MOTOR_LED_PIN1 = 48;
const int MOTOR_LED_PIN2 = 50;
const int MOTOR_LED_PIN3 = 52;

const int READY_LED_PIN = 13;
const int ARMED_LED_PIN = 12;

const int ACCEL_PIN_X = 0;
const int ACCEL_PIN_Y = 1;
const int ACCEL_PIN_Z = 2;

//const int IR_PIN = 3;

const int MIN_MOTOR_VALUE = 1000;
const int MAX_MOTOR_VALUE = 2000; //2000;
const int MOTOR_VALUE_RANGE = ( MAX_MOTOR_VALUE - MIN_MOTOR_VALUE ) / 2;
const float MOTOR_DIFF_SCALE = 0.1;

const long STATE_BUFFER_TIME = 500;
const long STATE_BUFFER_TIME_ARMED = 2000;

const int STATE_PENDING = 0;
const int STATE_READY = 1;
const int STATE_ARMED = 2;

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

const long RECEIVER_ARMING_TIME = 3000;
const long RECEIVER_ARMING_COOLDOWN_TIME = 1000;
const int RECEIVER_ARMING_IDLE = 0;
const int RECEIVER_ARMING_PENDING = 1;
const int RECEIVER_ARMING_COOLDOWN = 2;
int receiverArmingState;
long receiverArmingEnding;
long receiverArmingCooldown;

const long CONTROL_LOOP_FREQUENCY = 10;
long nextControlLoop;

const long LOG_FREQUENCY = 200;
long nextLogTime;

void setup() {
  Serial1.begin(9600);
  
  if (DEBUG) Serial1.println("SETUP STARTED");
  
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
  
  led_setup( MOTOR_LED_PIN0,
             MOTOR_LED_PIN1,
             MOTOR_LED_PIN2,
             MOTOR_LED_PIN3 );
  
  accel_setup();
  gyro_setup();
  
  state = STATE_PENDING;
  
  stateBufferExpires = millis();
  
  receiverArmingState = RECEIVER_ARMING_IDLE;
  receiverArmingEnding = millis();
  receiverArmingCooldown = millis();
  nextLogTime = millis();
  nextControlLoop = millis();
  
  if (DEBUG) Serial1.println("SETUP FINISHED");
  if (DEBUG) Serial1.println("STATE : PENDING");
}

void loop() {
  accel_loop();
  gyro_loop();
  
  log_data();
  
  if ( millis() < stateBufferExpires ) return;
  
  if ( state == STATE_PENDING ) {
    if ( receiver_ready() && gyro_ready() && accel_ready() ) {
      state = STATE_READY;
      digitalWrite( READY_LED_PIN, HIGH );
      stateBufferExpires = millis() + STATE_BUFFER_TIME;
      
      if ( DEBUG ) Serial1.println("STATE : READY");
    }
  } else if ( state == STATE_READY ) {
    if ( receiverArmingState == RECEIVER_ARMING_COOLDOWN ) {
      if ( receiver_get_value( 4 ) != 0 || receiver_get_value( 5 ) != 0 ) {
        if ( DEBUG ) Serial1.println("ARMING COOLDOWN BOUNCING");
        receiverArmingEnding = millis() + RECEIVER_ARMING_COOLDOWN_TIME;
      } else if ( millis() >= receiverArmingEnding ) {
        state = STATE_ARMED;
        digitalWrite( ARMED_LED_PIN, HIGH );
        stateBufferExpires = millis() + STATE_BUFFER_TIME_ARMED;
        
        receiverArmingState = RECEIVER_ARMING_IDLE;
          
        zero_motors();
        if ( DEBUG ) Serial1.println("ARMING FINISHED");
        if ( DEBUG ) Serial1.println("STATE : ARMED");
      }
    } else if ( receiverArmingState == RECEIVER_ARMING_PENDING ) {
      if ( receiver_get_value( 4 ) == 0 && receiver_get_value( 5 ) == 0 ) {
        if ( millis() >= receiverArmingEnding ) {
          receiverArmingState = RECEIVER_ARMING_COOLDOWN;
          receiverArmingEnding = millis() + RECEIVER_ARMING_COOLDOWN_TIME;
          if ( DEBUG ) Serial1.println("ARMING COOLDOWN");
        } else {
          if ( DEBUG ) Serial1.println("ARMING PENDING CANCELLED");
          receiverArmingState = RECEIVER_ARMING_IDLE;
        }
      } else if ( receiver_get_value( 4 ) < 80 || receiver_get_value( 5 ) < 80 ) {
        if ( millis() < receiverArmingEnding ) {
          if ( DEBUG ) Serial1.println("ARMING PENDING CANCELLED");
          receiverArmingState = RECEIVER_ARMING_IDLE;
        }
      }
    } else {
      if ( receiver_get_value( 4 ) >= 80 && receiver_get_value( 5 ) >= 80 ) {
        receiverArmingState = RECEIVER_ARMING_PENDING;
        receiverArmingEnding = millis() + RECEIVER_ARMING_TIME;
        
        if ( DEBUG ) Serial1.println("ARMING START");
      }
    }
  } else if ( state == STATE_ARMED ) {
    if ( receiverArmingState == RECEIVER_ARMING_COOLDOWN ) {
      if ( receiver_get_value( 4 ) != 0 || receiver_get_value( 5 ) != 0 ) {
        if ( DEBUG ) Serial1.println("DISARMING COOLDOWN BOUNCING");
        receiverArmingEnding = millis() + RECEIVER_ARMING_COOLDOWN_TIME;
      } else if ( millis() >= receiverArmingEnding ) {
        state = STATE_READY;
        digitalWrite( ARMED_LED_PIN, LOW );
        stateBufferExpires = millis() + STATE_BUFFER_TIME;
        
        receiverArmingState = RECEIVER_ARMING_IDLE;
        
        zero_motors();
        
        if ( DEBUG ) Serial1.println("DISARMING FINISHED");
        if ( DEBUG ) Serial1.println("STATE : READY");
      }
    } else if ( receiverArmingState == RECEIVER_ARMING_PENDING ) {
      if ( receiver_get_value( 4 ) == 0 && receiver_get_value( 5 ) == 0 ) {
        if ( millis() >= receiverArmingEnding ) {
          receiverArmingState = RECEIVER_ARMING_COOLDOWN;
          receiverArmingEnding = millis() + RECEIVER_ARMING_COOLDOWN_TIME;
          if ( DEBUG ) Serial1.println("DISARMING COOLDOWN");
          
          // disarming has started, shut down while waiting for cooldown
          digitalWrite( ARMED_LED_PIN, LOW );
          zero_motors();
        } else {
          if ( DEBUG ) Serial1.println("DISARMING PENDING CANCELLED");
          receiverArmingState = RECEIVER_ARMING_IDLE;
          
          // disarming cancelled, continue control loop
          control_loop();
        }
      } else if ( receiver_get_value( 4 ) < 80 || receiver_get_value( 5 ) < 80 ) {
        if ( millis() < receiverArmingEnding ) {
          if ( DEBUG ) Serial1.println("DISARMING PENDING CANCELLED");
          receiverArmingState = RECEIVER_ARMING_IDLE;
          
          // disarming cancelled, continue control loop
          control_loop();
        }
      }
    } else {
      if ( receiver_get_value( 4 ) >= 80 && receiver_get_value( 5 ) >= 80 ) {
        receiverArmingState = RECEIVER_ARMING_PENDING;
        receiverArmingEnding = millis() + RECEIVER_ARMING_TIME;
        
        if ( DEBUG ) Serial1.println("DISARMING START");
      }
      
      control_loop();
    }
  }
}

void control_loop() {
  if ( millis() > nextControlLoop ) {
    map_reciever();
    update_motors();
    nextControlLoop = millis() + CONTROL_LOOP_FREQUENCY;
  }
}

void map_reciever() {
  int receiver0 = receiver_get_value(0);
  int receiver1 = receiver_get_value(1);
  int receiver2 = receiver_get_value(2);
  
  int target = max( 0, map( -receiver2, 0, 100, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE ) );
  
  int motorValue0Adj = 
             ( map( receiver1 * MOTOR_DIFF_SCALE, -100, 100, -MOTOR_VALUE_RANGE, MOTOR_VALUE_RANGE ) + 
               map( receiver0 * MOTOR_DIFF_SCALE, -100, 100, -MOTOR_VALUE_RANGE, MOTOR_VALUE_RANGE ) );
  
  int motorValue1Adj = 
             ( map(  receiver1 * MOTOR_DIFF_SCALE, -100, 100, -MOTOR_VALUE_RANGE, MOTOR_VALUE_RANGE ) + 
               map( -receiver0 * MOTOR_DIFF_SCALE, -100, 100, -MOTOR_VALUE_RANGE, MOTOR_VALUE_RANGE ) );
  
  int motorValue2Adj = 
             ( map( -receiver1 * MOTOR_DIFF_SCALE, -100, 100, -MOTOR_VALUE_RANGE, MOTOR_VALUE_RANGE ) + 
               map( -receiver0 * MOTOR_DIFF_SCALE, -100, 100, -MOTOR_VALUE_RANGE, MOTOR_VALUE_RANGE ) );
  
  int motorValue3Adj = 
             ( map( -receiver1 * MOTOR_DIFF_SCALE, -100, 100, -MOTOR_VALUE_RANGE, MOTOR_VALUE_RANGE ) + 
               map(  receiver0 * MOTOR_DIFF_SCALE, -100, 100, -MOTOR_VALUE_RANGE, MOTOR_VALUE_RANGE ) );
  
  motorValue0 = max( min( target + motorValue0Adj, MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
  motorValue1 = max( min( target + motorValue1Adj, MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
  motorValue2 = max( min( target + motorValue2Adj, MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
  motorValue3 = max( min( target + motorValue3Adj, MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
}

void update_motors() {
  motor0.writeMicroseconds( motorValue0 );
  motor1.writeMicroseconds( motorValue1 );
  motor2.writeMicroseconds( motorValue2 );
  motor3.writeMicroseconds( motorValue3 );
  
  led_blink( motorValue0, motorValue1, motorValue2, motorValue3 );
}

void zero_motors() {
  motorValue0 = MIN_MOTOR_VALUE;
  motorValue1 = MIN_MOTOR_VALUE;
  motorValue2 = MIN_MOTOR_VALUE;
  motorValue3 = MIN_MOTOR_VALUE;
  
  update_motors();
}

void log_data() {
  if ( millis() < nextLogTime ) return;
  
  Serial1.print(receiver_get_state());
  Serial1.print('\t');
  
  Serial1.print(receiver_get_value(0));
  Serial1.print('\t');
  Serial1.print(receiver_get_value(1));
  Serial1.print('\t');
  Serial1.print(receiver_get_value(2));
  Serial1.print('\t');
  Serial1.print(receiver_get_value(3));
  Serial1.print('\t');
  Serial1.print(receiver_get_value(4));
  Serial1.print('\t');
  Serial1.print(receiver_get_value(5));
  
  Serial1.print('\t');
  Serial1.print(accel_get(0));
  Serial1.print('\t');
  Serial1.print(accel_get(1));
  Serial1.print('\t');
  Serial1.print(accel_get(2));
  
  Serial1.print('\t');
  Serial1.print(gyro_get(0));
  Serial1.print('\t');
  Serial1.print(gyro_get(1));
  Serial1.print('\t');
  Serial1.print(gyro_get(2));
  
  Serial1.print('\t');
  Serial1.print(motorValue0);
  Serial1.print('\t');
  Serial1.print(motorValue1);
  Serial1.print('\t');
  Serial1.print(motorValue2);
  Serial1.print('\t');
  Serial1.print(motorValue3);
  
  Serial1.println();
  
  nextLogTime = millis() + LOG_FREQUENCY;
}

