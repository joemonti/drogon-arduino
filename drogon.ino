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

#include <DrogonPosition.h>
#include <DrogonController.h>

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


const int X = 0;
const int Y = 1;
const int Z = 2;


const long IDLE_DELAY = 2000;


const int MIN_MOTOR_VALUE = 1000;
const int MAX_MOTOR_VALUE = 2000; //2000;
const float MAX_MOTOR_ADJUST = ( MAX_MOTOR_VALUE - MIN_MOTOR_VALUE ) * 0.25;
const float MIN_MOTOR_ADJUST = -MAX_MOTOR_ADJUST;
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

double accelValues[3];
double gyroValues[3];
int motorValues[4];
double motorAdjusts[4];

long stateBufferExpires;

const long RECEIVER_ARMING_TIME = 3000;
const long RECEIVER_DISARMING_TIME = 500;
const long RECEIVER_ARMING_COOLDOWN_TIME = 1000;
const int RECEIVER_ARMING_IDLE = 0;
const int RECEIVER_ARMING_PENDING = 1;
const int RECEIVER_ARMING_COOLDOWN = 2;
int receiverArmingState;
long receiverArmingEnding;
long receiverArmingCooldown;

const long LOG_FREQUENCY = 100;
long nextLogTime;

DrogonPosition pos;
DrogonController controller(&pos);
const long CONTROL_FREQUENCY = 5000;
unsigned long nextControlTime;
int lastRunDuration;

void setup() {
  Serial1.begin(9600);
  
  if (DEBUG) Serial1.println("SETUP STARTED");
  
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
  
  
  receiver_setup();
  
  led_setup( MOTOR_LED_PIN0,
             MOTOR_LED_PIN1,
             MOTOR_LED_PIN2,
             MOTOR_LED_PIN3 );
  
  accel_setup();
  gyro_setup();
  
  state = STATE_PENDING;
  
  stateBufferExpires = millis();
  
  lastRunDuration = 0;
  receiverArmingState = RECEIVER_ARMING_IDLE;
  receiverArmingEnding = millis();
  receiverArmingCooldown = millis();
  nextLogTime = millis();
  nextControlTime = micros();
  
  if (DEBUG) Serial1.println("5\tSETUP FINISHED");
  if (DEBUG) Serial1.println("5\tSTATE : PENDING");
}

void loop() {
  log_data();
  
  if ( millis() < stateBufferExpires ) return;
  
  if ( state == STATE_ARMED ) {
    if ( receiverArmingState == RECEIVER_ARMING_COOLDOWN ) {
      if ( receiver_get_value( 4 ) != 0 || receiver_get_value( 5 ) != 0 ) {
        if ( DEBUG ) Serial1.println("5\tDISARMING COOLDOWN BOUNCING");
        receiverArmingEnding = millis() + RECEIVER_ARMING_COOLDOWN_TIME;
      } else if ( millis() >= receiverArmingEnding ) {
        state = STATE_READY;
        stateBufferExpires = millis() + STATE_BUFFER_TIME;
        
        receiverArmingState = RECEIVER_ARMING_IDLE;
        
        if ( DEBUG ) Serial1.println("5\tDISARMING FINISHED");
        if ( DEBUG ) Serial1.println("5\tSTATE : READY");
      }
    } else if ( receiverArmingState == RECEIVER_ARMING_PENDING ) {
      if ( receiver_get_value( 4 ) == 0 && receiver_get_value( 5 ) == 0 ) {
        if ( millis() >= receiverArmingEnding ) {
          receiverArmingState = RECEIVER_ARMING_COOLDOWN;
          receiverArmingEnding = millis() + RECEIVER_ARMING_COOLDOWN_TIME;
          if ( DEBUG ) Serial1.println("5\tDISARMING COOLDOWN");
          
          // disarming has started, shut down while waiting for cooldown
          disarm_motors();
        } else {
          if ( DEBUG ) Serial1.println("5\tDISARMING PENDING CANCELLED");
          receiverArmingState = RECEIVER_ARMING_IDLE;
          
          // disarming cancelled, continue control loop
          control_loop();
        }
      } else if ( receiver_get_value( 4 ) < 80 || receiver_get_value( 5 ) < 80 ) {
        if ( millis() < receiverArmingEnding ) {
          if ( DEBUG ) Serial1.println("5\tDISARMING PENDING CANCELLED");
          receiverArmingState = RECEIVER_ARMING_IDLE;
          
          // disarming cancelled, continue control loop
          control_loop();
        }
      }
    } else {
      if ( receiver_get_value( 4 ) >= 80 && receiver_get_value( 5 ) >= 80 ) {
        receiverArmingState = RECEIVER_ARMING_PENDING;
        receiverArmingEnding = millis() + RECEIVER_DISARMING_TIME;
        
        if ( DEBUG ) Serial1.println("5\tDISARMING START");
      }
      
      control_loop();
    }
  } else if ( state == STATE_PENDING ) {
    accel_update();
    gyro_update();
    
    if ( receiver_ready() && gyro_ready() && accel_ready() ) {
      state = STATE_READY;
      digitalWrite( READY_LED_PIN, HIGH );
      stateBufferExpires = millis() + STATE_BUFFER_TIME;
      
      if ( DEBUG ) Serial1.println("5\tSTATE : READY");
    }
  } else if ( state == STATE_READY ) {
    position_loop();
    
    if ( receiverArmingState == RECEIVER_ARMING_COOLDOWN ) {
      if ( receiver_get_value( 4 ) != 0 || receiver_get_value( 5 ) != 0 ) {
        if ( DEBUG ) Serial1.println("5\tARMING COOLDOWN BOUNCING");
        receiverArmingEnding = millis() + RECEIVER_ARMING_COOLDOWN_TIME;
      } else if ( millis() >= receiverArmingEnding ) {
        state = STATE_ARMED;
        stateBufferExpires = millis() + STATE_BUFFER_TIME_ARMED;
        
        receiverArmingState = RECEIVER_ARMING_IDLE;
        
        arm_motors();
        if ( DEBUG ) Serial1.println("5\tARMING FINISHED");
        if ( DEBUG ) Serial1.println("5\tSTATE : ARMED");
      }
    } else if ( receiverArmingState == RECEIVER_ARMING_PENDING ) {
      if ( receiver_get_value( 4 ) == 0 && receiver_get_value( 5 ) == 0 ) {
        if ( millis() >= receiverArmingEnding ) {
          receiverArmingState = RECEIVER_ARMING_COOLDOWN;
          receiverArmingEnding = millis() + RECEIVER_ARMING_COOLDOWN_TIME;
          if ( DEBUG ) Serial1.println("5\tARMING COOLDOWN");
        } else {
          if ( DEBUG ) Serial1.println("5\tARMING PENDING CANCELLED");
          receiverArmingState = RECEIVER_ARMING_IDLE;
        }
      } else if ( receiver_get_value( 4 ) < 80 || receiver_get_value( 5 ) < 80 ) {
        if ( millis() < receiverArmingEnding ) {
          if ( DEBUG ) Serial1.println("5\tARMING PENDING CANCELLED");
          receiverArmingState = RECEIVER_ARMING_IDLE;
        }
      }
    } else {
      if ( receiver_get_value( 4 ) >= 80 && receiver_get_value( 5 ) >= 80 ) {
        receiverArmingState = RECEIVER_ARMING_PENDING;
        receiverArmingEnding = millis() + RECEIVER_ARMING_TIME;
        
        if ( DEBUG ) Serial1.println("5\tARMING START");
      }
    }
  }
}

void position_loop() {
  long m = micros();
  
  if ( m >= nextControlTime ) {
    position_update( m );
    
    lastRunDuration = ( micros() - m );
    
    nextControlTime = m + CONTROL_FREQUENCY;
  }
}

void position_update( long m ) {
  accel_update();
  gyro_update();
  
  pos.update( m, accelValues, gyroValues );
}

void control_loop() {
  unsigned long m = micros();
  if ( m >= nextControlTime ) {
    control_loop_update( m );
    
    update_motors();
    
    lastRunDuration = ( micros() - m );
    
    nextControlTime = m + CONTROL_FREQUENCY;
  }
}

void control_loop_update( unsigned long m ) {
  double zeroTarget[] = { 0.0, 0.0, 0.0 };
  
  position_update( m );
  
  controller.control_update( m, zeroTarget );
  
  double receiver0 = receiver_get_value(0);
  double receiver1 = receiver_get_value(1);
  double receiver2 = receiver_get_value(2);
  
  int target = max( 0, map( -receiver2*10, 0, 1000, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE ) );
  
  motorAdjusts[0] = controller.motorAdjusts[0];
  motorAdjusts[1] = controller.motorAdjusts[1];
  motorAdjusts[2] = controller.motorAdjusts[2];
  motorAdjusts[3] = controller.motorAdjusts[3];
  
  // allow error scaling by receiver channel 1
  if ( receiver1 < 0.0 ) {
    motorAdjusts[0] += ( motorAdjusts[0] * -receiver1 / 10.0 );
    motorAdjusts[1] += ( motorAdjusts[1] * -receiver1 / 10.0 );
    motorAdjusts[2] += ( motorAdjusts[2] * -receiver1 / 10.0 );
    motorAdjusts[3] += ( motorAdjusts[3] * -receiver1 / 10.0 );
  } else if ( receiver1 > 0.0 ) {
    motorAdjusts[0] -= ( motorAdjusts[0] * ( receiver1 / 100.0 ) );
    motorAdjusts[1] -= ( motorAdjusts[1] * ( receiver1 / 100.0 ) );
    motorAdjusts[2] -= ( motorAdjusts[2] * ( receiver1 / 100.0 ) );
    motorAdjusts[3] -= ( motorAdjusts[3] * ( receiver1 / 100.0 ) );
  }
  
  motorAdjusts[0] = max( MIN_MOTOR_ADJUST, min( MAX_MOTOR_ADJUST, motorAdjusts[0] ) );
  motorAdjusts[1] = max( MIN_MOTOR_ADJUST, min( MAX_MOTOR_ADJUST, motorAdjusts[1] ) );
  motorAdjusts[2] = max( MIN_MOTOR_ADJUST, min( MAX_MOTOR_ADJUST, motorAdjusts[2] ) );
  motorAdjusts[3] = max( MIN_MOTOR_ADJUST, min( MAX_MOTOR_ADJUST, motorAdjusts[3] ) );
  
  motorValues[0] = max( min( target + motorAdjusts[0], MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
  motorValues[1] = max( min( target + motorAdjusts[1], MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
  motorValues[2] = max( min( target + motorAdjusts[2], MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
  motorValues[3] = max( min( target + motorAdjusts[3], MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
}

void arm_motors() {
  motor0.attach( MOTOR_PIN0 );
  motor1.attach( MOTOR_PIN1 );
  motor2.attach( MOTOR_PIN2 );
  motor3.attach( MOTOR_PIN3 );
  
  digitalWrite( ARMED_LED_PIN, HIGH );
  zero_motors();
}

void disarm_motors() {
  digitalWrite( ARMED_LED_PIN, LOW );
  zero_motors();
  
  delay( 1000 );

  motor0.detach( );
  motor1.detach( );
  motor2.detach( );
  motor3.detach( );
}


void update_motors() {
  motor0.writeMicroseconds( motorValues[0] );
  motor1.writeMicroseconds( motorValues[1] );
  motor2.writeMicroseconds( motorValues[2] );
  motor3.writeMicroseconds( motorValues[3] );
  
  led_blink( motorValues );
}

void zero_motors() {
  motorValues[0] = 0.0;
  motorValues[1] = 0.0;
  motorValues[2] = 0.0;
  motorValues[3] = 0.0;
  
  update_motors();
}

void log_data() {
  if ( millis() < nextLogTime ) return;
  
  Serial1.print('6'); // arduino data log event
  Serial1.print('\t');
  
  Serial1.print(millis());
  Serial1.print('\t');
  
  Serial1.print(lastRunDuration);
  Serial1.print('\t');
  
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
  Serial1.print(accelValues[X]);
  Serial1.print('\t');
  Serial1.print(accelValues[Y]);
  Serial1.print('\t');
  Serial1.print(accelValues[Z]);
  
  Serial1.print('\t');
  Serial1.print(gyroValues[X]);
  Serial1.print('\t');
  Serial1.print(gyroValues[Y]);
  Serial1.print('\t');
  Serial1.print(gyroValues[Z]);
  
  Serial1.print('\t');
  Serial1.print(motorAdjusts[0]);
  Serial1.print('\t');
  Serial1.print(motorAdjusts[1]);
  Serial1.print('\t');
  Serial1.print(motorAdjusts[2]);
  Serial1.print('\t');
  Serial1.print(motorAdjusts[3]);
  
  Serial1.print('\t');
  Serial1.print(pos.x);
  Serial1.print('\t');
  Serial1.print(pos.y);
  
  Serial1.print('\t');
  Serial1.print(controller.pidAbsoluteA.error);
  Serial1.print('\t');
  Serial1.print(controller.pidAbsoluteB.error);
  
  Serial1.println();
  
  nextLogTime = millis() + LOG_FREQUENCY;
}

