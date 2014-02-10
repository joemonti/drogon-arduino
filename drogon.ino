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

const int RESET_TIME = 5000;

const int ZERO_ITERS = 1000;
const int ZERO_DELAY = 10; // approx 10 seconds
int zeroIterCount;
long nextZeroUpdate;

const long IDLE_DELAY = 2000;


const int MIN_MOTOR_VALUE = 1000;
const int MAX_MOTOR_VALUE = 2000; //2000;
const float MAX_MOTOR_ADJUST = ( MAX_MOTOR_VALUE - MIN_MOTOR_VALUE ) * 0.25;
const float MIN_MOTOR_ADJUST = -MAX_MOTOR_ADJUST;
const float MOTOR_DIFF_SCALE = 0.1;

const int STATE_RESET = 0;
const int STATE_ZEROING = 1;
const int STATE_READY = 2;
const int STATE_ARMED = 3;

int state;

Servo motor0;
Servo motor1;
Servo motor2;
Servo motor3;

double accelValues[3];
double gyroValues[3];
int motorValues[4];
double motorAdjusts[4];

double motorMaster;
double motorRotate[3];

const int SERIAL_READ_BUFFER_SIZE = 512;
char* serialReadBuffer;
int serialReadBufferIndex = 0;

const long LOG_FREQUENCY = 500;
long nextLogTime;

const int CONTROL_ENGAGE_THRESHOLD = MIN_MOTOR_VALUE + (int) ( ( MAX_MOTOR_VALUE - MIN_MOTOR_VALUE ) * 0.1 );
boolean controlEngaged;

DrogonPosition pos;
DrogonController controller(&pos);
const long CONTROL_FREQUENCY = 5000; // 5ms
unsigned long nextControlTime;
unsigned long lastRunDuration;

void setup() {
  Serial.begin(9600);
  
  if (DEBUG) Serial.println("5\tSETUP STARTED");
  
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
  
  serialReadBuffer = (char*) malloc( SERIAL_READ_BUFFER_SIZE );
  
  motorMaster = 0.0;
  motorRotate[0] = 0.0;
  motorRotate[1] = 0.0;
  motorRotate[2] = 0.0;
  
  //receiver_setup();
  
  led_setup( MOTOR_LED_PIN0,
             MOTOR_LED_PIN1,
             MOTOR_LED_PIN2,
             MOTOR_LED_PIN3 );
  
  accel_setup();
  gyro_setup();
  
  state = STATE_RESET;
  
  lastRunDuration = 0;
  
  //receiverArmingState = RECEIVER_ARMING_IDLE;
  //receiverArmingEnding = millis();
  //receiverArmingCooldown = millis();
  
  nextLogTime = millis();
  nextControlTime = micros();
  controlEngaged = false;
  
  if (DEBUG) Serial.println("5\tSETUP FINISHED");
  if (DEBUG) Serial.println("5\tSTATE : RESET");
}

void loop() {
  read_serial();
  
  log_data();
  
  if ( state == STATE_RESET ) {
    accel_reset();
    gyro_reset();
    
    nextZeroUpdate = millis() + RESET_TIME;
    
    state = STATE_ZEROING;
    
    if ( DEBUG ) Serial.println("5\tSTATE : ZEROING");
  } else if ( state == STATE_ZEROING ) {
    if ( millis() >= nextZeroUpdate ) {
      accel_zero_accum();
      gyro_zero_accum();
      
      zeroIterCount++;
      
      if ( zeroIterCount >= ZERO_ITERS ) {
        accel_zero();
        gyro_zero();
        
        state = STATE_READY;
        digitalWrite( READY_LED_PIN, HIGH );
        
        if ( DEBUG ) Serial.println("5\tSTATE : READY");
      } else {
        nextZeroUpdate = millis() + ZERO_DELAY;
      }
    }
  } else if ( state == STATE_READY ) {
    position_loop();
  } else if ( state == STATE_ARMED ) {
    control_loop();
  }
}

void read_serial() {
  while ( Serial.available() ) {
    if ( serialReadBufferIndex >= SERIAL_READ_BUFFER_SIZE ) {
      Serial.println("5\tOVERFLOWED SERIAL BUFFER!!");
      serialReadBuffer[SERIAL_READ_BUFFER_SIZE-1] = '\0';
      parse_serial_command();
      serialReadBufferIndex = 0;
      return;
    } else {
      serialReadBuffer[serialReadBufferIndex] = Serial.read();
      if ( serialReadBuffer[serialReadBufferIndex] == '\n' ) {
        serialReadBuffer[serialReadBufferIndex] = '\0';
        parse_serial_command();
        serialReadBufferIndex = 0;
        return;
      }
      serialReadBufferIndex++;
    }
  }
}

void parse_serial_command() {
  int i;
  switch( serialReadBuffer[0] ) {
    case 'A':
      if ( state == STATE_READY ) {
        arm_motors();
        state = STATE_ARMED;
        if ( DEBUG ) Serial.println("5\tSTATE : ARMED");
      }
      break;
    case 'D':
      if ( state == STATE_ARMED ) {
        disarm_motors();
        state = STATE_READY;
        if ( DEBUG ) Serial.println("5\tSTATE : READY");
      }
      break;
    case 'M':
      i = 1;
      while ( serialReadBuffer[i] < '0' && serialReadBuffer[i] > '9' ) {
        if ( serialReadBuffer == '\0' ) {
          if ( DEBUG ) Serial.println("5\tMOTOR COMMAND NOT VALID");
          return;
        }
        i++;
        if ( i >= SERIAL_READ_BUFFER_SIZE ) {
          if ( DEBUG ) Serial.println("5\tMOTOR COMMAND NOT VALID");
          return;
        }
      }
      motorMaster = atof(&serialReadBuffer[i]);
      if ( DEBUG ) {
        Serial.print("5\tMOTOR SET TO ");
        Serial.print(motorMaster);
        Serial.println();
      }
      break;
    default:
      Serial.print("5\tINVALID COMMAND: ");
      Serial.print(serialReadBuffer[0]);
      Serial.println();
      break;
  }
}

void position_loop() {
  unsigned long m = micros();
  
  if ( m >= nextControlTime ) {
    position_update( m );
    
    lastRunDuration = ( micros() - m );
    
    nextControlTime = m + CONTROL_FREQUENCY;
  }
}

void position_update( unsigned long m ) {
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
  position_update( m );
  
  int target = (int) map_double( motorMaster, 0.0, 100.0, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE );
  
  target = constrain( target, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE );
  
  if ( controlEngaged ) {
    if ( target < CONTROL_ENGAGE_THRESHOLD ) {
      controller.reset( m );
      
      motorAdjusts[0] = 0;
      motorAdjusts[1] = 0;
      motorAdjusts[2] = 0;
      motorAdjusts[3] = 0;
      
      controlEngaged = false;
    } else {
      controller.control_update( m, motorRotate );
      
      motorAdjusts[0] = controller.motorAdjusts[0];
      motorAdjusts[1] = controller.motorAdjusts[1];
      motorAdjusts[2] = controller.motorAdjusts[2];
      motorAdjusts[3] = controller.motorAdjusts[3];
    }
  } else if ( target >= CONTROL_ENGAGE_THRESHOLD ) {
    controller.reset( m );
    
    controller.control_update( m, motorRotate );
    
    motorAdjusts[0] = controller.motorAdjusts[0];
    motorAdjusts[1] = controller.motorAdjusts[1];
    motorAdjusts[2] = controller.motorAdjusts[2];
    motorAdjusts[3] = controller.motorAdjusts[3];
    
    controlEngaged = true;
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
  motorMaster = 0.0;
  motorRotate[0] = 0.0;
  motorRotate[1] = 0.0;
  motorRotate[2] = 0.0;
  
  motorValues[0] = 0.0;
  motorValues[1] = 0.0;
  motorValues[2] = 0.0;
  motorValues[3] = 0.0;
  
  update_motors();
}

double map_double(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void log_data() {
  if ( millis() < nextLogTime ) return;
  
  Serial.print('6'); // arduino data log event
  Serial.print('\t');
  
  Serial.print(millis());
  Serial.print('\t');
  
  Serial.print(lastRunDuration);
  Serial.print('\t');
  
  /*
  Serial.print(receiver_get_state());
  Serial.print('\t');
  
  Serial.print(receiver_get_value(0));
  Serial.print('\t');
  Serial.print(receiver_get_value(1));
  Serial.print('\t');
  Serial.print(receiver_get_value(2));
  Serial.print('\t');
  Serial.print(receiver_get_value(3));
  Serial.print('\t');
  Serial.print(receiver_get_value(4));
  Serial.print('\t');
  Serial.print(receiver_get_value(5));
  */
  
  Serial.print('\t');
  Serial.print(accelValues[X]);
  Serial.print('\t');
  Serial.print(accelValues[Y]);
  Serial.print('\t');
  Serial.print(accelValues[Z]);
  
  Serial.print('\t');
  Serial.print(gyroValues[X]);
  Serial.print('\t');
  Serial.print(gyroValues[Y]);
  Serial.print('\t');
  Serial.print(gyroValues[Z]);
  
  Serial.print('\t');
  Serial.print(motorAdjusts[0]);
  Serial.print('\t');
  Serial.print(motorAdjusts[1]);
  Serial.print('\t');
  Serial.print(motorAdjusts[2]);
  Serial.print('\t');
  Serial.print(motorAdjusts[3]);
  
  Serial.print('\t');
  Serial.print(pos.x);
  Serial.print('\t');
  Serial.print(pos.y);
  
  Serial.print('\t');
  Serial.print(controller.pidAbsoluteA.error);
  Serial.print('\t');
  Serial.print(controller.pidAbsoluteB.error);
  
  Serial.println();
  
  nextLogTime = millis() + LOG_FREQUENCY;
}

