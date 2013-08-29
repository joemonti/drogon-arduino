/*
 * Drogon : Control
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

const long CONTROL_LOOP_FREQUENCY = 2;
long nextControlLoop;

const int NUM_FEATURES = 3;

const double THETAS[] = {
  5.0, 0.04, 0.4, /* KP KI KD */
};

double featuresX[NUM_FEATURES];
double featuresY[NUM_FEATURES];

const int MAX_ERR_TOTAL = 500;
const int MIN_ERR_TOTAL = -MAX_ERR_TOTAL;

const double MAX_MOTOR_ADJUST = MOTOR_VALUE_RANGE*.25;
const double MIN_MOTOR_ADJUST = -MAX_MOTOR_ADJUST;

const double aScale = 1/6.2;

boolean controlStart = false;
long lastErrUpdate;
double errTotalX;
double errTotalY;

double errLastX;
double errLastY;

const float ARM_LENGTH = 30; // 30cm, almost 1ft
const double ARM_ANGLE_A = 45.0 * PI / 180.0;
const double ARM_ANGLE_B = (90.0+45.0) * PI / 180.0;
const double motorAOffsetMatrix[] = {
    cos(ARM_ANGLE_A), -sin(ARM_ANGLE_A), 0,
    sin(ARM_ANGLE_A),  cos(ARM_ANGLE_A), 0,
    0,                 0,                1
  };
const double motorBOffsetMatrix[] = {
    cos(ARM_ANGLE_B), -sin(ARM_ANGLE_B), 0,
    sin(ARM_ANGLE_B),  cos(ARM_ANGLE_B), 0,
    0,                 0,                1
  };
const double motorOffsetVector[] = {
    0, ARM_LENGTH, 0
  };

void control_setup() {
  zero_motor_values();
  controlStart = false;
}

void zero_motor_values() {
  motorValues[0] = MIN_MOTOR_VALUE;
  motorValues[1] = MIN_MOTOR_VALUE;
  motorValues[2] = MIN_MOTOR_VALUE;
  motorValues[3] = MIN_MOTOR_VALUE;
  
  for ( int i = 0; i < NUM_FEATURES; i++ ) {
    featuresX[i] = 0.0;
    featuresY[i] = 0.0;
  }
  
  nextControlLoop = millis();
  
  lastErrUpdate = micros();
  
  errTotalX = 0.0;
  errTotalY = 0.0;
  
  errLastX = 0.0;
  errLastY = 0.0;
  
  angleX = 0.0;
  angleY = 0.0;
  
  errX = 0.0;
  errY = 0.0;
  
  motorAdjusts[0] = 0.0;
  motorAdjusts[1] = 0.0;
  motorAdjusts[2] = 0.0;
  motorAdjusts[3] = 0.0;
}

boolean control_update() {
  if ( !controlStart ) {
    zero_motor_values();
    controlStart = true;
  } else if ( millis() > nextControlLoop ) {
    double lastAccelX = accelValues[X];
    double lastAccelY = accelValues[Y];
    
    accel_update();
    gyro_update();
    
    angleX = (-lastAccelY*aScale) + gyroValues[X];
    angleY = (lastAccelX*aScale) + gyroValues[Y];
    
    calc_errs();
    update_motor_values();
    nextControlLoop = millis() + CONTROL_LOOP_FREQUENCY;
    return true;
  }
  return false;
}

void update_motor_values() {
  double receiver0 = receiver_get_value(0);
  double receiver1 = receiver_get_value(1);
  double receiver2 = receiver_get_value(2);
  
  int target = max( 0, map( -receiver2*10, 0, 1000, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE ) );
  
  /*
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
  */
  
  if ( receiver1 < 0.0 ) {
    errX += ( errorX * -receiver1 / 10.0 );
    errY += ( errorY * -receiver1 / 10.0 );
  } else if ( receiver1 > 0.0 ) {
    errX /= ( receiver1 / 10.0 );
    errY /= ( receiver1 / 10.0 );
  }
  
  update_motor_adjusts( errX, errY );
  
  motorValues[0] = max( min( target + motorAdjusts[0], MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
  motorValues[1] = max( min( target + motorAdjusts[1], MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
  motorValues[2] = max( min( target + motorAdjusts[2], MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
  motorValues[3] = max( min( target + motorAdjusts[3], MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
}

void calc_errs() {
  double rawErrX = angleX;
  double rawErrY = angleY;
  
  errTotalX += rawErrX;
  errTotalY += rawErrY;
  
  errTotalX = max( MIN_ERR_TOTAL, min( MAX_ERR_TOTAL, errTotalX ) );
  errTotalY = max( MIN_ERR_TOTAL, min( MAX_ERR_TOTAL, errTotalY ) );
  
  long t = micros();
  double elapsed = ( t - lastErrUpdate ) / 1000000.0;
  
  double errDiffX = ( rawErrX - errLastX ) / elapsed;
  double errDiffY = ( rawErrY - errLastY ) / elapsed;
  
  featuresX[0] = rawErrX;
  featuresX[1] = errTotalX;
  featuresX[2] = errDiffX;
  
  featuresY[0] = rawErrY;
  featuresY[1] = errTotalY;
  featuresY[2] = errDiffY;
  
  errX = array_mult( featuresX, THETAS, NUM_FEATURES );  
  errY = array_mult( featuresY, THETAS, NUM_FEATURES );
  
  errLastX = rawErrX;
  errLastY = rawErrY;
  
  lastErrUpdate = t;
}

double array_mult( const double* a, const double* b, int len ) {
  double value = 0.0;
  for ( int i = 0; i < len; i++ ) {
    value += ( a[i] * b[i] );
  }
  return value;
}

void update_motor_adjusts( double angleX, double angleY ) {
  double rot1[3];
  double rot2[3];
  double rot3[3];
  
  double angleXRadians = angleX * PI / 180.0;
  double angleYRadians = angleY * PI / 180.0;
  
  double sinAngleX = sin(angleXRadians);
  double sinAngleY = sin(angleYRadians);
  double cosAngleX = cos(angleXRadians);
  double cosAngleY = cos(angleYRadians);
  
  double motorXOffsetMatrix[] = {
    1, 0,          0,
    0, cosAngleX, -sinAngleX,
    0, sinAngleX,  cosAngleX
  };
  
  double motorYOffsetMatrix[] = {
     cosAngleY, 0, sinAngleY,
     0,         1, 0,
    -sinAngleY, 0, cosAngleY
  };
  
  rot_matrix_mult( motorAOffsetMatrix, motorOffsetVector, rot1 );
  rot_matrix_mult( motorXOffsetMatrix, rot1, rot2 );
  rot_matrix_mult( motorYOffsetMatrix, rot2, rot3 );
  
  motorAdjusts[1] = -rot3[Z];
  motorAdjusts[3] =  rot3[Z];
  
  rot_matrix_mult( motorBOffsetMatrix, motorOffsetVector, rot1 );
  rot_matrix_mult( motorXOffsetMatrix, rot1, rot2 );
  rot_matrix_mult( motorYOffsetMatrix, rot2, rot3 );
  
  motorAdjusts[0] =  rot3[Z];
  motorAdjusts[2] = -rot3[Z];
  
  motorAdjusts[0] = max( MIN_MOTOR_ADJUST, min( MAX_MOTOR_ADJUST, motorAdjusts[0] ) );
  motorAdjusts[1] = max( MIN_MOTOR_ADJUST, min( MAX_MOTOR_ADJUST, motorAdjusts[1] ) );
  motorAdjusts[2] = max( MIN_MOTOR_ADJUST, min( MAX_MOTOR_ADJUST, motorAdjusts[2] ) );
  motorAdjusts[3] = max( MIN_MOTOR_ADJUST, min( MAX_MOTOR_ADJUST, motorAdjusts[3] ) );
}

void rot_matrix_mult( const double* a, const double* b, double* dst ) {
  for ( int i = 0; i < 3; i++ ) {
    dst[i] = 0.0;
    for ( int j = 0; j < 3; j++ ) {
      dst[i] += ( a[i*3+j] * b[j] );
    }
  }
}

