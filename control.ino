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

double featuresA[NUM_FEATURES];
double featuresB[NUM_FEATURES];

const int MAX_ERR_TOTAL = 500;
const int MIN_ERR_TOTAL = -MAX_ERR_TOTAL;

const double MAX_MOTOR_ADJUST = MOTOR_VALUE_RANGE*.25;
const double MIN_MOTOR_ADJUST = -MAX_MOTOR_ADJUST;

const double aScale = 1/6.2;

boolean controlStart = false;
long lastErrUpdate;
double errTotalA;
double errTotalB;

double errLastA;
double errLastB;

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
    featuresA[i] = 0.0;
    featuresB[i] = 0.0;
  }
  
  angleX = 0.0;
  angleY = 0.0;
  
  motorOffsetA = 0.0;
  motorOffsetB = 0.0;
  
  nextControlLoop = millis();
  
  lastErrUpdate = micros();
  
  errTotalA = 0.0;
  errTotalB = 0.0;
  
  errLastA = 0.0;
  errLastB = 0.0;
  
  errA = 0.0;
  errB = 0.0;
  
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
    update_angles();
    update_motor_values();
    nextControlLoop = millis() + CONTROL_LOOP_FREQUENCY;
    return true;
  }
  return false;
}

void update_angles() {
  double lastAccelX = accelValues[X];
  double lastAccelY = accelValues[Y];
  
  accel_update();
  gyro_update();
  
  angleX = (-lastAccelY*aScale) + gyroValues[X];
  angleY = (lastAccelX*aScale) + gyroValues[Y];
}

void update_motor_values() {
  double receiver0 = receiver_get_value(0);
  double receiver1 = receiver_get_value(1);
  double receiver2 = receiver_get_value(2);
  
  int target = max( 0, map( -receiver2*10, 0, 1000, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE ) );
  
  // adjust angles 
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
  
  map_angles_to_motor_offsets( angleX, angleY );

  calc_errs();
  
  // allow error scaling by receiver channel 1
  if ( receiver1 < 0.0 ) {
    errA += ( errA * -receiver1 / 10.0 );
    errB += ( errB * -receiver1 / 10.0 );
  } else if ( receiver1 > 0.0 ) {
    errA /= ( receiver1 / 10.0 );
    errB /= ( receiver1 / 10.0 );
  }
  
  motorAdjusts[1] += -errA;
  motorAdjusts[3] +=  errA;
  
  motorAdjusts[0] +=  errB;
  motorAdjusts[2] += -errB;
  
  motorAdjusts[0] = max( MIN_MOTOR_ADJUST, min( MAX_MOTOR_ADJUST, motorAdjusts[0] ) );
  motorAdjusts[1] = max( MIN_MOTOR_ADJUST, min( MAX_MOTOR_ADJUST, motorAdjusts[1] ) );
  motorAdjusts[2] = max( MIN_MOTOR_ADJUST, min( MAX_MOTOR_ADJUST, motorAdjusts[2] ) );
  motorAdjusts[3] = max( MIN_MOTOR_ADJUST, min( MAX_MOTOR_ADJUST, motorAdjusts[3] ) );
  
  motorValues[0] = max( min( target + motorAdjusts[0], MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
  motorValues[1] = max( min( target + motorAdjusts[1], MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
  motorValues[2] = max( min( target + motorAdjusts[2], MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
  motorValues[3] = max( min( target + motorAdjusts[3], MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
}

/*
 * Calculate errors using PID algorithm from motor offsets (distance to center)
 */
void calc_errs() {
  double rawErrA = motorOffsetA;
  double rawErrB = motorOffsetB;
  
  errTotalA += rawErrA;
  errTotalB += rawErrB;
  
  // bound err total to min/max to prevent overrunning
  errTotalA = max( MIN_ERR_TOTAL, min( MAX_ERR_TOTAL, errTotalA ) );
  errTotalB = max( MIN_ERR_TOTAL, min( MAX_ERR_TOTAL, errTotalB ) );
  
  // calculate elapsed time since last error update in seconds
  long t = micros();
  double elapsed = ( t - lastErrUpdate ) / 1000000.0;
  
  // err diff is change in degrees in seconds from last update
  double errDiffA = ( rawErrA - errLastA ) / elapsed;
  double errDiffB = ( rawErrB - errLastB ) / elapsed;
  
  // setup features (PID values) for motor A
  featuresA[0] = rawErrA;     // P component, raw error offset
  featuresA[1] = errTotalA;   // I component, total error
  featuresA[2] = errDiffA;    // D component, rate of change of error
  
  // setup features (PID values) for motor B
  featuresB[0] = rawErrB;     // P component, raw error offset
  featuresB[1] = errTotalB;   // I component, total error
  featuresB[2] = errDiffB;    // D component, rate of change of error
  
  // compute PID function by multiplying feature vector with 
  // thetas vecor (PID constants)
  errA = array_mult( featuresA, THETAS, NUM_FEATURES );
  errA = array_mult( featuresB, THETAS, NUM_FEATURES );
  
  // store last error values
  errLastA = rawErrA;
  errLastB = rawErrB;
  
  // store time of this update
  lastErrUpdate = t;
}

/*
 * multiply arrays. like multiplying matrices if b was transposed.
 */
double array_mult( const double* a, const double* b, int len ) {
  double value = 0.0;
  for ( int i = 0; i < len; i++ ) {
    value += ( a[i] * b[i] );
  }
  return value;
}

/*
 * Map angle to motor offsets.
 */
void map_angles_to_motor_offsets( double angleX, double angleY ) {
  double rot1[3];
  double rot2[3];
  double rot3[3];
  
  double zA;
  double zB;
  
  // convert angle in degrees to radians for trig functions
  double angleXRadians = angleX * PI / 180.0;
  double angleYRadians = angleY * PI / 180.0;
  
  // compute sines/cosines for rotation matrices
  double sinAngleX = sin(angleXRadians);
  double sinAngleY = sin(angleYRadians);
  double cosAngleX = cos(angleXRadians);
  double cosAngleY = cos(angleYRadians);
  
  // X rotation matrix
  double motorXOffsetMatrix[] = {
    1, 0,          0,
    0, cosAngleX, -sinAngleX,
    0, sinAngleX,  cosAngleX
  };
  
  // Y rotation matrix
  double motorYOffsetMatrix[] = {
     cosAngleY, 0, sinAngleY,
     0,         1, 0,
    -sinAngleY, 0, cosAngleY
  };
  
  // rotate motor A around Z axis (each motor arm offset 45 degrees)
  rot_matrix_mult( motorAOffsetMatrix, motorOffsetVector, rot1 );
  
  // rotate around X axis
  rot_matrix_mult( motorXOffsetMatrix, rot1, rot2 );
  
  // rotate around Y axis
  rot_matrix_mult( motorYOffsetMatrix, rot2, rot3 );
  
  // extract Z (height) of motor A
  zA = rot3[Z];
  
  // rotate motor B around Z axis (each motor arm offset 45 degrees)
  rot_matrix_mult( motorBOffsetMatrix, motorOffsetVector, rot1 );
  
  // rotate around X axis
  rot_matrix_mult( motorXOffsetMatrix, rot1, rot2 );

  // rotate around Y axis
  rot_matrix_mult( motorYOffsetMatrix, rot2, rot3 );
  
  // extract Z (height) of motor B
  zB = rot3[Z];
  
  // Convert Z (height) to angle then to arc length to accurately
  // reprsent distance of motor to center.
  //  Variables:
  //    OPP: z (height) of arm (known from rotations)
  //    ADJ: length of arm (known from constant)
  //    ANG: angle of arm (unknown, computed)
  //    R: length of arm (known constant)
  //    ARCLEN: distance of motor to center (computed)
  //  Formulas:
  //    ANG = arcsin(OPP/ADJ))   <-- from sin(ANG) = OPP/ADJ
  //    ARCLEN = ANG * R
  motorOffsetA = asin(zA/ARM_LENGTH) * ARM_LENGTH;
  motorOffsetB = asin(zB/ARM_LENGTH) * ARM_LENGTH;
}

/*
 * Multiple rotation matrix by XYZ vector. Write to dst vector rotated XYZ.
 */
void rot_matrix_mult( const double* a, const double* b, double* dst ) {
  for ( int i = 0; i < 3; i++ ) {
    dst[i] = 0.0;
    for ( int j = 0; j < 3; j++ ) {
      dst[i] += ( a[i*3+j] * b[j] );
    }
  }
}

