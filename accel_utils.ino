/*
 * Drogon : Acelleromter Utilities
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

const int ANALOG_RESOLUTION = 12;

const int ACCEL_ZERO_ITERS = 1000;
const int ACCEL_ZERO_DELAY = 10; // approx 10 seconds

const float ACCEL_FILTER_ALPHA = 0.8;

const double ACCEL_ZERO_BUFFER = 2.5;

int accelZeroCount;
long accelZeroXTotal;
long accelZeroYTotal;
long accelZeroZTotal;
boolean accelZeroed;

double accelZeroX;
double accelZeroY;
double accelZeroZ;

void accel_reset() {
  accelZeroed = false;
  
  accelZeroXTotal = 0;
  accelZeroYTotal = 0;
  accelZeroZTotal = 0;
  accelZeroCount = 0;
  
  accelValues[X] = 0;
  accelValues[Y] = 0;
  accelValues[Z] = 0;  
}

void accel_setup() {
  analogReadResolution( ANALOG_RESOLUTION );
  
  accel_reset();
}

void accel_zero_accum() {
  int gx = analogRead( ACCEL_PIN_X );
  int gy = analogRead( ACCEL_PIN_Y );
  int gz = analogRead( ACCEL_PIN_Z );
  
  accelZeroXTotal += gx;
  accelZeroYTotal += gy;
  accelZeroZTotal += gz;
  accelZeroCount += 1;
  
  accelValues[X] = gx;
  accelValues[Y] = gy;
  accelValues[Z] = gz;
}

void accel_zero() {
  accelZeroX = accelZeroXTotal / (double) accelZeroCount;
  accelZeroY = accelZeroYTotal / (double) accelZeroCount;
  accelZeroZ = accelZeroZTotal / (double) accelZeroCount;
  accelZeroed = true;
  
  accelValues[X] = 0.0;
  accelValues[Y] = 0.0;
  accelValues[Z] = 0.0;
}

void accel_update() {
  int gx = analogRead( ACCEL_PIN_X );
  int gy = analogRead( ACCEL_PIN_Y );
  int gz = analogRead( ACCEL_PIN_Z );

  if ( accelZeroed ) {
    double ax = ( gx - accelZeroX );
    double ay = ( gy - accelZeroY );
    double az = ( gz - accelZeroZ );
    
    if ( abs( ax ) <= ACCEL_ZERO_BUFFER ) ax = 0.0;
    if ( abs( ay ) <= ACCEL_ZERO_BUFFER ) ay = 0.0;
    if ( abs( az ) <= ACCEL_ZERO_BUFFER ) az = 0.0;
    
    accelValues[X] = ax; // + ACCEL_FILTER_ALPHA * ( accelValues[X] - gx );
    accelValues[Y] = ay; // + ACCEL_FILTER_ALPHA * ( accelValues[Y] - gy );
    accelValues[Z] = az; // + ACCEL_FILTER_ALPHA * ( accelValues[Z] - gz );
  }
}

