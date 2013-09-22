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

const int ACCEL_ZERO_ITERS = 500;
const int ACCEL_ZERO_DELAY = 10; // approx 5 seconds

const float ACCEL_FILTER_ALPHA = 0.8;

const double ACCEL_ZERO_BUFFER = 1.2;

int accelZeroCount;
int accelZeroXValues[ACCEL_ZERO_ITERS];
int accelZeroYValues[ACCEL_ZERO_ITERS];
int accelZeroZValues[ACCEL_ZERO_ITERS];
boolean accelZerod;

double accelZeroX;
double accelZeroY;
double accelZeroZ;

long accelNextUpdate;

void accel_reset() {
  accelZeroCount = 0;
  accelZerod = false;
  
  accelValues[X] = 0;
  accelValues[Y] = 0;
  accelValues[Z] = 0;  
  
  accelNextUpdate = millis() + IDLE_DELAY;
}

void accel_setup() {
  analogReadResolution( ANALOG_RESOLUTION );
  
  accel_reset();
}

void accel_update() {
  int gx = analogRead( ACCEL_PIN_X );
  int gy = analogRead( ACCEL_PIN_Y );
  int gz = analogRead( ACCEL_PIN_Z );

  if ( !accelZerod ) {
    if ( millis() >= accelNextUpdate ) {
      accelZeroXValues[accelZeroCount] = gx;
      accelZeroYValues[accelZeroCount] = gy;
      accelZeroZValues[accelZeroCount] = gz;
      accelZeroCount += 1;
      
      accelValues[X] = gx;
      accelValues[Y] = gy;
      accelValues[Z] = gz;
      
      if ( accelZeroCount >= ACCEL_ZERO_ITERS ) {
        int zeroXTotal = 0;
        int zeroYTotal = 0;
        int zeroZTotal = 0;
        
        for ( int i = 0; i < ACCEL_ZERO_ITERS; i++ ) {
          zeroXTotal += accelZeroXValues[i];
          zeroYTotal += accelZeroYValues[i];
          zeroZTotal += accelZeroZValues[i];
        }
        
        accelZeroX = zeroXTotal / (double) accelZeroCount;
        accelZeroY = zeroYTotal / (double) accelZeroCount;
        accelZeroZ = zeroZTotal / (double) accelZeroCount;
        accelZerod = true;
        
        //Serial.print("Z");
        //printXYZ( gx, gy, gz, zeroX, zeroY, zeroZ );
        
        accelValues[X] = accelZeroX;
        accelValues[Y] = accelZeroX;
        accelValues[Z] = accelZeroZ;
      }
      
      accelNextUpdate = millis() + ACCEL_ZERO_DELAY;
    }
  } else {
    double ax = ( gx - accelZeroX );
    double ay = ( gy - accelZeroY );
    double az = ( gz - accelZeroZ );
    
    if ( abs( ax ) < ACCEL_ZERO_BUFFER ) ax = 0.0;
    if ( abs( ay ) < ACCEL_ZERO_BUFFER ) ay = 0.0;
    if ( abs( az ) < ACCEL_ZERO_BUFFER ) az = 0.0;
    
    accelValues[X] = ax; // + ACCEL_FILTER_ALPHA * ( accelValues[X] - gx );
    accelValues[Y] = ay; // + ACCEL_FILTER_ALPHA * ( accelValues[Y] - gy );
    accelValues[Z] = az; // + ACCEL_FILTER_ALPHA * ( accelValues[Z] - gz );
  }
}

boolean accel_ready( ) {
  return accelZerod;
}

