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

const float ACCEL_FILTER_ALPHA = 0.5;

int accelZeroCount;
int accelZeroXValues[ACCEL_ZERO_ITERS];
int accelZeroYValues[ACCEL_ZERO_ITERS];
int accelZeroZValues[ACCEL_ZERO_ITERS];
boolean accelZerod;

int accelZeroX;
int accelZeroY;
int accelZeroZ;

long accelNextUpdate;

void accel_reset() {
  accelZeroCount = 0;
  accelZerod = false;
  
  accelValues[0] = 0;
  accelValues[1] = 0;
  accelValues[2] = 0;  
  
  accelNextUpdate = millis();
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
      
      accelValues[0] = gx;
      accelValues[1] = gy;
      accelValues[2] = gz;
      
      if ( accelZeroCount >= ACCEL_ZERO_ITERS ) {
        int zeroXTotal = 0;
        int zeroYTotal = 0;
        int zeroZTotal = 0;
        
        for ( int i = 0; i < ACCEL_ZERO_ITERS; i++ ) {
          zeroXTotal += accelZeroXValues[i];
          zeroYTotal += accelZeroYValues[i];
          zeroZTotal += accelZeroZValues[i];
        }
        
        accelZeroX = zeroXTotal / accelZeroCount;
        accelZeroY = zeroYTotal / accelZeroCount;
        accelZeroZ = zeroZTotal / accelZeroCount;
        accelZerod = true;
        
        //Serial.print("Z");
        //printXYZ( gx, gy, gz, zeroX, zeroY, zeroZ );
        
        accelValues[0] = accelZeroX;
        accelValues[1] = accelZeroX;
        accelValues[2] = accelZeroZ;
      }
      
      accelNextUpdate = millis() + ACCEL_ZERO_DELAY;
    }
  } else {
    gx -= accelZeroX;
    gy -= accelZeroY;
    gz -= accelZeroZ;
    
    accelValues[0] = gx + ACCEL_FILTER_ALPHA * ( accelValues[0] - gx );
    accelValues[1] = gy + ACCEL_FILTER_ALPHA * ( accelValues[1] - gy );
    accelValues[2] = gz + ACCEL_FILTER_ALPHA * ( accelValues[2] - gz );
  }
}

boolean accel_ready( ) {
  return accelZerod;
}

