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
float accelZeroXValues[ACCEL_ZERO_ITERS];
float accelZeroYValues[ACCEL_ZERO_ITERS];
float accelZeroZValues[ACCEL_ZERO_ITERS];
boolean accelZerod;

float accelZeroX;
float accelZeroY;
float accelZeroZ;

float accelX;
float accelY;
float accelZ;

long accelNextUpdate;

void accel_setup() {
  analogReadResolution( ANALOG_RESOLUTION );
  
  accelZeroCount = 0;
  accelZerod = false;
  
  accelNextUpdate = millis();
}

void accel_loop() {
  float gx = analogRead( ACCEL_PIN_X );
  float gy = analogRead( ACCEL_PIN_Y );
  float gz = analogRead( ACCEL_PIN_Z );

  if ( !accelZerod ) {
    if ( millis() >= accelNextUpdate ) {
      accelZeroXValues[accelZeroCount] = gx;
      accelZeroYValues[accelZeroCount] = gy;
      accelZeroZValues[accelZeroCount] = gz;
      accelZeroCount += 1;
      
      accelX = gx;
      accelY = gy;
      accelZ = gz;
      
      if ( accelZeroCount >= ACCEL_ZERO_ITERS ) {
        float zeroXTotal = 0.0;
        float zeroYTotal = 0.0;
        float zeroZTotal = 0.0;
        
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
        
        accelX = accelZeroX;
        accelY = accelZeroX;
        accelZ = accelZeroZ;
      }
      
      accelNextUpdate = millis() + ACCEL_ZERO_DELAY;
    }
  } else {
    gx -= accelZeroX;
    gy -= accelZeroY;
    gz -= accelZeroZ;
    
    accelX = gx + ACCEL_FILTER_ALPHA * ( accelX - gx );
    accelY = gy + ACCEL_FILTER_ALPHA * ( accelY - gy );
    accelZ = gz + ACCEL_FILTER_ALPHA * ( accelZ - gz );
  }
}

boolean accel_ready( ) {
  return accelZerod;
}
