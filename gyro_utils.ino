/*
 * Drogon : Gyro Utilities
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

#include <Wire.h> 
#include <Adafruit_L3GD20.h>

Adafruit_L3GD20 gyro;

const int ZERO_ITERS = 500;
const int ZERO_DELAY = 10; // approx 5 seconds

const float CENTER = 180;

const float MAX_DEFAULT = 573.43;

const float FILTER_ALPHA = 0.5;

int zeroCount;
float zeroXValues[ZERO_ITERS];
float zeroYValues[ZERO_ITERS];
float zeroZValues[ZERO_ITERS];
float maxX;
float maxY;
float maxZ;
boolean zerod;

float zeroX;
float zeroY;
float zeroZ;

long nextUpdate;

void gyro_setup() {
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS)) {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }
  
  gyro_reset();
}

void gyro_reset() {
  zeroCount = 0;
  zerod = false;
  
  maxX = -1;
  maxY = -1;
  maxZ = -1;
  
  gyroValues[0] = 0;
  gyroValues[1] = 0;
  gyroValues[2] = 0;
  
  nextUpdate = millis();
}

void gyro_update() {
  // put your main code here, to run repeatedly: 
  gyro.read();
  
  float gx = gyro.data.x;
  float gy = gyro.data.y;
  float gz = gyro.data.z;

  if ( !zerod ) {
    if ( millis() >= nextUpdate ) {
      if ( gx > CENTER && gx > maxX ) maxX = gx;
      if ( gy > CENTER && gy > maxY ) maxY = gy;
      if ( gz > CENTER && gz > maxZ ) maxZ = gz;
      
      zeroXValues[zeroCount] = gx;
      zeroYValues[zeroCount] = gy;
      zeroZValues[zeroCount] = gz;
      zeroCount += 1;
      
      gyroValues[0] = gx;
      gyroValues[1] = gy;
      gyroValues[2] = gz;
      
      if ( zeroCount >= ZERO_ITERS ) {
        float zeroXTotal = 0.0;
        float zeroYTotal = 0.0;
        float zeroZTotal = 0.0;
        
        if ( maxX < 0 ) maxX = MAX_DEFAULT;
        if ( maxY < 0 ) maxY = MAX_DEFAULT;
        if ( maxZ < 0 ) maxZ = MAX_DEFAULT;
        
        for ( int i = 0; i < ZERO_ITERS; i++ ) {
          zeroXTotal += zeroXValues[i] > CENTER ? zeroXValues[i] - maxX : zeroXValues[i];
          zeroYTotal += zeroYValues[i] > CENTER ? zeroYValues[i] - maxY : zeroYValues[i];
          zeroZTotal += zeroZValues[i] > CENTER ? zeroZValues[i] - maxZ : zeroZValues[i];
        }
        
        zeroX = zeroXTotal / zeroCount;
        zeroY = zeroYTotal / zeroCount;
        zeroZ = zeroZTotal / zeroCount;
        zerod = true;
        
        //Serial.print("Z");
        //printXYZ( gx, gy, gz, zeroX, zeroY, zeroZ );
        
        gyroValues[0] = zeroX;
        gyroValues[1] = zeroX;
        gyroValues[2] = zeroZ;
      }
      
      nextUpdate = millis() + ZERO_DELAY;
    }
  } else {
    gx = gx > CENTER ? gx - maxX : gx;
    gy = gy > CENTER ? gy - maxY : gy;
    gz = gz > CENTER ? gz - maxZ : gz;
    
    gx -= zeroX;
    gy -= zeroY;
    gz -= zeroZ;
    
    gyroValues[0] = gx + FILTER_ALPHA * ( gyroValues[0] - gx );
    gyroValues[1] = gy + FILTER_ALPHA * ( gyroValues[1] - gy );
    gyroValues[2] = gz + FILTER_ALPHA * ( gyroValues[2] - gz );
  }
}

boolean gyro_ready( ) {
  return zerod;
}

