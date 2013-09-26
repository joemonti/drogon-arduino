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

const float GYRO_MAX = 0xffff * L3GD20_SENSITIVITY_250DPS;
const float GYRO_CENTER = GYRO_MAX/2;

const int GYRO_BUFFER = 3000;
const float FILTER_ALPHA = 0.1;

int zeroCount;
float zeroXValues[ZERO_ITERS];
float zeroYValues[ZERO_ITERS];
float zeroZValues[ZERO_ITERS];
boolean zerod;

float zeroX;
float zeroY;
float zeroZ;

int gyroBufferIndex;
double gyroXBuffer[GYRO_BUFFER];
double gyroYBuffer[GYRO_BUFFER];
double gyroZBuffer[GYRO_BUFFER];

double gyroLastX;
double gyroLastY;
double gyroLastZ;

long nextUpdate;
long lastUpdate;

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
  
  gyroValues[X] = 0.0;
  gyroValues[Y] = 0.0;
  gyroValues[Z] = 0.0;
  
  for ( int i = 0; i < GYRO_BUFFER; i++ ) {
    gyroXBuffer[i] = 0.0;
    gyroYBuffer[i] = 0.0;
    gyroZBuffer[i] = 0.0;
  }
  gyroBufferIndex = 0;
  
  nextUpdate = millis() + IDLE_DELAY;
}

void gyro_update() {
  // put your main code here, to run repeatedly: 
  gyro.read();
  
  double gx = gyro.data.x;
  double gy = gyro.data.y;
  double gz = gyro.data.z;

  if ( !zerod ) {
    if ( millis() >= nextUpdate ) {
      zeroXValues[zeroCount] = gx;
      zeroYValues[zeroCount] = gy;
      zeroZValues[zeroCount] = gz;
      zeroCount += 1;
      
      gyroValues[X] = gx;
      gyroValues[Y] = gy;
      gyroValues[Z] = gz;
      
      if ( zeroCount >= ZERO_ITERS ) {
        float zeroXTotal = 0.0;
        float zeroYTotal = 0.0;
        float zeroZTotal = 0.0;
        
        for ( int i = 0; i < ZERO_ITERS; i++ ) {
          zeroXTotal += zeroXValues[i] > GYRO_CENTER ? zeroXValues[i] - GYRO_MAX : zeroXValues[i];
          zeroYTotal += zeroYValues[i] > GYRO_CENTER ? zeroYValues[i] - GYRO_MAX : zeroYValues[i];
          zeroZTotal += zeroZValues[i] > GYRO_CENTER ? zeroZValues[i] - GYRO_MAX : zeroZValues[i];
        }
        
        zeroX = zeroXTotal / zeroCount;
        zeroY = zeroYTotal / zeroCount;
        zeroZ = zeroZTotal / zeroCount;
        zerod = true;
        
        //Serial.print("Z");
        //printXYZ( gx, gy, gz, zeroX, zeroY, zeroZ );
        lastUpdate = micros();
        
        if ( gx > GYRO_CENTER ) gx -= GYRO_MAX;
        if ( gy > GYRO_CENTER ) gy -= GYRO_MAX;
        if ( gz > GYRO_CENTER ) gz -= GYRO_MAX;
        
        gx -= zeroX;
        gy -= zeroY;
        gz -= zeroZ;
        
        gyroLastX = gx;
        gyroLastY = gy;
        gyroLastZ = gz;
        gyroValues[X] = 0.0;
        gyroValues[Y] = 0.0;
        gyroValues[Z] = 0.0;
      }
      
      nextUpdate = millis() + ZERO_DELAY;
    }
  } else {
    if ( gx > GYRO_CENTER ) gx -= GYRO_MAX;
    if ( gy > GYRO_CENTER ) gy -= GYRO_MAX;
    if ( gz > GYRO_CENTER ) gz -= GYRO_MAX;
    
    gx -= zeroX;
    gy -= zeroY;
    gz -= zeroZ;
    
    /*
    gyroValues[X] = gx + FILTER_ALPHA * ( gyroValues[X] - gx );
    gyroValues[Y] = gy + FILTER_ALPHA * ( gyroValues[Y] - gy );
    gyroValues[Z] = gz + FILTER_ALPHA * ( gyroValues[Z] - gz );
    */
    
    /*
    long m = micros();
    double elapsedSeconds = ( m - lastUpdate ) / 1000000.0;
    gx *= elapsedSeconds;
    gy *= elapsedSeconds;
    gz *= elapsedSeconds;
    lastUpdate = m;
    */
    
    gyroValues[X] = gx; // + FILTER_ALPHA * ( gyroValues[X] - gx );
    gyroValues[Y] = gy; // + FILTER_ALPHA * ( gyroValues[Y] - gy );
    gyroValues[Z] = gz; // + FILTER_ALPHA * ( gyroValues[Z] - gz );
    
    /*
    double gxUpdate = ( gyroLastX + gx ) / 2.0;
    double gyUpdate = ( gyroLastY + gy ) / 2.0;
    double gzUpdate = ( gyroLastZ + gz ) / 2.0;
    
    gyroValues[0] += gxUpdate;
    gyroValues[1] += gyUpdate;
    gyroValues[2] += gzUpdate;
    
    gyroLastX = gx;
    gyroLastY = gy;
    gyroLastZ = gz;
    */
    
    /*
    gyroValues[0] -= gyroXBuffer[gyroBufferIndex];
    gyroValues[1] -= gyroYBuffer[gyroBufferIndex];
    gyroValues[2] -= gyroZBuffer[gyroBufferIndex];
    
    gyroXBuffer[gyroBufferIndex] = gx;
    gyroYBuffer[gyroBufferIndex] = gy;
    gyroZBuffer[gyroBufferIndex] = gz;
    
    gyroValues[0] += gyroXBuffer[gyroBufferIndex];
    gyroValues[1] += gyroYBuffer[gyroBufferIndex];
    gyroValues[2] += gyroZBuffer[gyroBufferIndex];
    
    gyroBufferIndex += 1;
    if ( gyroBufferIndex >= GYRO_BUFFER ) gyroBufferIndex = 0;
    */
  }
}

boolean gyro_ready( ) {
  return zerod;
}

