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

const float GYRO_MAX = 0xffff * L3GD20_SENSITIVITY_250DPS;
const float GYRO_CENTER = GYRO_MAX/2;

const int GYRO_BUFFER = 3000;
const float FILTER_ALPHA = 0.1;

int gyroZeroCount;
double gyroZeroXTotal;
double gyroZeroYTotal;
double gyroZeroZTotal;
boolean gyroZeroed;

float gyroZeroX;
float gyroZeroY;
float gyroZeroZ;

void gyro_setup() {
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS)) {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }
  
  gyro_reset();
}

void gyro_reset() {
  gyroZeroed = false;
  
  gyroZeroXTotal = 0.0;
  gyroZeroYTotal = 0.0;
  gyroZeroZTotal = 0.0;
  gyroZeroCount = 0;
  
  gyroValues[X] = 0.0;
  gyroValues[Y] = 0.0;
  gyroValues[Z] = 0.0;
}

void gyro_zero_accum() {
  gyro.read();
  
  double gx = gyro.data.x;
  double gy = gyro.data.y;
  double gz = gyro.data.z;
  
  gyroZeroXTotal += ( gx > GYRO_CENTER ? gx - GYRO_MAX : gx );
  gyroZeroYTotal += ( gy > GYRO_CENTER ? gy - GYRO_MAX : gy );
  gyroZeroZTotal += ( gz > GYRO_CENTER ? gz - GYRO_MAX : gz );
  gyroZeroCount += 1;
  
  gyroValues[X] = gx;
  gyroValues[Y] = gy;
  gyroValues[Z] = gz;
}

void gyro_zero() {
  gyroZeroX = gyroZeroXTotal / gyroZeroCount;
  gyroZeroY = gyroZeroYTotal / gyroZeroCount;
  gyroZeroZ = gyroZeroZTotal / gyroZeroCount;
  gyroZeroed = true;
  
  gyroValues[X] = 0.0;
  gyroValues[Y] = 0.0;
  gyroValues[Z] = 0.0;
}

void gyro_update() {
  gyro.read();
  
  double gx = gyro.data.x;
  double gy = gyro.data.y;
  double gz = gyro.data.z;

  if ( gyroZeroed ) {
    gyroValues[X] = ( gx > GYRO_CENTER ? gx - GYRO_MAX : gx ) - gyroZeroX;
    gyroValues[Y] = ( gy > GYRO_CENTER ? gy - GYRO_MAX : gy ) - gyroZeroY;
    gyroValues[Z] = ( gz > GYRO_CENTER ? gz - GYRO_MAX : gz ) - gyroZeroZ;
  }
}

