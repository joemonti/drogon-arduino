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

const long CONTROL_LOOP_FREQUENCY = 5;
long nextControlLoop;

void control_setup() {
  nextControlLoop = millis();
  
  zero_motor_values();
}

void zero_motor_values() {
  motorValues[0] = MIN_MOTOR_VALUE;
  motorValues[1] = MIN_MOTOR_VALUE;
  motorValues[2] = MIN_MOTOR_VALUE;
  motorValues[3] = MIN_MOTOR_VALUE;
}

boolean control_update() {
  if ( millis() > nextControlLoop ) {
	accel_update();
	gyro_update();
    update_motor_values();
    nextControlLoop = millis() + CONTROL_LOOP_FREQUENCY;
    return true;
  }
  return false;
}

void update_motor_values() {
  int receiver0 = receiver_get_value(0);
  int receiver1 = receiver_get_value(1);
  int receiver2 = receiver_get_value(2);
  
  int target = max( 0, map( -receiver2, 0, 100, MIN_MOTOR_VALUE, MAX_MOTOR_VALUE ) );
  
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
  
  motorValues[0] = max( min( target + motorValue0Adj, MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
  motorValues[1] = max( min( target + motorValue1Adj, MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
  motorValues[2] = max( min( target + motorValue2Adj, MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
  motorValues[3] = max( min( target + motorValue3Adj, MAX_MOTOR_VALUE ), MIN_MOTOR_VALUE );
}
