/*
 * Author: Rafael Rodrigues da Silva
 * Date: May 13, 2023
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "ir_sensor.h"

IRSensor IRSensor::front_left(IR_LEFT_a0);
IRSensor IRSensor::front_right(IR_RIGHT_a0);

void IRSensor::Initialize(Stream *p_debug) {
  if (p_debug != NULL) {
    p_debug->println("IR Sensors Initialized");
  }
}


IRSensor::IRSensor(int pin) {
  _pin = pin;
}


float IRSensor::getValue() {
  int analog_value_min = 0;
  int analog_value_max = 1023;
  int analog_value;
  float return_value;
  
  analog_value = analogRead(_pin);
  return_value = 100.0 * ((float)(analog_value - analog_value_min)) / ((float)(analog_value_max - analog_value_min));
  return return_value;
}
