/*
 * Author: Rafael Rodrigues da Silva
 * Date: May 21, 2023
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

#include "SnifferDurationSensor.h"

SnifferDurationSensor SnifferDurationSensor::duration;

void SnifferDurationSensor::begin(Stream *p_debug) {
  SnifferDurationSensor::duration.Reset();
  
  if (p_debug != NULL) {
    p_debug->println("Duration begind");
  }
}


void SnifferDurationSensor::Reset() {
  _initial_time_millis = millis();
}


SnifferDurationSensor::SnifferDurationSensor() {
  Reset();
}


float SnifferDurationSensor::read() {
  unsigned long current_time_millis, elapsed_time_millis;

  current_time_millis = millis();
  if (current_time_millis >= _initial_time_millis) {
    elapsed_time_millis = current_time_millis - _initial_time_millis;
  } else {
    elapsed_time_millis = current_time_millis + (ULONG_MAX - _initial_time_millis);
  }   
  
  return ((float)elapsed_time_millis) / 1000.0f;
}
