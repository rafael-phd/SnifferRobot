/*
 * Author: Rafael Rodrigues da Silva
 * Date: April 29, 2023
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


#include "SnifferSpeedSensor.h" 


SnifferSpeedSensor SnifferSpeedSensor::left_wheel(SNIFFER_SPEED_SENSOR_ELAPSED_TIME_MAX_MILLISEC);
SnifferSpeedSensor SnifferSpeedSensor::right_wheel(SNIFFER_SPEED_SENSOR_ELAPSED_TIME_MAX_MILLISEC);


void SnifferSpeedSensor::begin(Stream *p_debug) {
  pinMode(SNIFFER_SPEED_SENSOR_RIGHT_out, INPUT);
  pinMode(SNIFFER_SPEED_SENSOR_LEFT_out, INPUT);

  attachInterrupt(digitalPinToInterrupt(SNIFFER_SPEED_SENSOR_RIGHT_out), SnifferSpeedSensor::rightSensor_Event, RISING);
  attachInterrupt(digitalPinToInterrupt(SNIFFER_SPEED_SENSOR_LEFT_out), SnifferSpeedSensor::leftSensor_Event, RISING);
  
  if (p_debug != NULL) {
    p_debug->println("Speed Sensors begind");
  }
}


void SnifferSpeedSensor::rightSensor_Event() {
  SnifferSpeedSensor::right_wheel.pulseEvent();  
}

void SnifferSpeedSensor::leftSensor_Event() {
  SnifferSpeedSensor::left_wheel.pulseEvent();  
}

SnifferSpeedSensor::SnifferSpeedSensor(unsigned long elapsed_time_max) : _sampler(elapsed_time_max) {
  _speed_millimeters_per_secs = 0.0f;
}


float SnifferSpeedSensor::read(bool is_forward) {
  if (_sampler) {
    _speed_millimeters_per_secs = 0.0f;
  } 
  
  if (!is_forward) {
    return -1.0f * _speed_millimeters_per_secs;  
  }  
  return _speed_millimeters_per_secs;  
}
  

void SnifferSpeedSensor::pulseEvent() {
  // Parameters
  const float wheel_perimeter_millimeter = 60.0f * PI;
  const float wheel_pulses_per_rot = 20.0f;

  if (_sampler) {
    _speed_millimeters_per_secs = 0.0f;
  } else {
    _speed_millimeters_per_secs = ((wheel_perimeter_millimeter / wheel_pulses_per_rot) / _sampler.getElapsedTimeInSec());
    _sampler.Reset();
  }    
}
