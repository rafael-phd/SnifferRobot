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


#include "speed_sensor.h" 


SpeedSensor SpeedSensor::left_wheel(SPEED_SENSOR_ELAPSED_TIME_MAX_MILLISEC);
SpeedSensor SpeedSensor::right_wheel(SPEED_SENSOR_ELAPSED_TIME_MAX_MILLISEC);


void SpeedSensor::Initialize(Stream *p_debug) {
  pinMode(SPEED_SENSOR_RIGHT_out, INPUT);
  pinMode(SPEED_SENSOR_LEFT_out, INPUT);

  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_RIGHT_out), SpeedSensor::rightSensor_Event, RISING);
  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_LEFT_out), SpeedSensor::leftSensor_Event, RISING);
  
  if (p_debug != NULL) {
    p_debug->println("Speed Sensors Initialized");
  }
}


void SpeedSensor::rightSensor_Event() {
  SpeedSensor::right_wheel.pulseEvent();  
}

void SpeedSensor::leftSensor_Event() {
  SpeedSensor::left_wheel.pulseEvent();  
}

SpeedSensor::SpeedSensor(unsigned long elapsed_time_max) : _sampler(elapsed_time_max) {
  _speed_millimeters_per_secs = 0.0f;
}


float SpeedSensor::getValue(bool is_forward) {
  if (_sampler) {
    _speed_millimeters_per_secs = 0.0f;
  } 
  
  if (!is_forward) {
    return -1.0f * _speed_millimeters_per_secs;  
  }  
  return _speed_millimeters_per_secs;  
}
  

void SpeedSensor::pulseEvent() {
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
