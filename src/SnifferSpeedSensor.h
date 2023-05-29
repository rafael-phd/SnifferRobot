/*
 * Author: Rafael Rodrigues da Silva
 * Date: May 2, 2023
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
 
#ifndef SNIFFER_SPEED_SENSOR_h
#define SNIFFER_SPEED_SENSOR_h

#include <Arduino.h>

#include "SnifferSamplingCtrl.h" /* SnifferSamplingCtrl */

#define SNIFFER_SPEED_SENSOR_RIGHT_out                  3
#define SNIFFER_SPEED_SENSOR_LEFT_out                   2
#define SNIFFER_SPEED_SENSOR_ELAPSED_TIME_MAX_MILLISEC  1000u

class SnifferSpeedSensor {
  private:
  float _speed_millimeters_per_secs;
  SnifferSamplingCtrl _sampler;

  public:
  SnifferSpeedSensor(unsigned long elapsed_time_max);
  float read(bool is_forward);
  void pulseEvent();
//////////////////////////////////////////////
// Sniffer Robot
  static SnifferSpeedSensor left_wheel;
  static SnifferSpeedSensor right_wheel;
  static void leftSensor_Event();
  static void rightSensor_Event();
  static void begin(Stream *p_debug);
//////////////////////////////////////////////
};

#endif
