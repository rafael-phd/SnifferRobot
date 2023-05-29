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
 
#ifndef SNIFFER_DURATION_SENSOR_h
#define SNIFFER_DURATION_SENSOR_h

#include <Arduino.h>
#include <limits.h> /* ULONG_MAX */


class SnifferDurationSensor {
  private:
  unsigned long _initial_time_millis;

  public:
  SnifferDurationSensor();
  void Reset();
  float read();
//////////////////////////////////////////////
// Sniffer Robot
  static SnifferDurationSensor duration;
  static void begin(Stream *p_debug);
//////////////////////////////////////////////
};

#endif
