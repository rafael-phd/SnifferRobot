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
 
#ifndef SNIFFER_IR_SENSOR_h
#define SNIFFER_IR_SENSOR_h

#include <Arduino.h>

#define SNIFFER_IR_LEFT_a0              A0 
#define SNIFFER_IR_RIGHT_a0             A1 


class SnifferIRSensor {
  private:
  int _pin;

  public:
  SnifferIRSensor(int pin);
  float read();
//////////////////////////////////////////////
// Sniffer Robot
  static SnifferIRSensor front_left;
  static SnifferIRSensor front_right;
  static void begin(Stream *p_debug);
//////////////////////////////////////////////
};

#endif
