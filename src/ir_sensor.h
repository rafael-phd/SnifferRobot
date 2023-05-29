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
 
#ifndef IR_SENSOR_h
#define IR_SENSOR_h

#include <Arduino.h>

#define IR_LEFT_a0              A0 
#define IR_RIGHT_a0             A1 


class IRSensor {
  private:
  int _pin;

  public:
  IRSensor(int pin);
  float getValue();
//////////////////////////////////////////////
// Sniffer Robot
  static IRSensor front_left;
  static IRSensor front_right;
  static void Initialize(Stream *p_debug);
//////////////////////////////////////////////
};

#endif
