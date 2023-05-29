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
 
#ifndef SNIFFER_MOTOR_DRIVE_h
#define SNIFFER_MOTOR_DRIVE_h

#include <Arduino.h>

#define SNIFFER_MOTOR_DRIVE_enA 11
#define SNIFFER_MOTOR_DRIVE_enB 10
#define SNIFFER_MOTOR_DRIVE_in1 9
#define SNIFFER_MOTOR_DRIVE_in2 8
#define SNIFFER_MOTOR_DRIVE_in3 7
#define SNIFFER_MOTOR_DRIVE_in4 6

#define SNIFFER_MOTOR_DRIVE_CONTROL_MIN -255.0f
#define SNIFFER_MOTOR_DRIVE_CONTROL_MAX  255.0f

#define SNIFFER_MOTOR_DRIVE_CONTROL_IS_FWD(u, u_prev) (((u == 0) && (u_prev > 0)) || (u > 0))

class SnifferMotorDrive {
  private:
  int _control_a;
  int _control_b;
  int _control_a_prev;
  int _control_b_prev;

  bool _isForward(int control);

  public:
  SnifferMotorDrive();
  float getControl_A();
  float getControl_B();
  bool getIsForward_A();
  bool getIsForward_B();
  void setControl_A(float control);
  void setControl_B(float control);
//////////////////////////////////////////////
// Sniffer Robot
  static SnifferMotorDrive motor_drive;
  static void begin(Stream *p_debug);
//////////////////////////////////////////////
};

#endif
