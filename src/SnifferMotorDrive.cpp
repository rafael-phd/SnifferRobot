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
 
#include "SnifferMotorDrive.h"

SnifferMotorDrive SnifferMotorDrive::motor_drive;

void SnifferMotorDrive::begin(Stream *p_debug) {
  pinMode(SNIFFER_MOTOR_DRIVE_enA, OUTPUT);
  pinMode(SNIFFER_MOTOR_DRIVE_enB, OUTPUT);
  pinMode(SNIFFER_MOTOR_DRIVE_in1, OUTPUT);
  pinMode(SNIFFER_MOTOR_DRIVE_in2, OUTPUT);
  pinMode(SNIFFER_MOTOR_DRIVE_in3, OUTPUT);
  pinMode(SNIFFER_MOTOR_DRIVE_in4, OUTPUT);  
  if (p_debug != NULL) {
    p_debug->println("Motor Drive begind");
  }
}

SnifferMotorDrive::SnifferMotorDrive() {
  _control_a = 0;
  _control_b = 0;  
  _control_a_prev = 0;
  _control_b_prev = 0;  
}


float SnifferMotorDrive::getControl_A() {
  return (float)_control_a;
}


bool SnifferMotorDrive::getIsForward_A() {
  return SNIFFER_MOTOR_DRIVE_CONTROL_IS_FWD(_control_a, _control_a_prev);
}

bool SnifferMotorDrive::getIsForward_B() {
  return SNIFFER_MOTOR_DRIVE_CONTROL_IS_FWD(_control_b, _control_b_prev);
}


float SnifferMotorDrive::getControl_B() {
  return (float)_control_b;
}


void SnifferMotorDrive::setControl_A(float control) {
  int control_a_curr = (int)control;

  if (control_a_curr != _control_a) {
    _control_a_prev = _control_a;
    _control_a = control_a_curr;
    
  
    if (_control_a > 255) {
      _control_a = 255;
    } else if (_control_a < -255) {
      _control_a = -255;
    }
    
    analogWrite(SNIFFER_MOTOR_DRIVE_enA, abs(_control_a)); // set motor torque (0-255)
    if (_control_a == 0) {
      digitalWrite(SNIFFER_MOTOR_DRIVE_in1, LOW);
      digitalWrite(SNIFFER_MOTOR_DRIVE_in2, LOW);
    } else if (_control_a > 0) {
      digitalWrite(SNIFFER_MOTOR_DRIVE_in2, HIGH);
      digitalWrite(SNIFFER_MOTOR_DRIVE_in1, LOW);
    } else {
      digitalWrite(SNIFFER_MOTOR_DRIVE_in2, LOW);
      digitalWrite(SNIFFER_MOTOR_DRIVE_in1, HIGH);
    }
  }
}

void SnifferMotorDrive::setControl_B(float control) {
  int control_b_curr = (int)control;

  if (control_b_curr != _control_b) {
    _control_b_prev = _control_b;
    _control_b = control_b_curr;
  
    if (_control_b > 255) {
      _control_b = 255;
    } else if (_control_b < -255) {
      _control_b = -255;
    }
    
    analogWrite(SNIFFER_MOTOR_DRIVE_enB, abs(_control_b)); // set motor torque (0-255)
    if (_control_b == 0) {
      digitalWrite(SNIFFER_MOTOR_DRIVE_in3, LOW);
      digitalWrite(SNIFFER_MOTOR_DRIVE_in4, LOW);
    } else if (_control_b > 0) {
      digitalWrite(SNIFFER_MOTOR_DRIVE_in3, HIGH);
      digitalWrite(SNIFFER_MOTOR_DRIVE_in4, LOW);
    } else {
      digitalWrite(SNIFFER_MOTOR_DRIVE_in3, LOW);
      digitalWrite(SNIFFER_MOTOR_DRIVE_in4, HIGH);
    }
  }
}
