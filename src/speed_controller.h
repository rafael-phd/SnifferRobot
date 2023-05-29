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
 
#ifndef SPEED_CONTROLLER_h
#define SPEED_CONTROLLER_h

#include <Arduino.h>

#include "motor_drive.h" /* MOTOR_DRIVE_CONTROL_MIN, MOTOR_DRIVE_CONTROL_MAX */
#include "sampling_ctrl.h" /* SamplingController */

#define SPEED_CONTROL_SAMPLING_TIME_MILLISEC 100u
#define SPEED_CONTROL_LEFT_KP_DEFAULT    0.0f
#define SPEED_CONTROL_LEFT_KI_DEFAULT    0.0f
#define SPEED_CONTROL_LEFT_KD_DEFAULT    0.0f
#define SPEED_CONTROL_LEFT_TAU_DEFAULT   0.0f
#define SPEED_CONTROL_RIGHT_KP_DEFAULT   0.0f
#define SPEED_CONTROL_RIGHT_KI_DEFAULT   0.0f
#define SPEED_CONTROL_RIGHT_KD_DEFAULT   0.0f
#define SPEED_CONTROL_RIGHT_TAU_DEFAULT  0.0f

class SpeedController {
  private:
  float _proportional_gain;
  float _integral_gain;
  float _derivative_gain;
  float _derivative_filter_gain;
  float _previous_control;
  float _previous_error;
  float _previous_previous_error;
  float _previous_filtered_derivative;
  float _control_min;
  float _control_max;
  float _target_value;
  SamplingController _sampler;

  public:
  SpeedController(float u_min, float u_max, unsigned long ts_millisec);
  void setProportionalGain(float kp);
  void setIntegralGain(float ki);
  void setDerivativeGain(float kd);
  void setDerivativeFilterGain(float tau);
  void setTargetValue(float target_value);
  void Reset();
  /*
   * Discrete-time PID Controller with Derivative Low-Pass Filter and Integral Anti-Windup.
   * sources: 1) https://en.wikipedia.org/wiki/PID_controller#Discrete_implementation
   *          2) https://thingsdaq.org/2022/04/07/digital-pid-controller/
   * 
   * u[k] = u[k-1] + kp * (e[k] - e[k-1]) + ki * ts * e[k] + kd * (e[k] - 2 * e[k-1] + e[k-2]) / ts
   * 
   * @param current_value current value of the observed state variable
   * @param target_value target value of the observed state variable
   * @return current PID control, i.e., u[k]
   */  
  float calcControl(float current_value);
//////////////////////////////////////////////
// Sniffer Robot
  static SpeedController left_wheel;
  static SpeedController right_wheel;
  static void Initialize(Stream *p_debug);
//////////////////////////////////////////////
};

#endif
