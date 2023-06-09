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

 #include "SnifferSpeedCtrl.h"


SnifferSpeedCtrl SnifferSpeedCtrl::left_wheel(/* u_min = */ SNIFFER_MOTOR_DRIVE_CONTROL_MIN, 
                                            /* u_max = */ SNIFFER_MOTOR_DRIVE_CONTROL_MAX, 
                                            /* ts_millisec = */ SNIFFER_SPEED_CTRL_SAMPLING_TIME_MILLISEC);
SnifferSpeedCtrl SnifferSpeedCtrl::right_wheel(/* u_min = */ SNIFFER_MOTOR_DRIVE_CONTROL_MIN, 
                                             /* u_max = */ SNIFFER_MOTOR_DRIVE_CONTROL_MAX, 
                                             /* ts_millisec = */ SNIFFER_SPEED_CTRL_SAMPLING_TIME_MILLISEC);


void SnifferSpeedCtrl::begin(Stream *p_debug) {
  SnifferSpeedCtrl::left_wheel.setProportionalGain(SNIFFER_SPEED_CTRL_LEFT_KP_DEFAULT);
  SnifferSpeedCtrl::left_wheel.setIntegralGain(SNIFFER_SPEED_CTRL_LEFT_KI_DEFAULT);
  SnifferSpeedCtrl::left_wheel.setDerivativeGain(SNIFFER_SPEED_CTRL_LEFT_KD_DEFAULT);  
  SnifferSpeedCtrl::left_wheel.setDerivativeFilterGain(SNIFFER_SPEED_CTRL_LEFT_TAU_DEFAULT);
  SnifferSpeedCtrl::left_wheel.Reset();
  SnifferSpeedCtrl::right_wheel.setProportionalGain(SNIFFER_SPEED_CTRL_RIGHT_KP_DEFAULT);
  SnifferSpeedCtrl::right_wheel.setIntegralGain(SNIFFER_SPEED_CTRL_RIGHT_KI_DEFAULT);
  SnifferSpeedCtrl::right_wheel.setDerivativeGain(SNIFFER_SPEED_CTRL_RIGHT_KD_DEFAULT);  
  SnifferSpeedCtrl::right_wheel.setDerivativeFilterGain(SNIFFER_SPEED_CTRL_RIGHT_TAU_DEFAULT);
  SnifferSpeedCtrl::right_wheel.Reset();
}


SnifferSpeedCtrl::SnifferSpeedCtrl(float u_min, float u_max, unsigned long ts_millisec) : _sampler(ts_millisec > 0 ? ts_millisec : 1000u) {
  _proportional_gain = 0.0f;
  _integral_gain = 0.0f;
  _derivative_gain = 0.0f;
  _derivative_filter_gain = 0.0f;
  _target_value = 0.0f;
  
  _previous_control = 0.0f;
  _previous_error = 0.0f;
  _previous_previous_error = 0.0f;
  _previous_filtered_derivative = 0.0f;
  
  if (u_min > u_max) {
    _control_min = u_min;
    _control_max = u_max;
  } else {
    _control_min = u_max;
    _control_max = u_min;
  }  
}


void SnifferSpeedCtrl::setProportionalGain(float kp) {  
  if (kp >= 0.0f) {
    // we assume a non-negative number
    _proportional_gain = kp;
  }
}


void SnifferSpeedCtrl::setIntegralGain(float ki) {
  if (ki >= 0.0f) {
    // we assume a non-negative number
    _integral_gain = ki;      
  }  
}


void SnifferSpeedCtrl::setDerivativeGain(float kd) {
  if (kd >= 0.0f) {
    // we assume a non-negative number
    _derivative_gain = kd;    
  }    
}


void SnifferSpeedCtrl::setDerivativeFilterGain(float tau) {
  if (tau >= 0.0f) {
    // we assume a non-negative number
    _derivative_filter_gain = tau;
  }  
}

void SnifferSpeedCtrl::setTargetValue(float target_value) {
  _target_value = target_value;  
}


void SnifferSpeedCtrl::Reset() {
  _previous_control = 0.0f;
  _previous_error = 0.0f;
  _previous_previous_error = 0.0f;
  _previous_filtered_derivative = 0.0f;
  _sampler.begin();
}
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
float SnifferSpeedCtrl::calcControl(float current_value) {
  float sampling_time, control, error, proportional, integral, derivative, filtered_derivative;

  if (_sampler) {
    sampling_time = _sampler.getElapsedTimeInSec();
    error = _target_value - current_value;
  
    proportional = _proportional_gain * (error - _previous_error);
    integral = _integral_gain * sampling_time * error;
    derivative = _derivative_gain * (error - 2 * _previous_error + _previous_previous_error) / sampling_time;

    // anti-windup
    if ((integral < _control_min) || (integral > _control_max)) {
      integral = 0.0f;
    }

    // Derivative Low-Pass First-Order IIR Filter
    filtered_derivative = (_derivative_filter_gain / (_derivative_filter_gain + sampling_time)) * _previous_filtered_derivative
                        + (          sampling_time / (_derivative_filter_gain + sampling_time)) * derivative;

    control = _previous_control + proportional + integral + filtered_derivative;
    
    _previous_control = control;
    _previous_previous_error = _previous_error;
    _previous_error = error;
    _previous_filtered_derivative = filtered_derivative;    
  } else {
    control = _previous_control;
  }

  return control;
}
