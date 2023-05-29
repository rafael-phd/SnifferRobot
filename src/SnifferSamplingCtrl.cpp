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

#include "SnifferSamplingCtrl.h" 


SnifferSamplingCtrl::SnifferSamplingCtrl(unsigned long ts_millisec) {
  _sampling_time_millisec = ts_millisec;
  begin();
}


unsigned long SnifferSamplingCtrl::getSamplingTimeInMillis() {
  return _sampling_time_millisec;
}

unsigned long SnifferSamplingCtrl::getElapsedTimeInMillis() {
  return _elapsed_time_millisec;
}


float SnifferSamplingCtrl::getElapsedTimeInSec() {
  return ((float)getElapsedTimeInMillis()) / 1000.0f;
}

bool SnifferSamplingCtrl::_isExpired() {
  _current_time_millisec = millis();
  
  if (_current_time_millisec >= _last_sampled_time_millisec) {
    _elapsed_time_millisec = _current_time_millisec - _last_sampled_time_millisec;
  } else {
    _elapsed_time_millisec = _current_time_millisec + (ULONG_MAX - _last_sampled_time_millisec);
  }   

  if (_elapsed_time_millisec >= _sampling_time_millisec) {
    Reset();
    return true;
  }
  return false;
}


SnifferSamplingCtrl::operator bool() const {
  return _isExpired();
}

void SnifferSamplingCtrl::Reset() {
  _last_sampled_time_millisec = _current_time_millisec;
}

void SnifferSamplingCtrl::begin() {
  _current_time_millisec = millis();
  _last_sampled_time_millisec = _current_time_millisec;
  _elapsed_time_millisec = 0u;
}
 
