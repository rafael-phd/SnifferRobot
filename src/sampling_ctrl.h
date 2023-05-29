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
 
#ifndef SAMPLING_CTRL_h
#define SAMPLING_CTRL_h

#include <Arduino.h>
#include <limits.h> /* ULONG_MAX */

class SamplingController {
  private:
  unsigned long _current_time_millisec;
  unsigned long _elapsed_time_millisec;
  unsigned long _last_sampled_time_millisec;
  unsigned long _sampling_time_millisec;
  bool _isExpired();

  public:
  /*
   * Basic Sampling Time Control 
   * 
   * Our central idea is to regulate the timing of asynchronous and sequential processes 
   * by taking a sample at the first moment when the elapsed time exceeds the set sampling time. 
   */
  SamplingController(unsigned long ts_millisec);  
  explicit operator bool() const;  
  unsigned long getSamplingTimeInMillis();
  unsigned long getElapsedTimeInMillis();  
  float getElapsedTimeInSec();  
  void Reset();
  void Initialize();
};

#endif
