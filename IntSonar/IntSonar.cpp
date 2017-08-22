/****************************************************
   IntSonar: Another Arudino library for the HC-SR04 Sonar modules
   Created by Simon Pauw, August 2017

   This library does low-cost polling of the sonar, using interruts (so make sure
   you connect the echo-pin to an interrupt capable Arduino pin). The advantage of
   making the polling low cost is that you can get more measurements (and therefor
   more accuracy) than with other methods.

   The interrupt handling requires to maintain the measurement values in a global scope.
   So I've chose not to bother with classes and implement everything as normal functions.
*/

#include "IntSonar.h"
#include "Arduino.h"

namespace Sonar
{
  

void print_int_array(int a[], int n);
void print_long_array(unsigned long a[], int n);
/* window variables */
int _window_size, _window_index, _poll_index;
int _window[MAX_WINDOW_SIZE];
unsigned long _timestamps[MAX_WINDOW_SIZE];

/* timer */
unsigned long _time;

/* polling vars */
int _min_d, _max_d, _v_sound, _zero_cal;

/* pins */
int _trigger_pin, _echo_pin;

/* Callback for end of echo pulse */
void _end_ISR() {
  unsigned long dtime = micros() - _time;
  _window[_window_index] = ((dtime * _v_sound) / 2000) - _zero_cal;
  _window_index++;
  _window_index %= _window_size;
  detachInterrupt(digitalPinToInterrupt(_echo_pin));
}

/* Callback for start of echo pulse */
void _start_ISR()
{
  _time = micros();
  attachInterrupt(digitalPinToInterrupt(_echo_pin), _end_ISR, FALLING);
}

/* init */
void init(int trigger_pin, int echo_pin, int min_d, int max_d, int window_size)
{
  /* window init */
  _window_size = window_size;
  _window_index = 0;
  _poll_index = 0;
  for (int i = 0; i < _window_size; i++)
  {
    _window[i] = 0;
    _timestamps[i] = 0;
  }

  /* timer */
  _time = 0;

  /* polling vars */
  _min_d       = min_d;
  _max_d       = max_d;
  _v_sound     = 350;
  _zero_cal    = 0;

  /* pins */
  _trigger_pin = trigger_pin;
  _echo_pin    = echo_pin;

  /* pin setup */
  pinMode(_trigger_pin, OUTPUT);
  pinMode(_echo_pin, INPUT);
  digitalWrite(_trigger_pin, HIGH);

}

/* compute variance over window */
int variance(int distance)
{
  unsigned long tv = 0;
  for (int i = 0; i < _window_size; i++)
  {
    int d = _window[i] - distance;
    tv += d * d;
  }
  return tv / _window_size;
}

/* compute mean over window */
int mean_distance_mm()
{
  int n_data_points = 0;
  int n_too_close   = 0;
  int n_too_far     = 0;
  unsigned long total = 0;
  for (int i = 0; i < _window_size; i++)
  {
    int m = _window[i];
    //Serial.println(m);
    if (m == 0) {}
    else if (m < _min_d) n_too_close++;
    else if (m >= _max_d) n_too_far++;
    else
    {
      total += m;
      n_data_points++;
    }
  }
  if (n_data_points < (_window_size / 2))
  {
    if (n_too_close > n_too_far  ) return TOO_CLOSE;
    if (n_too_far   > n_too_close) return TOO_FAR;
    return NO_DATA;
  }
  else return (int)(total / n_data_points);
}

int  mean_speed_mm_s()
{
  int distances[_window_size];
  int n_samples = 0;
  unsigned long times[_window_size];
  for(int i = 0; i<_window_size-1; i++)
  {
    int i_index = (i + _poll_index) % _window_size;
    int val = _window[i_index];
    if(val >= _min_d && val < _max_d)
    {
      distances[n_samples]  = val;
      times[n_samples]      = _timestamps[i];
      n_samples++;
    }
  }
//  print_long_array(times, n_samples);
//  print_int_array(distances, n_samples);
  int n_tangents = n_samples - 1;
  if(n_tangents < 1) return 0;

  long total_tangent = 0;
  for(int i = 0; i< n_tangents; i++)
  {
    int j = i+1;
    long dt = times[j]-times[i];
    long dx = distances[j]-distances[i];
    total_tangent += (1000*dx)/dt; 
  }
  return total_tangent/n_tangents;
}

/* polls the HC-SR04. Sends a pulse to the trigger pin, and setsup interrupt
    for the echo pin
*/
void poll()
{
  // ignore further interrupts
  detachInterrupt(digitalPinToInterrupt(_echo_pin));

  // set missed measurements to 0
  while (_window_index != _poll_index)
  {
    _window[_window_index] = 0;
    _window_index++;
    _window_index %= _window_size;
  }

  _timestamps[_poll_index] = millis();

  // set interrupt on echo pin
  attachInterrupt(digitalPinToInterrupt(_echo_pin), _start_ISR, RISING);

  // send pulse
  digitalWrite(_trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(_trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigger_pin, LOW);

  _poll_index++;
  _poll_index %= _window_size;
}

void set_v_sound(int v)
{
  _v_sound = v;
}

void set_zero_cal(int z)
{
  _zero_cal = z;
}

void print_int_array(int a[], int n)
{
  Serial.print("[");
  if(n > 0)
  {
    for(int i = 0; i < n-1; i++)
    {
      Serial.print(a[i]);
      Serial.print(", ");
    }
    Serial.print(a[n-1]);
  }
  Serial.print("]");
}
void print_long_array(unsigned long a[], int n)
{
  Serial.print("[");
  if(n > 0)
  {
    for(int i = 0; i < n-1; i++)
    {
      Serial.print(a[i]);
      Serial.print(", ");
    }
    Serial.print(a[n-1]);
  }
  Serial.print("]");
}
}

