/****************************************************
 * IntSonar: Another Arudino library for the HC-SR04 Sonar modules
 * Created by Simon Pauw, August 2017
 * 
 * This library does non-blocking polling of the sonar, using interruts (so make sure 
 * you connect the echo-pin to an interrupt capable Arduino pin). The main advantage of
 * making the polling non-blocking is that you can get more measurements (and therefor
 * more accuracy) than with other methods.
 */

namespace Sonar
{
  #define MAX_WINDOW_SIZE 30
  
  #define NO_DATA         -1
  #define TOO_FAR         -2
  #define TOO_CLOSE       -3

  /* init
   *  trigger_pin : pin connected to "trig" of the HC-SR04
   *  echo_pin    : pin connected to "echo" of the HC-SR04 (should support interrupts!)
   *  min_d       : minimum distance in mm
   *  max_d       : maximum distance in mm
   *  window_size : number of samples to keep in window (should be at least 3)
   */
  void init(int trigger_pin, int echo_pin, int min_d, int max_d, int window_size);

  /* distance in millimeters (average over the window) */
  int  mean_distance_mm();
  int  variance(int distance);
  int  mean_speed_mm_s();

  /* poll the sonar */
  void poll();

  /* Set speed of sound */
  void set_v_sound(int v);

  /* Calibrate zero-point of measurement */
  void set_zero_cal(int z);
}
