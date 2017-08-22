/* Use IntSonar to read out the distance and speed of an object 
 * with HC-SR04.
 *  
 * Created by Simon Pauw 2017
 */
#include <IntSonar.h>

void setup() {
  // setup sonar
  int trigger_pin   = 3;
  int echo_pin      = 2;
  int min_distance  = 40;   // mm
  int max_distance  = 2000; // mm
  int window        = 16;   // use at least 3, but pref. more.
  Sonar::init(trigger_pin, echo_pin, min_distance, max_distance, window);
  
  // start serial connection
  Serial.begin(9600);

  // led
  pinMode(13, OUTPUT);

  // wait for serial connection
  delay(1000);
}


void loop() {
  // Do any number of polls before getting the measurement
  // More measurements gives more accuracy, but more than 
  // the size of the window is pointless.
  for (int i = 0; i < 16; i++)
  {
    Sonar::poll();
    delay(20);
  }

  // get distance
  int distance = Sonar::mean_distance_mm();

  // show results
  Serial.println("===========================");
  if (distance == TOO_CLOSE)
  {
    // distance lower than lower bound
    Serial.println("Object too close");
  }
  else if (distance == TOO_FAR)
  { 
    // distance higher than upper bound
    Serial.println("Object too far");
  } else if (distance == NO_DATA)
  {
    // distance either to high for HC-SR04, or some issue with the wiring
    Serial.println("No data");
  } else
  {
    // get the distance
    Serial.print("Distance\t= ");
    Serial.println(distance);

    // get variance for said distance
    int variance = Sonar::variance(distance);
    Serial.print("Variance\t= ");
    Serial.println(variance);

    // get the tangent (speed)
    int speed = Sonar::mean_speed_mm_s();
    Serial.print("Speed\t\t= ");
    Serial.println(speed);

    // If something comes too close too quickly, panic! (And,
    // show this underwelmingly by fleshing LED 13. 
    if(distance < 200 && speed < -200)
      digitalWrite(13, HIGH);
    else
      digitalWrite(13, LOW);
  }
  delay(100);
}








