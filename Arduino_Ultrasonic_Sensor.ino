

#include <ros.h>
#include <std_msgs/Float64.h>

ros::NodeHandle  nh;

std_msgs::Float64 dist_msg;
ros::Publisher distMsg("distMsg", &dist_msg);




// defines pins numbers
const int trigPin = 13;
const int echoPin = 10;

// defines variables
long duration;
double distance;

void setup() {
pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT); // Sets the echoPin as an Input
Serial.begin(57600); // Starts the serial communication
 nh.initNode();
  nh.advertise(distMsg);
}

void loop() {
// Clears the trigPin
digitalWrite(trigPin, LOW);
delayMicroseconds(2);

// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);

// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH);

// Calculating the distance
distance= (duration*0.034/2) / 2.54;

// Prints the distance on the Serial Monitor
delay(100);

if (distance < 200){
  if (distance > 1){
  Serial.print("Distance: ");
Serial.println(distance);
  dist_msg.data = distance;
    distMsg.publish( &dist_msg );
    nh.spinOnce();
  }
}
}
