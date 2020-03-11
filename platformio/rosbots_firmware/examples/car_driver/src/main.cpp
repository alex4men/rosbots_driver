/**
 * Motor Driver Example
 */

// Fixes ROS-Arduino communication issues
#define USE_USBCON
#include <Arduino.h>
#include <ros.h>
#include <Servo.h>

#include <std_msgs/Float32.h>

ros::NodeHandle nh;

Servo throttle;
Servo steering;

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

#define THROTTLE_PWM 14
#define STEERING_PWM 15


void throttleCb( const std_msgs::Float32 &power ) {
  //nh.loginfo("Wheel Power - Right");
  //char result[8];
  //dtostrf(wheel_power.data, 6, 2, result); 
  //nh.loginfo(result);
  throttle.writeMicroseconds(power.data);
}
void steeringCb( const std_msgs::Float32 &power ) {
    //nh.loginfo("Wheel Power - Left");
    steering.writeMicroseconds(power.data);
}
ros::Subscriber<std_msgs::Float32> sub_throttle("throttle",
                                            &throttleCb );
ros::Subscriber<std_msgs::Float32> sub_steering("steering",
                                           &steeringCb );


void setup()
{
  // Tried to fix ROS-Arduino communication issues
  // nh.getHardware()->setBaud(115200);
  // Serial.begin(115200);
  // Serial.begin(57600);

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Init motors to stop
  throttle.attach(THROTTLE_PWM);
  throttle.writeMicroseconds(1500);
  steering.attach(STEERING_PWM);
  steering.writeMicroseconds(1500);

  nh.initNode();
  nh.subscribe(sub_throttle);
  nh.subscribe(sub_steering);

  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  //nh.loginfo("Log Me");
  nh.spinOnce();
  // Serial.println('test');

  // wait for a second
  //delay(1000);
}
