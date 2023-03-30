#define IN4 11
#define IN3 10
#define IN2 6
#define IN1 5

//#define USE_USBCON

#include <ros.h>
#include <std_msgs/Float32.h>


ros::NodeHandle  nh;

void leftTrackCallback(const std_msgs::Float32& speed) {
  if (speed.data > 0) {
    //forward
    analogWrite(IN4, LOW);
    analogWrite(IN3, constrain(abs(speed.data * 255), 0, 255));
  } else {
    //reverse
    analogWrite(IN4, constrain(abs(speed.data * 255), 0, 255));
    analogWrite(IN3, LOW);
  }

}

void rightTrackCallback(const std_msgs::Float32& speed) {
  if (speed.data > 0) {
    //forward
    analogWrite(IN2, LOW);
    analogWrite(IN1, constrain(abs(speed.data * 255), 0, 255));
  } else {
    //reverse
    analogWrite(IN2, constrain(abs(speed.data * 255), 0, 255));
    analogWrite(IN1, LOW);
  }

}

ros::Subscriber<std_msgs::Float32> left_sub("left_track", leftTrackCallback );
ros::Subscriber<std_msgs::Float32> right_sub("right_track", rightTrackCallback );

void setup()
{

  pinMode(IN4, OUTPUT);
  pinMode(IN3, OUTPUT);

  pinMode(IN2, OUTPUT);
  pinMode(IN1, OUTPUT);

  nh.initNode();

  nh.subscribe(left_sub);
  nh.subscribe(right_sub);
}

void loop()
{
  nh.spinOnce();
  delay(50); // 20Hz
}
