/* Import necessary libraries: ROS, Servo, String */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <Servo.h>

/* Define important variables */

ros::NodeHandle nh;
//std_msgs::String objLoc;
std_msgs::Int32 objLoc;
std_msgs::String debug;

/* Subscriber Callback function */

void messageCb(const std_msgs::Int32 &obj_loc)
{
  //global objLoc;
  objLoc = obj_loc;
}

/* Define subscriber and servo objects */

ros::Subscriber<std_msgs::Int32> obj_loc("object_location", &messageCb);
ros::Publisher debug_pub("arduino_debug", &debug);
Servo servoTop, servoBottom;

/* Define servo pins, limits, actions and signals */

int pinServoTop = 3;
int pinServoBottom = 5;

int servoTopMin = 45;
int servoTopMax = 135;
int servoBottomMin = 45;
int servoBottomMax = 135;

int up = 1;
int down = -1;
int left = 1;
int right = -1;
int none = 0;

int servoTopSignal = 90;
int servoBottomSignal = 90;

/* Setup code: Run once */

void setup() {

  Serial.begin(57600);

  /* Attach servos to pins */
  
  servoTop.attach(pinServoTop);
  servoBottom.attach(pinServoBottom);

  /* Start subscriber node */

  nh.initNode();
  nh.subscribe(obj_loc);
  nh.advertise(debug_pub);
}

/* Loop code: Run repeatedly */

void loop() {

  /* Check object location and update servo signals */

  if(objLoc.data == 11)
  {
    servoTopSignal += up;
    servoBottomSignal += left;
  }
  else if(objLoc.data == 12)
  {
    servoTopSignal += up;
    servoBottomSignal += none;
  }
  else if(objLoc.data == 13)
  {
    servoTopSignal += up;
    servoBottomSignal += right;
  }
  else if(objLoc.data == 21)
  {
    servoTopSignal += none;
    servoBottomSignal += left;
  }
  else if(objLoc.data == 22)
  {
    debug.data = "DETECTED";
    servoTopSignal += none;
    servoBottomSignal += none;
  }
  else if(objLoc.data == 23)
  {
    servoTopSignal += none;
    servoBottomSignal += right;
  }
  else if(objLoc.data == 31)
  {
    servoTopSignal += down;
    servoBottomSignal += left;
  }
  else if(objLoc.data == 32)
  {
    servoTopSignal += down;
    servoBottomSignal += none;
  }
  else if(objLoc.data == 33)
  {
    servoTopSignal += down;
    servoBottomSignal += right;
  }

  /* Ensure servo limits */

  if(servoTopSignal > servoTopMax)
  {
    servoTopSignal = servoTopMax;
  }
  else if(servoTopSignal < servoTopMin)
  {
    servoTopSignal = servoTopMin;
  }

  if(servoBottomSignal > servoBottomMax)
  {
    servoBottomSignal = servoBottomMax;
  }
  else if(servoBottomSignal < servoBottomMin)
  {
    servoBottomSignal = servoBottomMin;
  }

  //debug.data = objLoc.data;
  debug_pub.publish(&debug);

  /* Apply servo signals */

  servoTop.write(servoTopSignal);
  servoBottom.write(servoBottomSignal);

  /* Spin once and wait */
  
  nh.spinOnce();
  delay(50);
}


