
#include <ros.h>
#include <ArduinoHardware.h>
#include <SimpleTimer.h>
#include <geometry_msgs/Twist.h>
#include <Ultrasonic.h>
#include <math.h>
#include <Servo.h>


ros::NodeHandle nh;

Servo servoLeft;
Servo servoRight;

SimpleTimer timer;
int stopid;

void messageCb( const geometry_msgs::Twist& robot_controls){
  
  //Determine velocity 
  double vel = fabs(robot_controls.linear.x);
  if(vel < fabs(robot_controls.angular.z)){
    vel = fabs(robot_controls.angular.z);
  }
    
  //check if there is an obstactle and call move
   if(!(digitalRead(5) == 0 || digitalRead(7) == 0)){
    if(robot_controls.linear.x > 0){
      move(255*vel, 255*vel);
    } else if(robot_controls.angular.z < 0){
      move(255*vel, -255*vel);
    } else if(robot_controls.angular.z > 0){
      move(-255*vel, 255*vel);
    }
  } else {
     move(0, 0);
  }
  
  //Even with an obstactle you can drive backwards
  if(robot_controls.linear.x < 0){
    move(-255*vel, -255*vel);
  }
    
  //Check if there is already a stop timer active
  timer.restartTimer(stopid);
}

//Move leftmotor and right motor between 0 and 255
void move(double leftmotor, double rightmotor){ 
  
    if(rightmotor > 0){
      //Forward right motor
      servoRight.writeMicroseconds(1300);
    } else if(rightmotor < 0){
       servoRight.writeMicroseconds(1700);
    }else{
      servoRight.writeMicroseconds(1500);  
    }
    
    if(leftmotor > 0){
      //Forward left motor
      servoLeft.writeMicroseconds(1700);;
    } else if(leftmotor < 0){
      servoLeft.writeMicroseconds(1300);
    } else{
      servoLeft.writeMicroseconds(1500);    
    }  
}

void stopRobot(){
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
}
  ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void setup()
{

  pinMode(7, INPUT);
  pinMode(5, INPUT);
  nh.initNode();
  nh.subscribe(sub);
  servoLeft.attach(13);
  servoRight.attach(12);
  stopRobot(); 
  stopid = timer.setInterval(500, stopRobot);
}

void loop()
{
  nh.spinOnce();
  timer.run();
  delay(10);
}
