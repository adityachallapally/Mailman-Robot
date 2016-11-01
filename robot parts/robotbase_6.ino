/**
* Group number : 6
* Student 1: 
* Bryan van Wijk 4363329
* Student 2:
* Dorian de Koning 4348737
*/

#include <ros.h>
#include <ArduinoHardware.h>
#include <SimpleTimer.h>
#include <geometry_msgs/Twist.h>
#include <Ultrasonic.h>
#include <math.h>

//Right motor pins
#define REV_R 7   //Reverse
#define EN_R 24   //Enable
#define FWD_R 6   //Forward

//Left motor pins
#define REV_L 3  //Reverse
#define EN_L 25  //Enable
#define FWD_L 2  //Forward

//Set right serial port for bluetooth connection
class NewHardware : public ArduinoHardware {
  public : NewHardware ( ) : ArduinoHardware(&Serial1, 57600) {};
};

//Set the pins of the ultrasonic sensor
Ultrasonic ultrasonic(23,22);

ros::NodeHandle_<NewHardware> nh;

// the timer object
SimpleTimer timer;

//ID of the stop timer
int stopid;

//Call back when twist message received
void messageCb( const geometry_msgs::Twist& robot_controls) {
  
  //Determine velocity based on message
  double vel = fabs(robot_controls.linear.x);
  if(vel < fabs(robot_controls.angular.z)){
    vel = fabs(robot_controls.angular.z);
  }
    
  //check if there is an obstactle and call move with right parameters
  if(ultrasonic.Ranging(CM) > 10){
    if(robot_controls.linear.x > 0){
      move(255*vel, 255*vel);
    } else if(robot_controls.angular.z < 0){
      move(255*vel, -0.5*255*vel);
    } else if(robot_controls.angular.z > 0){
      move(-0.5*255*vel, 255*vel);
    } else {
      move(0, 0);
    }
  } else {
     move(0, 0);
  }
  
  //Even with an obstactle in front you can drive backwards
  if(robot_controls.linear.x < 0){
    move(-255*vel, -255*vel);
  }
    
  //Check if there is already a stop timer active and restart to stop after 500 ms
  if(timer.isEnabled(stopid)) {  
    timer.restartTimer(stopid);
  } else{
    stopid = timer.setTimeout(500, stopRobot);
  }
}

//Subscribe to the cmd_vel topic
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

//Move leftmotor and right motor between -255 and 255
//Negative is opposite direction from positive
void move(double leftmotor, double rightmotor){ 
  
    // Enable motors
    analogWrite(EN_R, 255);
    analogWrite(EN_L, 255);
    
    if(rightmotor > 0){
      //Forward right motor
      analogWrite(REV_R, 0);
      analogWrite(FWD_R, rightmotor);
    }else{
      //Backward right motor
      analogWrite(REV_R, fabs(rightmotor));
      analogWrite(FWD_R, 0);  
    }
    
    if(leftmotor > 0){
      //Forward left motor
      analogWrite(FWD_L, leftmotor);
      analogWrite(REV_L, 0);
    } else{
      //Backward right motor
      analogWrite(FWD_L, 0);
      analogWrite(REV_L, fabs(leftmotor));      
    }  
}

//Stop robot and disable motors
void stopRobot(){
    move(0, 0);
    analogWrite(EN_R, 0);
    analogWrite(EN_L, 0);
}

//Setup the ros node and be sure the robot is standing still
void setup() {
  nh.initNode();
  nh.subscribe(sub);
  stopRobot();
}

void loop() {
   nh.spinOnce();
   timer.run();
   delay(10);
}
