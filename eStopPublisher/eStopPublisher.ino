/*
 * E Stop Publisher
 * Author: Muhammad Suleman, Maxwell Nguyen, Roshan Cheriyan, Malachai Smith
 */

#include <ros.h>

#include <std_msgs/Bool.h>

void CheckAutonomousCallback(const std_msgs::Bool & vehicle_engage_msg);

//All of the ROS definitions
ros::NodeHandle nh; //Handles the various nodes
std_msgs::Bool bool_msg;

//Initialize the ROS node, the subscribing node, and the publishing node
ros::Publisher estop_pub("gem_estop", & bool_msg);

//The node we are subscribing to for the autonomous light to either be on or off
ros::Subscriber < std_msgs::Bool > vehicle_engage("vehicle/engage", & CheckAutonomousCallback);

//Defines the arduino pins
//int inputPin = A4;  //Input Voltage (A4)
int lightPin = 9; //Output Signal (D4)
int estop1 = 2; //E-Stop Pin (D2)
int estop2 = 3; //E-Stop Pin (D3)
int estop3 = 4; //E-Stop Pin (D4) -- SEE WHAT PINS ARE AVAILABLE. COULD DAISY CHAIN THE ESTOPS TOGETHER TO ONE PIN
int estop4 = 5; //E-Stop Pin (D5)
bool vehicle_engaged = false;

void CheckAutonomousCallback(const std_msgs::Bool & vehicle_engage_msg) {
   vehicle_engaged = vehicle_engage_msg.data;
}

void setup() {
  //Serial.begin(57600);
  
  nh.initNode();
  pinMode(lightPin, OUTPUT);
  pinMode(estop1, INPUT);
  pinMode(estop2, INPUT);
  pinMode(estop3, INPUT);
  pinMode(estop4, INPUT);

  nh.advertise(estop_pub);
  nh.subscribe(vehicle_engage);
}

//Main loop that the program runs
void loop() {

  bool estopBtnState = digitalRead(estop1);
  /*
   * || digitalRead(
    estop2) || digitalRead(estop3) || digitalRead(estop4)
   */
  //Serial.println(estopBtnState);
     
  // Makes the light in stead state
  digitalWrite(lightPin, LOW);


  /*
   * If vehicle_engaged is true and estop button is not presses. 
   * (estopBtnState = LOW) is pressed and (estopBtnState == HIGH)
   */
  if (vehicle_engaged && estopBtnState) {
    delay(250);
    digitalWrite(lightPin, HIGH);
    delay(250);
    vehicle_engaged = false;
  }

  // Checks for estop button press event
  if (estopBtnState == LOW) {
    bool_msg.data = true;
  } else {
    bool_msg.data = false;
  }
  estop_pub.publish( & bool_msg);
  nh.spinOnce();
}
