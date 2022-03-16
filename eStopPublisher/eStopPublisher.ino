/*
 * E Stop Publisher
 * Author: Muhammad Suleman, Maxwell Nguyen, Roshan Cheriyan, Malachai Smith
 */

#include <ros.h>

#include <std_msgs/Bool.h>

void CheckAutonomousCallback(const std_msgs::Bool & vehicle_engage_msg);

//All of the ROS definitions
//Handles the various nodes

ros::NodeHandle nh; 
std_msgs::Bool stop_gem_car;

//Initialize the ROS node, the subscribing node, and the publishing node
ros::Publisher estop_pub("gem_estop", & stop_gem_car);
//The node we are subscribing to for the autonomous light to either be on or off
ros::Subscriber < std_msgs::Bool > vehicle_engage("vehicle/engage", & CheckAutonomousCallback);

//Defines the arduino pins
int lightPin = 9; //Output Signal (D4)
int estopPin = 2; //E-Stop Pin (D2_
int wirelessEstopPin = 5;
bool vehicle_engaged = false;

// Button Debounce related variable
int estopBtnState = HIGH;         // the current reading from the input pin
int lastButtonState = HIGH;   // the previous reading from the input pin

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 500;    // the debounce time; increase if the output flickers

void CheckAutonomousCallback(const std_msgs::Bool & vehicle_engage_msg) {
   vehicle_engaged = vehicle_engage_msg.data;
}

void setup() {
  Serial.begin(57600);
  nh.initNode();
  pinMode(lightPin, OUTPUT);
  pinMode(estopPin, INPUT);
  pinMode(wirelessEstopPin, INPUT);

  nh.advertise(estop_pub);
  nh.subscribe(vehicle_engage);
}

//Main loop that the program runs
void loop() {

  int reading = digitalRead(estopPin) & digitalRead(wirelessEstopPin);
     
  // Makes the light in stead state
  digitalWrite(lightPin, LOW);

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  // whatever the reading is at, it's been there for longer than the debounce
  // delay, so take it as the actual current state:
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // if the button state has changed:
    if (reading != estopBtnState) {
      estopBtnState = reading;
    }
  }

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

  Serial.println(estopBtnState);
  // Checks for estop button press event
  if (estopBtnState) {
    stop_gem_car.data = false;
  } else {
    stop_gem_car.data = true;
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
  estop_pub.publish( & stop_gem_car);
  nh.spinOnce();
}
