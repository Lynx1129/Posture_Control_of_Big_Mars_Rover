  // File:          rover_test1.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#define TIME_STEP 64
// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

 // Motor *wheelmotor1 = robot->getMotor("wheelmotor1");
 // Motor *wheelmotor2 = robot->getMotor("wheelmotor2");
 // Motor *wheelmotor3 = robot->getMotor("wheelmotor3");
 // Motor *wheelmotor4 = robot->getMotor("wheelmotor4");
 
 // Motor *pullmotor1 = robot->getMotor("pullmotor1");
 // Motor *pullmotor2 = robot->getMotor("pullmotor2");
 // Motor *pullmotor3 = robot->getMotor("pullmotor3");
 // Motor *pullmotor4 = robot->getMotor("pullmotor4");
 
 // wheelmotor1->setPosition(20);
   // wheelmotor2->setPosition(20);
 // wheelmotor3->setPosition(-20);
 // wheelmotor4->setPosition(-20);
 
 // pullmotor1->setPosition(5.11);
 // pullmotor2->setPosition(0);
 // pullmotor3->setPosition(0);
 // pullmotor4->setPosition(0.4);
 
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
