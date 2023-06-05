// File:          Rover_pid_test1.cpp
// Date:          2022.4.5
// Description:   PID test for rover
// Author:        Yin Siyuan
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <iostream>
#include <algorithm>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

 
  int timeStep = 32;  // get the time step of the current world.
  int i=0; //simulation step counter
  
  double P_roll = 10; //pid control parameter
  double I_roll = 0.1;
  double D_roll = 1;//pid control parameter
  double P_pitch = 10;
  double I_pitch = 0.1;
  double D_pitch = 1;
  double Roll,Pitch,Yaw;  //the 3 values used to represent body pose
  double last_Roll = 0; double last_Pitch = 0;
  double sum_Roll = 0; double sum_Pitch = 0;

  InertialUnit *imu = robot->getInertialUnit("imu");
  imu->enable(timeStep);
  
  Motor *wheelmotor1 = robot->getMotor("wheelmotor1");
  Motor *wheelmotor2 = robot->getMotor("wheelmotor2");
  Motor *wheelmotor3 = robot->getMotor("wheelmotor3");
  Motor *wheelmotor4 = robot->getMotor("wheelmotor4");

 
  Motor *pullmotor1 = robot->getMotor("pullmotor1");
  Motor *pullmotor2 = robot->getMotor("pullmotor2");
  Motor *pullmotor3 = robot->getMotor("pullmotor3");
  Motor *pullmotor4 = robot->getMotor("pullmotor4");


  while (robot->step(timeStep) != -1) {
  
    const double *imu_value = imu->getRollPitchYaw();
    Roll = imu_value[0];
    Pitch = imu_value[1];
    Yaw = imu_value[2];
    
    Roll = (round(10000.0 * Roll))/10000.0;
    Pitch = (round(10000.0 * Pitch))/10000.0;
    Yaw = (round(10000.0 * Yaw))/10000.0;
    
    std::cout << "Roll Pitch Yaw is:" << "\t" << Roll << "\t" << Pitch << "\t" << Yaw <<std::endl;
    std::cout << "Simulation step is: " << i << std::endl;
    
    if(i<=100)
    {
     pullmotor1->setPosition(4.91);
     pullmotor2->setPosition(-0.2);
     pullmotor3->setPosition(-0.2);
     pullmotor4->setPosition(0.2);
    }
    else
    {
     wheelmotor1->setPosition(INFINITY);
     wheelmotor2->setPosition(INFINITY);
     wheelmotor3->setPosition(INFINITY);
     wheelmotor4->setPosition(INFINITY);
     wheelmotor1->setVelocity(1.6);
     wheelmotor2->setVelocity(1.6);
     wheelmotor3->setVelocity(-1.6);
     wheelmotor4->setVelocity(-1.6);
     
     pullmotor1->setPosition(INFINITY);
     pullmotor2->setPosition(INFINITY);
     pullmotor3->setPosition(INFINITY);
     pullmotor4->setPosition(INFINITY);
     
     double vel_roll = P_roll * Roll + D_roll * (Roll-last_Roll) + I_roll * sum_Roll;
     double vel_pitch = P_pitch * Pitch + D_pitch * (Pitch-last_Pitch) + I_pitch * sum_Pitch;
     double pullmotor_vel1 = std::min (std::max( vel_roll - vel_pitch,-0.2), 0.2);
     double pullmotor_vel2 = std::min (std::max( vel_roll + vel_pitch,-0.2), 0.2);
     double pullmotor_vel3 = std::min (std::max( - vel_roll - vel_pitch,-0.2), 0.2);
     double pullmotor_vel4 = std::min (std::max( - vel_roll + vel_pitch,-0.2), 0.2);
     
     pullmotor1->setVelocity(pullmotor_vel1);
     pullmotor2->setVelocity(pullmotor_vel2);
     pullmotor3->setVelocity(pullmotor_vel3);
     pullmotor4->setVelocity(pullmotor_vel4);
    }
 
    last_Roll = Roll;
    last_Pitch = Pitch;
    sum_Roll +=Roll;
    sum_Pitch +=Pitch;
    i++;
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
