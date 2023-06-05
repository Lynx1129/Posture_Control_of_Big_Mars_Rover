// File:          Rover_pid_pos.cpp
// Date:          2022.4.5
// Description:   PID test for rover
// Author:        Yin Siyuan
// Modifications: It implemented a PID controller for pull motor position

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/DistanceSensor.hpp>

#include <iostream>
#include <algorithm>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

 
  int timeStep = 32;  // get the time step of the current world.
  int i=0; //simulation step counter
  
  double P_Pitch = 30; //pid control parameter
  double I_Pitch = 0.1;
  double D_Pitch = 1;//pid control parameter
  double P_Roll = 30;
  double I_Roll = 0.1;
  double D_Roll = 1;
  double Pitch,Roll,Yaw;  //the 3 values used to represent body pose
  double last_Pitch = 0; double last_Roll = 0;
  double sum_Pitch = 0; double sum_Roll = 0;
  double ground_Clearance = 0; //ground clearance

  InertialUnit *imu = robot -> getInertialUnit("imu");
  imu->enable(timeStep);
  DistanceSensor *distance_sensor1 = robot -> getDistanceSensor("distance_sensor1");
  distance_sensor1->enable(timeStep);
  
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
    Pitch = imu_value[0];
    Roll = imu_value[1];
    Yaw = imu_value[2];
    
    Pitch = (round(10000.0 * Pitch))/10000.0;
    Roll = (round(10000.0 * Roll))/10000.0;
    Yaw = (round(10000.0 * Yaw))/10000.0;
    
    ground_Clearance = distance_sensor1->getValue();
    ground_Clearance = (round(50 * ground_Clearance))/1000.0;
 
       
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
     
     double pos_Pitch = P_Pitch * Pitch + D_Pitch * (Pitch-last_Pitch) + I_Pitch * sum_Pitch;
     double pos_Roll = P_Roll * Roll + D_Roll * (Roll-last_Roll) + I_Roll * sum_Roll;
     double pullmotor_pos1 = std::min (std::max( pos_Pitch - pos_Roll,-0.2), 0.2) + 4.91;
     double pullmotor_pos2 = std::min (std::max( pos_Pitch + pos_Roll,-0.2), 0.2) - 0.2;
     double pullmotor_pos3 = std::min (std::max( - pos_Pitch - pos_Roll,-0.2), 0.2) - 0.2;
     double pullmotor_pos4 = std::min (std::max( - pos_Pitch + pos_Roll,-0.2), 0.2) + 0.2;
     
     pullmotor1->setPosition(pullmotor_pos1);
     pullmotor2->setPosition(pullmotor_pos2);
     pullmotor3->setPosition(pullmotor_pos3);
     pullmotor4->setPosition(pullmotor_pos4);
    }
 
    std::cout << "Simulation step is: " << i << std::endl;
    std::cout << "Pitch Roll Yaw is:" << "\t" << Roll << "\t" << Pitch << "\t" << Yaw <<std::endl;
    std::cout << "Ground Clearance is: "<< "\t" << ground_Clearance << " cm" <<std::endl;
 
    last_Pitch = Pitch;
    last_Roll = Roll;
    sum_Pitch += Pitch;
    sum_Roll += Roll;
    i++;
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
