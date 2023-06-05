// File:          Rover_pid_vel2.cpp
// Date:          2022.4.5
// Description:   PID test for rover
// Author:        Yin Siyuan
// Modifications: PID controller for velocity & displacement 

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Display.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <math.h>
#include <array>

#include <time.h>
#include <ctime>
#include <algorithm>
#include <chrono>

#define PI 3.1415926

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int timeStep = 32;  // get the time step of the current world.
int i=0; //simulation step counter

//pid control parameter
double P_Pitch = 30; double I_Pitch = 0.1; double D_Pitch = 1;
double P_Roll = 30; double I_Roll = 0.1; double D_Roll = 1;
double P_Pos = 0.5; double I_Pos = 0; double D_Pos = 0.01;
  
double Pitch,Roll,Yaw;  //the 3 values used to represent body pose
double last_Pitch = 0; double last_Roll = 0;
double sum_Pitch = 0; double sum_Roll = 0;
double last_Pos1 = 0; double last_Pos2 = 0; double last_Pos3 = 0; double last_Pos4 = 0;
double sum_Pos1 = 0; double sum_Pos2 = 0; double sum_Pos3 = 0; double sum_Pos4 = 0;

double ground_Clearance = 0; //ground clearance
double pull_Pos1,pull_Pos2,pull_Pos3,pull_Pos4; //position of pull motors


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  InertialUnit *imu = robot -> getInertialUnit("imu");
  imu->enable(timeStep);
  
  DistanceSensor *distance_sensor1 = robot -> getDistanceSensor("distance_sensor1");
  distance_sensor1->enable(timeStep);
  
  PositionSensor *pullsensor1 = robot -> getPositionSensor("pullsensor1");
  PositionSensor *pullsensor2 = robot -> getPositionSensor("pullsensor2");
  PositionSensor *pullsensor3 = robot -> getPositionSensor("pullsensor3");
  PositionSensor *pullsensor4 = robot -> getPositionSensor("pullsensor4");
  pullsensor1->enable(timeStep);
  pullsensor2->enable(timeStep);
  pullsensor3->enable(timeStep);
  pullsensor4->enable(timeStep);
  
  Motor *wheelmotor1 = robot->getMotor("wheelmotor1");
  Motor *wheelmotor2 = robot->getMotor("wheelmotor2");
  Motor *wheelmotor3 = robot->getMotor("wheelmotor3");
  Motor *wheelmotor4 = robot->getMotor("wheelmotor4");
  Motor *pullmotor1 = robot->getMotor("pullmotor1");
  Motor *pullmotor2 = robot->getMotor("pullmotor2");
  Motor *pullmotor3 = robot->getMotor("pullmotor3");
  Motor *pullmotor4 = robot->getMotor("pullmotor4");
  
  // wheel touch-3d sensor
  TouchSensor *wheelsensor1 = robot -> getTouchSensor("wheelsensor1");
  TouchSensor *wheelsensor2 = robot -> getTouchSensor("wheelsensor2");
  TouchSensor *wheelsensor3 = robot -> getTouchSensor("wheelsensor3");
  TouchSensor *wheelsensor4 = robot -> getTouchSensor("wheelsensor4");
  wheelsensor1->enable(timeStep);
  wheelsensor2->enable(timeStep);
  wheelsensor3->enable(timeStep);
  wheelsensor4->enable(timeStep);
  
  // bigside position sensor
  PositionSensor *bigsidesensor1 = robot -> getPositionSensor("bigsidesensor1");
  PositionSensor *bigsidesensor2 = robot -> getPositionSensor("bigsidesensor2");
  bigsidesensor1->enable(timeStep);
  bigsidesensor2->enable(timeStep);


  while (robot->step(timeStep) != -1) {
  
    const double *imu_value = imu->getRollPitchYaw();
    Pitch = imu_value[0];
    Roll = imu_value[1];
    Yaw = imu_value[2];  
    Pitch = (round(1000.0 * Pitch))/1000.0;
    Roll = (round(1000.0 * Roll))/1000.0;
    Yaw = (round(1000.0 * Yaw))/1000.0;
    
    ground_Clearance = distance_sensor1->getValue();
    ground_Clearance = (round(50 * ground_Clearance))/1000.0;
 
    pull_Pos1 = (pullsensor1 -> getValue()) - 4.91;
    pull_Pos2 = (pullsensor2 -> getValue()) + 0.2;
    pull_Pos3 = (pullsensor3 -> getValue()) + 0.2;
    pull_Pos4 = (pullsensor4 -> getValue()) - 0.2;
    pull_Pos1 = (round(1000.0 * pull_Pos1))/1000.0;
    pull_Pos2 = (round(1000.0 * pull_Pos2))/1000.0;
    pull_Pos3 = (round(1000.0 * pull_Pos3))/1000.0;
    pull_Pos4 = (round(1000.0 * pull_Pos4))/1000.0;
       
    if(i<=500)
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
     wheelmotor1->setVelocity(0.8);
     wheelmotor2->setVelocity(0.8);
     wheelmotor3->setVelocity(-0.8);
     wheelmotor4->setVelocity(-0.8);
     
     pullmotor1->setPosition(INFINITY);
     pullmotor2->setPosition(INFINITY);
     pullmotor3->setPosition(INFINITY);
     pullmotor4->setPosition(INFINITY);
     
     double vel_Pitch = P_Pitch * Pitch + D_Pitch * (Pitch-last_Pitch) + I_Pitch * sum_Pitch;
     double vel_Roll = P_Roll * Roll + D_Roll * (Roll-last_Roll) + I_Roll * sum_Roll;
     double vel_Pos1 = P_Pos * pull_Pos1 + D_Pos * (pull_Pos1- last_Pos1) + I_Pos * sum_Pos1;
     double vel_Pos2 = P_Pos * pull_Pos2 + D_Pos * (pull_Pos2- last_Pos2) + I_Pos * sum_Pos2;
     double vel_Pos3 = P_Pos * pull_Pos3 + D_Pos * (pull_Pos3- last_Pos3) + I_Pos * sum_Pos3;
     double vel_Pos4 = P_Pos * pull_Pos4 + D_Pos * (pull_Pos4- last_Pos4) + I_Pos * sum_Pos4;
     
     double pullmotor_vel1 = std::min (std::max( vel_Pitch - vel_Roll - vel_Pos1,-0.2), 0.2);
     double pullmotor_vel2 = std::min (std::max( vel_Pitch + vel_Roll - vel_Pos2,-0.2), 0.2);
     double pullmotor_vel3 = std::min (std::max( - vel_Pitch - vel_Roll - vel_Pos3,-0.2), 0.2);
     double pullmotor_vel4 = std::min (std::max( - vel_Pitch + vel_Roll - vel_Pos4,-0.2), 0.2);
     
     pullmotor1->setVelocity(pullmotor_vel1);
     pullmotor2->setVelocity(pullmotor_vel2);
     pullmotor3->setVelocity(pullmotor_vel3);
     pullmotor4->setVelocity(pullmotor_vel4);
    }
 
    std::cout << "Simulation step is: " << i << std::endl;
    std::cout << "Pitch Roll Yaw is:" << "\t" << Roll << "\t" << Pitch << "\t" << Yaw <<std::endl;
    std::cout << "Ground Clearance is: "<< "\t" << ground_Clearance << " cm" <<std::endl;
    std::cout << "Pull Motor Position is:  " << pull_Pos1 << "\t" << -pull_Pos2 << "\t" << -pull_Pos3 << "\t" << pull_Pos4 << "\n" <<std::endl;
 
    //update the data
    last_Pitch = Pitch; last_Roll = Roll;
    sum_Pitch += Pitch; sum_Roll += Roll;
    last_Pos1 = pull_Pos1; last_Pos2 = pull_Pos2; last_Pos3 = pull_Pos3; last_Pos4 = pull_Pos4;
    sum_Pos1 += pull_Pos1; sum_Pos2 += pull_Pos2; sum_Pos3 += pull_Pos3; sum_Pos4 += pull_Pos4;
    
    i++;
  };

  // Enter here exit cleanup code.
  delete robot;
  return 0;
}
