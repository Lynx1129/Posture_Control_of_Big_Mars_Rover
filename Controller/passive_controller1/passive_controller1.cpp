// File:          passive_controller1.cpp
// Date:          2022.4.21
// Description:   适用于被动悬挂的rover
// Author:        Yin Siyuan
// Modifications: 

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
#include <webots/GPS.hpp>

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
using namespace std;

const double rad = 180.0/PI;
const double mass_body = 5.0;
const double mass_differential_lever = 0.2;
const double mass_fishbar = 0.1;
const double mass_bigside = 1.6;
const double mass_sidebar = 0.2;
const double mass_pullup = 0.18;
const double mass_pulldown = 0.18;
const double mass_steer = 2.8;
const double mass_wheel = 2.2;
const double mass_Rover = 31.64;
   
// length properties
const double length_sidebar = 220.0;
const double wheel_radius = 100.0;

// motor constraints
// const double min14 = 0;    //pullmotor1,4 min position
// const double max14 = 0.5;  //pullmotor1,4 max position
// const double min23 = -0.5; //pullmotor2,3 min position
// const double max23 = 0;    //pullmotor2,3 max position

// performance function parameter
const double k1 = 1;   // sum of turnover angles
const double k2 = 2;   //stability angle
const double k3 = 4; // pullmotor locomotion
const double k4 = 200;   // row pitch penalize
const double k5 = 40;   //theta penalize



 
const int timeStep = 32;  // get the time step of the current world.
int i=0; //simulation step counter
 
double Pitch,Roll,Yaw;  //the 3 values used to represent body pose

double ground_Clearance = 0; //ground clearance
double pull_Pos1,pull_Pos2,pull_Pos3,pull_Pos4; //position of pull motors
double beta1,beta2,beta3,beta4; //position of pull motors in formula
double bigside_Pos1, bigside_Pos2; //position of bigside
double theta1,theta2; //position of bigside in formula
double alpha1,alpha2,alpha3,alpha4; //wheel-ground contact angle
double position_x,position_y,position_z; // coordinate in world frame

//position properties
double com_x,com_y,com_z;
double mu1,mu2,mu3,mu4,stability_angle_deg; // stability angles;
double contact_force1,contact_force2,contact_force3,contact_force4;// wheel-ground contact force
std::array<double,3> gravity_normal; //gravity normalization in Rover coordinate
std::array<double,12> opt_para; //optimization equation initial condition parameter: 4 beta and 1 theta
std::array<double,9> opt_val; //optimization value

// performance index
double performance_index;

//data output txt
std::ofstream mycout("PA_data2.txt");
   



// vector normalization
std::array<double,3> vec_normalization(std::array<double,3> vector)
{
  double length = sqrt (pow(vector.at(0),2)+pow(vector.at(1),2)+pow(vector.at(2),2));
  std::array<double,3> vec_nor = {vector.at(0)/length,vector.at(1)/length,vector.at(2)/length};
  return vec_nor;
}


// vector addition
std::array<double,3> vec_add(std::array<double,3> vec1, std::array<double,3> vec2)
{
  std::array<double,3> vec3;
  vec3.at(0) = vec1.at(0) + vec2.at(0);
  vec3.at(1) = vec1.at(1) + vec2.at(1);
  vec3.at(2) = vec1.at(2) + vec2.at(2);
  return vec3;
}


// vector subtraction
std::array<double,3> vec_sub(std::array<double,3> vec1, std::array<double,3> vec2)
{
  std::array<double,3> vec3;
  vec3.at(0) = vec1.at(0) - vec2.at(0);
  vec3.at(1) = vec1.at(1) - vec2.at(1);
  vec3.at(2) = vec1.at(2) - vec2.at(2);
  return vec3;
}


// vector dot product
std::array<double,3> dot_product(std::array<double,3> vec1, std::array<double,3> vec2)
{
  std::array<double,3> vec3 = {vec1.at(0)*vec2.at(0),vec1.at(1)*vec2.at(1),vec1.at(2)*vec2.at(2)};
  return vec3;
}


//vector cross product
std::array<double,3> cross_product(std::array<double,3> vec1, std::array<double,3> vec2)
{
  std::array<double,3> vec3;
  vec3.at(0) = vec1.at(1)*vec2.at(2) - vec1.at(2)*vec2.at(1);
  vec3.at(1) = -(vec1.at(0)*vec2.at(2) - vec1.at(2)*vec2.at(0));
  vec3.at(2) = vec1.at(0)*vec2.at(1) - vec1.at(1)*vec2.at(0);
  return vec3;
}


// vector inner product
double inner_product(std::array<double,3> vec1, std::array<double,3> vec2)
{
  double result = vec1.at(0)*vec2.at(0) + vec1.at(1)*vec2.at(1) + vec1.at(2)*vec2.at(2);
  return result;
}


// vector rotation x axis
std::array<double,3> vec_rot_x(std::array<double,3> vector, double angle)
{
 double v1 = vector.at(0);
 double v2 = vector.at(1);
 double v3 = vector.at(2);
 double x1,x2,x3;
 double angle_rad = angle;
 x1 = v1;
 x2 = v2 * cos(angle_rad) - v3 * sin(angle_rad);
 x3 = v2 * sin(angle_rad) + v3 * cos(angle_rad);
 std::array<double,3> new_vec = {x1,x2,x3};
 return new_vec;
}


// vector rotation y axis
std::array<double,3> vec_rot_y(std::array<double,3> vector, double angle)
{
 double v1 = vector.at(0);
 double v2 = vector.at(1);
 double v3 = vector.at(2);
 double x1,x2,x3;
 double angle_rad = angle;
 x1 = v1 * cos(angle_rad) + v3 * sin(angle_rad);
 x2 = v2;
 x3 = -v1 * sin(angle_rad) + v3 * cos(angle_rad);
 std::array<double,3> new_vec = {x1,x2,x3};
 return new_vec;
}


// stability and performance evaluation
std::array<double,9> stability_eva(std::array<double,12> parameter, std::array<double,3> gravity)
{
   double beta1 = parameter.at(0); double beta2 = parameter.at(1); double beta3 = parameter.at(2); double beta4 = parameter.at(3);
   double alpha1 = parameter.at(4); double alpha2 = parameter.at(5); double alpha3 = parameter.at(6); double alpha4 = parameter.at(7);
   double theta1 = parameter.at(8); double theta2 = parameter.at(8);
   double Roll = parameter.at(9); double Pitch = parameter.at(10);
   double pull_Pos1 = beta1 - 0.2; double pull_Pos2 = beta2 + 0.2; double pull_Pos3 = beta3 + 0.2; double pull_Pos4 = beta4 - 0.2; 
   
  
   //position properties
   double com_x,com_y,com_z;
   double com[3]; //center of mass
   double contact_point1[3]; double contact_point2[3]; double contact_point3[3]; double contact_point4[3];
   double mu1,mu2,mu3,mu4,stability_angle,stability_angle_deg; // stability angles;
   std::array<double,3> center_of_mass; //center of mass (array)
   std::array<double,3> cp1,cp2,cp3,cp4; // contact point vector in Rover coordinate
   std::array<double,3> p1,p2,p3,p4; //contact point vector from center of mass
   std::array<double,3> A1,A2,A3,A4; //tipover axis
   std::array<double,3> a1,a2,a3,a4; //tipover axis normalization
   std::array<double,3> I1,I2,I3,I4; //tipover axis normals
   std::array<double,3> i1,i2,i3,i4; //tipover axis normals normalization  
   std::array<double,3> gravity_normal = gravity; //gravity normalization in Rover coordinate
      
   // performance index
   double performance_index;
 
    // evaluation of wheel-ground contact points
    // contact_point1
    contact_point1[0] = 56.6 * sin(45/rad -theta1) + length_sidebar * cos(theta1+beta1) 
    - 130*sin(theta1) - wheel_radius*cos(theta1)*cos(alpha1) - wheel_radius*sin(theta1)*sin(alpha1);    
 
    contact_point1[1] = 210.0;
    
    contact_point1[2] = 80 - 56.6 * cos(45/rad-theta1) - length_sidebar * sin(theta1+beta1)
    - 130*cos(theta1) + wheel_radius*sin(theta1)*cos(alpha1) - wheel_radius * cos(theta1) * sin(alpha1);
    
    for (int i = 0; i <= 2; i++)
    {
     contact_point1[i] = (round(10000.0 * contact_point1[i]))/10000.0;
    }
    
    // contact_point2
    contact_point2[0] = (-56.6) * sin(45/rad+theta1) - length_sidebar * cos(theta1+beta2)
    - 130*sin(theta1) - wheel_radius*cos(theta1)*cos(alpha2) - wheel_radius*sin(theta1)*sin(alpha2);
    
    contact_point2[1] = 210.0;
    
    contact_point2[2] = 80 - 56.6 * cos(45/rad+theta1) + length_sidebar * sin(theta1+beta2)
    -130*cos(theta1) + wheel_radius*sin(theta1)*cos(alpha2) - wheel_radius*cos(theta1)*sin(alpha2);
    
    for (int i = 0; i <= 2; i++)
    {
     contact_point2[i] = (round(10000.0 * contact_point2[i]))/10000.0;
    }    
    
    // contact_point3
    contact_point3[0] = 56.6 * sin(45/rad+theta2) + length_sidebar * cos(theta2+beta3) 
    +130*sin(theta2) - wheel_radius*cos(theta2)*cos(alpha3) - wheel_radius*sin(theta2)*sin(alpha3);
    
    contact_point3[1] = -210.0;
    
    contact_point3[2] = 80 - 56.6 * cos(theta2+45/rad) + length_sidebar * sin(beta3+theta2)
    -130*cos(theta2) + wheel_radius * sin(theta2) * cos(alpha3) - wheel_radius * cos(theta2) * sin(alpha3);
    
    for (int i = 0; i <= 2; i++)
    {
     contact_point3[i] = (round(10000.0 * contact_point3[i]))/10000.0;
    }     
    
    // contact_point4
    contact_point4[0] = (-56.6) * sin(45/rad-theta2) - length_sidebar * cos(theta2+beta4)
    + 130*sin(theta2) - wheel_radius*cos(theta2)*cos(alpha4) - wheel_radius*sin(theta2)*sin(alpha4);
    
    contact_point4[1] = -210.0;
    
    contact_point4[2] = 80 - 56.6 * cos(45/rad - theta2) - length_sidebar * sin(beta4+theta2)
    -130 * cos(theta2) + wheel_radius * sin(theta2) * cos(alpha4) - wheel_radius * cos(theta2) * sin(alpha4);
    
        for (int i = 0; i <= 2; i++)
    {
     contact_point4[i] = (round(10000.0 * contact_point4[i]))/10000.0;
    }
    
    
    
    // COM evaluation
    com_x = (mass_differential_lever * 150 + 2 * mass_fishbar * 75 
    + ((-mass_steer) * (56.6*sin(45/rad+theta1)+240*cos(theta1+beta2)) - mass_wheel * (56.6*sin(45/rad+theta1)+240*cos(theta1+beta2)-130*sin(theta1))
    +  mass_steer * (56.6*sin(45/rad-theta1)+240*sin(90/rad-theta1-beta1)) + mass_wheel * (56.6*sin(45/rad-theta1)+240*sin(90/rad-theta1-beta1)+130*sin(theta1)) )
    + (mass_steer * (56.6*sin(45/rad+theta2)+240*cos(theta2+beta3)) + mass_wheel * (56.6*sin(45/rad+theta2)+240*cos(theta2+beta3)-130*sin(theta2))
    -  mass_steer * (56.6*sin(45/rad-theta2)+240*sin(90/rad-theta2-beta4)) - mass_wheel * (56.6*sin(45/rad-theta2)+240*sin(90/rad-theta2-beta4)+130*sin(theta2)) )   
    )/mass_Rover;
    
    com_y = 0.000;
    
    com_z = (mass_body * 80 + mass_differential_lever * 180 + 2 * mass_fishbar * 180 + 2 * mass_bigside * 80
    + (mass_steer * (80-56.6*cos(45/rad-theta1)-220*sin(theta1+beta1)-10*cos(theta1)) + mass_wheel * (80-56.6*cos(45/rad-theta1)-220*sin(theta1+beta1)-10*cos(theta1)-120*cos(theta1))
    + (2*mass_sidebar + mass_pullup + mass_pulldown) * (80 - 56.6*cos(45/rad-theta1)-110*sin(theta1+beta1)+55)
    +  mass_steer * (80-56.6*cos(45/rad+theta1)+220*sin(theta1+beta2)+10*cos(theta1)) + mass_wheel * (80-56.6*cos(45/rad+theta1)+220*sin(theta1+beta2)+10*cos(theta1)+120*cos(theta1))
    + (2*mass_sidebar + mass_pullup + mass_pulldown) * (80 - 56.6*cos(45/rad+theta1)+110*sin(theta1+beta2)+55) ) 
    + (mass_steer * (80-56.6*cos(45/rad-theta2)-220*sin(theta2+beta4)-10*cos(theta2)) + mass_wheel * (80-56.6*cos(45/rad-theta2)-220*sin(theta2+beta4)-10*cos(theta2)-120*cos(theta2))
    + (2*mass_sidebar + mass_pullup + mass_pulldown) * (80 - 56.6*cos(45/rad-theta2)-110*sin(theta2+beta4)+55)
    +  mass_steer * (80-56.6*cos(45/rad+theta2)+220*sin(theta2+beta3)+10*cos(theta2)) + mass_wheel * (80-56.6*cos(45/rad+theta2)+220*sin(theta2+beta3)+10*cos(theta2)+120*cos(theta2))
    + (2*mass_sidebar + mass_pullup + mass_pulldown) * (80 - 56.6*cos(45/rad+theta2)+110*sin(theta2+beta3)+55) )
    )/mass_Rover;
    
    com[0] = com_x; com[1] = com_y; com[2] = com_z;
    for(int i = 0;i<=2;i++){com[i] = (round(10000.0 * com[i]))/10000.0; }
    for(int i = 0;i<=2;i++){center_of_mass.at(i) = com[i];}
    
    
    // Stability evaluation
    for(int i = 0; i<=2; i++)
    {cp1.at(i)=contact_point1[i]; cp4.at(i)=contact_point2[i]; cp3.at(i)=contact_point4[i]; cp2.at(i)=contact_point3[i];}
    p1 = vec_sub(cp1 , center_of_mass);p2 = vec_sub(cp2 , center_of_mass);p3 = vec_sub(cp3 , center_of_mass);p4 = vec_sub(cp4 , center_of_mass);
    
    A1 = vec_sub(p2,p1); a1 = vec_normalization(A1);
    A2 = vec_sub(p3,p2); a2 = vec_normalization(A2);
    A3 = vec_sub(p4,p3); a3 = vec_normalization(A3);
    A4 = vec_sub(p1,p4); a4 = vec_normalization(A4);
    
    I1.at(0) = p2.at(0)*(1-pow(a1.at(0),2))+p2.at(1)*(-a1.at(0)*a1.at(1))+p2.at(2)*(-a1.at(0)*a1.at(2));
    I1.at(1) = p2.at(0)*(-a1.at(0)*a1.at(1))+p2.at(1)*(1-pow(a1.at(1),2))+p2.at(2)*(-a1.at(1)*a1.at(2));
    I1.at(2) = p2.at(0)*(-a1.at(0)*a1.at(2))+p2.at(1)*(-a1.at(1)*a1.at(2))+p2.at(2)*(1-pow(a1.at(2),2));
    
    I2.at(0) = p3.at(0)*(1-pow(a2.at(0),2))+p3.at(1)*(-a2.at(0)*a2.at(1))+p3.at(2)*(-a2.at(0)*a2.at(2));
    I2.at(1) = p3.at(0)*(-a2.at(0)*a2.at(1))+p3.at(1)*(1-pow(a2.at(1),2))+p3.at(2)*(-a2.at(1)*a2.at(2));
    I2.at(2) = p3.at(0)*(-a2.at(0)*a2.at(2))+p3.at(1)*(-a2.at(1)*a2.at(2))+p3.at(2)*(1-pow(a2.at(2),2));
    
    I3.at(0) = p4.at(0)*(1-pow(a3.at(0),2))+p4.at(1)*(-a3.at(0)*a3.at(1))+p4.at(2)*(-a3.at(0)*a3.at(2));
    I3.at(1) = p4.at(0)*(-a3.at(0)*a3.at(1))+p4.at(1)*(1-pow(a3.at(1),2))+p4.at(2)*(-a3.at(1)*a3.at(2));
    I3.at(2) = p4.at(0)*(-a3.at(0)*a3.at(2))+p4.at(1)*(-a3.at(1)*a3.at(2))+p4.at(2)*(1-pow(a3.at(2),2));
    
    I4.at(0) = p1.at(0)*(1-pow(a4.at(0),2))+p1.at(1)*(-a4.at(0)*a4.at(1))+p1.at(2)*(-a4.at(0)*a4.at(2));
    I4.at(1) = p1.at(0)*(-a4.at(0)*a4.at(1))+p1.at(1)*(1-pow(a4.at(1),2))+p1.at(2)*(-a4.at(1)*a4.at(2));
    I4.at(2) = p1.at(0)*(-a4.at(0)*a4.at(2))+p1.at(1)*(-a4.at(1)*a4.at(2))+p1.at(2)*(1-pow(a4.at(2),2));
    
    i1 = vec_normalization(I1); i2 = vec_normalization(I2); i3 = vec_normalization(I3); i4 = vec_normalization(I4);
    

    
    // stability angles
    if(inner_product(cross_product(i1,gravity_normal),a1)<0){mu1 = acos(inner_product(gravity_normal,i1));}
    else{mu1 = -acos(inner_product(gravity_normal,i1));}
    if(inner_product(cross_product(i2,gravity_normal),a2)<0){mu2 = acos(inner_product(gravity_normal,i2));}
    else{mu2 = -acos(inner_product(gravity_normal,i2));}
    if(inner_product(cross_product(i3,gravity_normal),a3)<0){mu3 = acos(inner_product(gravity_normal,i3));}
    else{mu3 = -acos(inner_product(gravity_normal,i3));}
    if(inner_product(cross_product(i4,gravity_normal),a4)<0){mu4 = acos(inner_product(gravity_normal,i4));}
    else{mu4 = -acos(inner_product(gravity_normal,i4));}
    mu1 = (round(10000.0 * mu1))/10000.0;
    mu2 = (round(10000.0 * mu2))/10000.0;
    mu3 = (round(10000.0 * mu3))/10000.0;
    mu4 = (round(10000.0 * mu4))/10000.0;
    
    stability_angle = min(mu1,min(mu2,min(mu3,mu4)));
    stability_angle_deg = stability_angle * rad;
      
    //performance index evaluation
    performance_index = k1/mu1 + k1/mu2 + k1/mu3 + k1/mu4 + k2/stability_angle
    + k3 * (abs(pull_Pos1)+abs(pull_Pos2)+abs(pull_Pos3)+abs(pull_Pos4))
    + k4 * (abs(Roll)+abs(Pitch))
    + k5 * abs(theta1);

    // return values
    std::array<double,9> return_value;
    return_value.at(0) = com_x;
    return_value.at(1) = com_y;
    return_value.at(2) = com_z;
    return_value.at(3) = mu1;
    return_value.at(4) = mu2;
    return_value.at(5) = mu3;
    return_value.at(6) = mu4;
    return_value.at(7) = stability_angle_deg;
    return_value.at(8) = performance_index;
    
    return return_value;
}








// main function
int main(int argc, char **argv) {
  
  
  // Robot initialization
  Robot *robot = new Robot();


  InertialUnit *imu = robot -> getInertialUnit("imu");
  imu->enable(timeStep);
  
  // ground clearance sensor
  DistanceSensor *distance_sensor1 = robot -> getDistanceSensor("distance_sensor1");
  distance_sensor1->enable(timeStep);
  
  // pullmotor position sensor
  PositionSensor *pullsensor1 = robot -> getPositionSensor("pullsensor1");
  PositionSensor *pullsensor2 = robot -> getPositionSensor("pullsensor2");
  PositionSensor *pullsensor3 = robot -> getPositionSensor("pullsensor3");
  PositionSensor *pullsensor4 = robot -> getPositionSensor("pullsensor4");
  pullsensor1->enable(timeStep);
  pullsensor2->enable(timeStep);
  pullsensor3->enable(timeStep);
  pullsensor4->enable(timeStep);
  
  // bigside position sensor
  PositionSensor *bigsidesensor1 = robot -> getPositionSensor("bigsidesensor1");
  PositionSensor *bigsidesensor2 = robot -> getPositionSensor("bigsidesensor2");
  bigsidesensor1->enable(timeStep);
  bigsidesensor2->enable(timeStep);
  
  // wheel touch-3d sensor
  TouchSensor *wheelsensor1 = robot -> getTouchSensor("wheelsensor1");
  TouchSensor *wheelsensor2 = robot -> getTouchSensor("wheelsensor2");
  TouchSensor *wheelsensor3 = robot -> getTouchSensor("wheelsensor3");
  TouchSensor *wheelsensor4 = robot -> getTouchSensor("wheelsensor4");
  wheelsensor1->enable(timeStep);
  wheelsensor2->enable(timeStep);
  wheelsensor3->enable(timeStep);
  wheelsensor4->enable(timeStep);
  
  // wheel motor
  Motor *wheelmotor1 = robot->getMotor("wheelmotor1");
  Motor *wheelmotor2 = robot->getMotor("wheelmotor2");
  Motor *wheelmotor3 = robot->getMotor("wheelmotor3");
  Motor *wheelmotor4 = robot->getMotor("wheelmotor4");
  
  //GPS
  GPS *gps = robot->getGPS("gps");
  gps->enable(timeStep);



  
  // control loop
  while (robot->step(timeStep) != -1) {
   
    // obtain info from sensors
    // imu value
    const double *imu_value = imu->getRollPitchYaw();
    Pitch = imu_value[0];
    Roll = imu_value[1];
    Yaw = imu_value[2];  
    Pitch = (round(10000.0 * Pitch))/10000.0;
    Roll = (round(10000.0 * Roll))/10000.0;
    Yaw = (round(10000.0 * Yaw))/10000.0;
    
    // ground clearance value
    ground_Clearance = distance_sensor1->getValue();
    ground_Clearance = (round(5 * ground_Clearance))/10.0;
 
    // pull motor position value
    // note that pull_Pos is the relative position of height for each leg
    // beta is the absolute angle of the the pull motor joint
    pull_Pos1 = (pullsensor1 -> getValue()) - 4.91;
    pull_Pos2 = (pullsensor2 -> getValue()) + 0.2;
    pull_Pos3 = (pullsensor3 -> getValue()) + 0.2;
    pull_Pos4 = (pullsensor4 -> getValue()) - 0.2;
    pull_Pos1 = (round(1000.0 * pull_Pos1))/1000.0;
    pull_Pos2 = (round(1000.0 * pull_Pos2))/1000.0;
    pull_Pos3 = (round(1000.0 * pull_Pos3))/1000.0;
    pull_Pos4 = (round(1000.0 * pull_Pos4))/1000.0;
    beta1 = pullsensor1 -> getValue() - 4.71;
    beta2 = pullsensor2 -> getValue();
    beta3 = pullsensor3 -> getValue();
    beta4 = pullsensor4 -> getValue();
    
    // bigside position value
    bigside_Pos1 = bigsidesensor1 -> getValue();
    bigside_Pos2 = bigsidesensor2 -> getValue();
    bigside_Pos1 = (round(10000.0 * bigside_Pos1))/10000.0;
    bigside_Pos2 = (round(10000.0 * bigside_Pos2))/10000.0;
    theta1 = bigside_Pos1; theta2 = bigside_Pos2;
    
    //gps
    const double *gps_value = gps->getValues();
    position_x = gps_value[0];
    position_y = gps_value[1];
    position_z = gps_value[2];

    
    // wheel force-3d sensor value
    const double *wheelsensor1_value = wheelsensor1->getValues();
    const double *wheelsensor2_value = wheelsensor2->getValues();
    const double *wheelsensor3_value = wheelsensor3->getValues();
    const double *wheelsensor4_value = wheelsensor4->getValues();
     
    // wheel-ground contact angle
    alpha1 = atan(wheelsensor1_value[2]/wheelsensor1_value[0]);
    if(alpha1 < 0){alpha1 += 180/rad;}
    alpha2 = atan(wheelsensor2_value[2]/wheelsensor2_value[0]);
    if(alpha2 < 0){alpha2 += 180/rad;}
    alpha3 = atan(wheelsensor3_value[2]/wheelsensor3_value[0]);
    if(alpha3 < 0){alpha3 += 180/rad;}
    alpha4 = atan(wheelsensor4_value[2]/wheelsensor4_value[0]);
    if(alpha4 < 0){alpha4 += 180/rad;} 
    
    //wheel-ground contact force
    contact_force1 = sqrt(pow(wheelsensor1_value[0],2)+pow(wheelsensor1_value[1],2)+pow(wheelsensor1_value[2],2));
    contact_force2 = sqrt(pow(wheelsensor2_value[0],2)+pow(wheelsensor2_value[1],2)+pow(wheelsensor2_value[2],2));
    contact_force3 = sqrt(pow(wheelsensor3_value[0],2)+pow(wheelsensor3_value[1],2)+pow(wheelsensor3_value[2],2));
    contact_force4 = sqrt(pow(wheelsensor4_value[0],2)+pow(wheelsensor4_value[1],2)+pow(wheelsensor4_value[2],2));
        
    // gravity in Rover coordinate
    gravity_normal.at(0) = -(-cos(Pitch)*sin(Yaw)*sin(Roll)-sin(Pitch)*cos(Roll));
    gravity_normal.at(1) = cos(Yaw)*sin(Roll);
    gravity_normal.at(2) = sin(Pitch)*sin(Yaw)*sin(Roll)-cos(Pitch)*cos(Yaw);
    for(int i=0; i<=2;i++){gravity_normal.at(i) = (round(10000.0 * gravity_normal.at(i)))/10000.0;}
    
    // optimization equation parameter
    opt_para.at(0) = beta1;
    opt_para.at(1) = beta2;
    opt_para.at(2) = beta3;
    opt_para.at(3) = beta4;
    opt_para.at(4) = alpha1;
    opt_para.at(5) = alpha2;
    opt_para.at(6) = alpha3;
    opt_para.at(7) = alpha4;   
    opt_para.at(8) = theta1;
    opt_para.at(9) = Roll;
    opt_para.at(10) = Pitch;
    opt_para.at(11) = Yaw;
    
  
  
    // stability evaluation
    opt_val = stability_eva(opt_para,gravity_normal);
    com_x = opt_val.at(0);
    com_y = opt_val.at(1);
    com_z = opt_val.at(2);
    mu1 = opt_val.at(3); 
    mu2 = opt_val.at(4);
    mu3 = opt_val.at(5);
    mu4 = opt_val.at(6);
    stability_angle_deg = opt_val.at(7);
    performance_index = opt_val.at(8);
    
    
    
    
    // control the motors
    if(i<120){}
    else
    {
     wheelmotor1->setPosition(INFINITY);
     wheelmotor2->setPosition(INFINITY);
     wheelmotor3->setPosition(INFINITY);
     wheelmotor4->setPosition(INFINITY);
     wheelmotor1->setVelocity(1);
     wheelmotor2->setVelocity(1);
     wheelmotor3->setVelocity(-1);
     wheelmotor4->setVelocity(-1);     
    }
    
   
    // print info
    std::cout << "----------------------------------------info-------------------------------------------" << std::endl;
    std::cout << "Simulation Step is: " << i << "  (" << i*0.032 << "s)" << std::endl;
    std::cout << "Ground Clearance is: "<< "\t" << ground_Clearance << " (mm)" <<std::endl;
    std::cout << "Roll Pitch Yaw is:  [\t"  << Roll << "\t" << Pitch << "\t" << Yaw <<"] (rad)" << std::endl;
    std::cout << "Center of Mass is:  [\t" << com_x << "\t" << com_y << "\t" << com_z << "] (mm)" << std::endl;      
    std::cout << "g is: "<< gravity_normal.at(0)<< "\t" << gravity_normal.at(1)<< "\t" << gravity_normal.at(2) <<std::endl; 
    std::cout << "4 Stability angles are: \t" << mu1*rad << "\t" << mu2*rad << "\t" << mu3*rad << "\t" << mu4*rad << std::endl;
    std::cout << "Stability angle is: \t" << stability_angle_deg << " (deg)" << std::endl;
    std::cout << "Performance index is: \t" << performance_index << std::endl;
    std::cout << "Wheel-ground contact force is:\t " << contact_force1 << "\t"
     << contact_force2 << "\t" <<contact_force3 << "\t" <<contact_force4 << " (N)" << std::endl;
      std::cout << "World Position: \t" << position_x << "\t" << position_y << "\t" << position_z << std::endl;

    
    // data output
    mycout << i*0.032 << "\t\t" << performance_index << "\t\t" << stability_angle_deg << "\t\t" << Roll * rad  
    << "\t\t" << Pitch * rad << "\t\t" << Yaw * rad << "\t\t" << contact_force1 << "\t\t"
     << contact_force2 << "\t\t" <<contact_force3 << "\t\t" <<contact_force4 << "\t\t" << position_z << std::endl;
    if(i==2100){ delete robot; return 0;}
    
    i++;
  };

  // Enter here exit cleanup code.

}
