# Posture_Control_of_Big_Mars_Rover

## Overview
This project proposed a posture control method for an omnidirectional mobile robot with integrated active suspension Big Mars Rover, to enhance its mobility and stability while crossing uneven terrains and obstacles.

For more details, please refer to our article: [Attitude Control of an All-Wheel-Drive Rover with Integrated Active Suspension System](https://ieeexplore.ieee.org/document/10106596)

## Robot and Model
Big Mars Rover is a 4-wheel omnidirectional mobile robot with active suspension system, as shown in the figure below. The active suspension is composed of 4 linear actuator which could be controlled separately to modify the posture of robot. 
![alt text](https://github.com/Lynx1129/Posture_Control_of_Big_Mars_Rover/blob/main/Images/Rover%E5%AE%9E%E8%BD%A6.png = 250*250 "BMR1")

For simplicity and fluency of computation in simulation software, we used simplied model in webots where critical parameters have been reserved. The simplified model is shown in the figure below. 
![alt text](https://github.com/Lynx1129/Posture_Control_of_Big_Mars_Rover/blob/main/Images/Rover_Sensor.png "BMR2")

## Expriment
BMR model was used to conduct a comparison experiment on complex terrain under different attitude control strategies in Webots simulation platform, so as to verify the feasibility of different attitude optimization methods.  
![alt text](https://github.com/Lynx1129/Posture_Control_of_Big_Mars_Rover/blob/main/Images/Rover%E8%BF%87%E5%B4%8E%E5%B2%96%E8%B7%AF%E9%9D%A2.png "experiment")


