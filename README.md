# Adaptive_Sliding_Mode_Control_of_Aerial_Manipulator
Adaptive Sliding Mode Control

This code derives the full dynamic model and control of Aerial Manipulator in Euler Lagrangian form. 

The reference for the dynamics and control law are below: 
1. S. Kim, S. Choi, and H. J. Kim, Aerial manipulation using a quadrotor with a two DOF robotic arm, in Proc.
IEEE/RSJ Int. Conf. Intell. Robots Syst., Nov. 2013, pp. 49904995.

Please run 
```
Script_AdSMC.m
```

Note this files calls the relevant simulink file. Go through the steps inside to realise what's happening inside the code. 

For necssary dynamics and kinematics please review my paper also: 
A very similar control and dynamics can be found in my paper: Thapa S., Bai H. and Acosta J.A. Cooperative Aerial Load Transport with Force Control. IFAC Workshop on Networked & Autonomous Air & Space Systems, June, 2018

## Results 
Aerial Maniulator 
![](https://github.com/sandeshthapa/Adaptive_Sliding_Mode_Control_of_Aerial_Manipulator/blob/master/uav_arm.jpg)

### Adaptive Sliding Mode Controller
![](https://github.com/sandeshthapa/Adaptive_Sliding_Mode_Control_of_Aerial_Manipulator/blob/master/asmc.jpg)

### Postion History 
![](https://github.com/sandeshthapa/Adaptive_Sliding_Mode_Control_of_Aerial_Manipulator/blob/master/pos_history.jpg)

### Attitude History 
![](https://github.com/sandeshthapa/Adaptive_Sliding_Mode_Control_of_Aerial_Manipulator/blob/master/attitude_history.jpg)


