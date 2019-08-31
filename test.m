clear all;

syms x y z real; 
syms x_dot y_dot z_dot real
syms psi theta phi real; % yaw(psi)-pitch(theta)-roll(phi) zyx
syms psi_dot theta_dot phi_dot real; %  angle rates
syms n1 n2 real % angle of joints 
syms n1_dot n2_dot real %rate of angle 
syms m_1 Ix1 Iy1 Iz1 real %mass of link and interia 
syms m_2 Ix2 Iy2 Iz2 real %mass of link and interia 
syms m_b m_1 m_2 real % mass of UAV + 2 links
syms Ixx Iyy Izz real %Inertia of UAV 
syms l1 l2 real 

ksi = [x y z phi theta psi n1 n2]';
ksi_dot = [x_dot y_dot z_dot phi_dot theta_dot psi_dot n1_dot n2_dot]';

p_b = [x y z]'; %postion of UAV with respect to inertial frame
p_b_dot = [x_dot y_dot z_dot]';
An_b = [psi theta phi]'; %Euler angles used: yaw pitch roll 
An_dot = [psi_dot theta_dot phi_dot]';
n_b = [n1 n2]';  % joint angles 
n_dot = [n1_dot n2_dot]'; % joint angles rate 

%Rotatation angle
x1 = [1 0 0]'; 
y1 = [0 1 0]';
z1 = [0 0 1]';
% Rotation matrix 
R_b = expm(hat(z1)*psi)*expm(hat(y1)*theta)*expm(hat(x1)*phi);

matlabFunction(R_b,'file','myR_b.m','vars', [m_b,m_1,m_2,Ixx,Iyy,Izz,Ix1,Iy1, Iz1, Ix2,Iy2,Iz2,l1,l2,x,y,z,phi,theta,psi,n1,n2,x_dot,y_dot,z_dot,phi_dot,theta_dot,psi_dot,n1_dot,n2_dot]);
