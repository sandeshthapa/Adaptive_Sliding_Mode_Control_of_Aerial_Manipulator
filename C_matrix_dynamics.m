
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

% linear velocity of UAV in intertial frame 
% p_b_dot = R_b*p_bb_dot; % not needed since position is in inertial frame

% R_b_dot =(hat(w_b)*R_b);
% now based on R and Rdot, calculate hat(w)
hat_w_b = ((diff(R_b,psi)*psi_dot+diff(R_b,theta)*theta_dot+diff(R_b,phi)*phi_dot)*R_b');
% now calculate the inertial angular velocity in terms of ypr angles and
% their rates
w_b = vee(hat_w_b);
% calculate the matrix A such that w = A*[psi_dot;theta_dot;phi_dot];
for i = 1:3
    T_b(i,1) = diff(w_b(i),psi_dot);
    T_b(i,2) = diff(w_b(i),theta_dot);
    T_b(i,3) = diff(w_b(i),phi_dot);
end
% compute the Jacobina matrix J such that [psi_dot;theta_dot;phi_dot] = Jw,
% i.e., J = inv(A);
% J = simplify(inv(T_b));

% In body frame angular velocity of UAV 
w_bb = R_b'*w_b;
% w_bb = (w_bb);
w_bb = simplify(w_bb);

Q = R_b'*T_b;
% Q = simplify(Q);

% Manipulator Forward Kinematics 
% function [R,p] = fwdkin_endeffector(q,type,H,P,n,PNT) 
% PNT is the final position of end effector 

% position of center of link 1 
h1 = [1 0 0]'; 
P10 = [0 0 0]';
p1c = [0 0 l1/2]';

R01 = expm((hat(h1))*n1);
p1c_b = P10 + R01*p1c;    % in base frame 
p1c_i = p_b + R_b*p1c_b;  % in inertial frame

% position of center of link 2 
h2 = [0 0 1]';
p12 = [0 0 l1]';
p2T = [0 l2 0]';
p2c = [0 l2/2 0]';

R12 = expm((hat(h2))*n2);
R02 = R01*R12;

p2c_b = P10 + R01*p1c + R02*p2c;   % in base frame
p2c_i = p_b + R_b*p2c_b;  % in inertial frame

% position of end effector 

p2_b = P10 + R01*p12 + R02*p2T;   % in base frame
p2_i = p_b + R_b*p2_b;  % in inertial frame

% Jacobians 
% Partial Jacobian for link 1 center 
% All joints are assumed to be prismatic
Jw1c = [h1 zeros(3,1)];
Jv1c = [hat(h1)*p1c_b zeros(3,1)];
J1c = [Jw1c; Jv1c];
% Jacobian for link 2 center 
Jw2c = [h1 h2];
Jv2c = [hat(h1)*(R01*p12) hat(h2)*p2c_b];
J2c = [Jw2c; Jv2c];

% J2c = [h1 h2; hat(h1)*(R01*p12) hat(h2)*p2c_b];
% Jacobian for end effector 
JT = [h1 h2; hat(h1)*R01*p12 hat(h2)*p2_b];
JwT = JT(1:3,1:2);
JvT = JT(4:6,1:2);

% Linear velocity of link COM 1-2 and end effector 
p1c_i_dot = p_b_dot + hat(w_b)*R_b*p1c_b + R_b*Jv1c*n_dot; % link 1 COM
p2c_i_dot = p_b_dot + hat(w_b)*R_b*p2c_b + R_b*Jv2c*n_dot; % link 2 COM
p2_i_dot = p_b_dot + hat(w_b)*R_b*p2_b + R_b*JvT*n_dot;    % end effector

% angular velocity 
w1c_i = w_b + R_b*Jw1c*n_dot;
w2c_i = w_b + R_b*Jw2c*n_dot;
w2_i = w_b + R_b*JwT*n_dot;

% Dynamic Model of Aerial Manipulator 
% Kinetic Energy of UAV 
I_b = [Ixx 0 0;0 Iyy 0;0 0 Izz]';
I_1 = [Ix1 0 0;0 Iy1 0;0 0 Iz1]';
I_2 = [Ix2 0 0;0 Iy2 0;0 0 Iz2]';

K_b = 1/2*m_b*(p_b_dot)'*p_b_dot   +   1/2*An_dot'*T_b'*R_b*I_b*R_b'*T_b*An_dot;

% KE of link
% I_1 = [Ix1 0 0;0 Iy1 0;0 0 Iz1]'; %link 1 moment of inertia 

K_1 = 1/2*m_1*(p1c_i_dot)'*p1c_i_dot  +   1/2*w1c_i'*R_b*R01*I_1*R01'*R_b'*w1c_i;
K_2 = 1/2*m_2*(p2c_i_dot)'*p2c_i_dot  +   1/2*w2c_i'*R_b*R02*I_2*R02'*R_b'*w2c_i;

% Inertia matrix B % Need to calculate the matrix 

M11 = (m_b + m_1 + m_2)*eye(3);

M22 = Q'*I_b*Q  +  m_1*T_b'*hat((R_b*p1c_b)')*hat(R_b*p1c_b)*T_b + Q'*R01*I_1*R01'*Q ...
        + m_2*T_b'*hat((R_b*p2c_b)')*hat(R_b*p2c_b)*T_b + Q'*R02*I_2*R02'*Q ;
    
M33 = m_1*(Jv1c)'*Jv1c + Jw1c'*R01*I_1*R01'*Jw1c ...
      + m_2*(Jv2c)'*Jv2c + Jw2c'*R02*I_2*R02'*Jw2c;
M12 = -(m_1*hat((R_b*p1c_b)')) -(m_2*hat((R_b*p2c_b)'));
M21 = M12';
M13 = m_1*R_b*Jv1c + m_2*R_b*Jv2c;
M31 = M13';
M23 = Q'*R01*I_1*R01'*Jw1c -m_1*T_b'*hat((R_b*p1c_b)')*R_b*Jv1c ...
      +Q'*R02*I_2*R02'*Jw2c -m_2*T_b'*hat((R_b*p2c_b)')*R_b*Jv2c ;
M32 = M23';

M1 = [M11,M12,M13];
M2 = [M21,M22,M23];
M3 = [M31,M32,M33];

M = [M1;M2;M3];

% Christoffel symbols methods
n =8;
for k = 1:n
    for j = 1:n
        for i = 1:n 
            Cc(i,j,k) = .5*((diff(M(k,j),ksi(i)))+ (diff(M(k,i),ksi(j)))...
                       -(diff(M(i,j),ksi(k))));
        end 
%         Ccd(k,j) = Cc(:,j,k)'*tdot;
    end
    C(k,:) = Cc(:,:,k)*ksi_dot;
end 

% ht1 = matlabFunction(C,'vars', [m_b,m_1,m_2,Ixx,Iyy,Izz,Ix1,Iy1, Iz1, Ix2,Iy2,Iz2,l1,l2,x,y,z,phi,theta,psi,n1,n2,x_dot,y_dot,z_dot,phi_dot,theta_dot,psi_dot,n1_dot,n2_dot]);
% matlabFunction(C,'file','C_Matrix_vars.m','vars', [m_b,m_1,m_2,Ixx,Iyy,Izz,Ix1,Iy1, Iz1, Ix2,Iy2,Iz2,l1,l2,x,y,z,phi,theta,psi,n1,n2,x_dot,y_dot,z_dot,phi_dot,theta_dot,psi_dot,n1_dot,n2_dot]);

% matlabFunction(C,'file','C_Matrix.m');

Ccdot = C*ksi_dot;

% PE term 
e3 = [0 0 1]';
g = 9.8;

Ub = m_b*g*e3'*p_b; % PE of UAV 
U_1 = m_1*g*e3'*(p_b + R_b*p1c_b); 
U_2 = m_2*g*e3'*(p_b + R_b*p2c_b); % PE of link 2 

Ut = Ub + U_1 + U_2;

for k = 1:n
    G = diff(Ut,ksi(k));
    G_a(k,:) = G;
end 
% Control 

temp1 = -hat(R_b*p2_b)*T_b;
temp2 = R_b*JvT;

J_a = [eye(3,3)  , zeros(3,3),    zeros(3,2); 
       zeros(3,3), T_b       ,    zeros(3,2);
       eye(2,3)  , temp1(1:2,1:3),temp2(1:2,1:2)]; 
   
for k = 1:n; 
    J_a_dot = diff(J_a,ksi(k));
end 


J_1 = [eye(3,3) -hat(R_b*p2_b)*T_b R_b*JvT];
