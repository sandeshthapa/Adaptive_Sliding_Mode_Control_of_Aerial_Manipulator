close all; clear all; clc; 

sim('Adaptive_SMC');
pos_e = sqrt(error(:,1).^2+ error(:,2).^2+error(:,3).^2);
% figure
subplot(2,2,1)
plot(time,pos_e,'b');
xlabel('time (sec)');
ylabel('position error (m)');
title('UAV position error norm')
% hold on 

orn_e = sqrt(error(:,4).^2+error(:,5).^2+error(:,6).^2);
subplot(2,2,2)
plot(time,orn_e,'b'); 
xlabel('time (sec)');
ylabel('orientation error(rad)');
title('orntn error norm')
% hold on 

man_e = sqrt(error(:,7).^2+error(:,8).^2);
subplot(2,2,3)
% figure 
plot(time,man_e,'b')
xlabel('time (sec)');
ylabel('ef pos error(m)');
title('Endeffector error norm')
% hold on 
% % 
% figure 
subplot(2,2,4)
plot(time,(q(:,3)),'b');
xlabel('t');
ylabel('z position (m)');
title('Z position of the rotor')

% hold on 
% figure
% plot(time,torque(:,1));
% legend('Torque 1');
% ylabel('Torque')
% xlabel('time')
% hold on 
% 
% figure
% plot(time,torque(:,2));
% legend('Torque 2')
% ylabel('Torque')
% xlabel('time')
% hold on
% 
% figure
% plot(time,torque(:,3));
% legend('Torque 3')
% ylabel('Torque')
% xlabel('time')
% hold on
% 
% figure
% plot(time,torque(:,4));
% legend('Torque 4')
% ylabel('Torque')
% xlabel('time')
% hold on
% 
% figure
% plot(time,error(:,5));
% legend('Torque 5')
% ylabel('Torque')
% xlabel('time')
% hold on
% 
% figure
% plot(time,torque(:,6));
% legend('Torque 6')
% ylabel('Torque')
% xlabel('time')
% hold on
% 
% figure
% plot(time,error(:,7));
% legend('Torque 7')
% ylabel('Torque')
% xlabel('time')
% hold on
% 
% figure
% plot(time,torque(:,8));
% legend('Torque 8')
% ylabel('Torque')
% xlabel('time')
% hold on
% 
% 
% figure
% plot(time,error(:,1));
% legend('error 1')
% ylabel('eror')
% xlabel('time')
% hold on
% 
% figure
% plot(time,error(:,2));
% legend('error 2')
% ylabel('eror')
% xlabel('time')
% 
% hold on
% 
% figure
% plot(time,error(:,3));
% legend('error 3')
% ylabel('eror')
% xlabel('time')
% hold on
% 
% figure
% plot(time,error(:,4));
% legend('error 4')
% ylabel('eror')
% xlabel('time')
% hold on
% 
% figure
% plot(time,error(:,5));
% legend('error 5')
% ylabel('eror')
% xlabel('time')
% hold on
% 
% figure
% plot(time,error(:,6));
% legend('error 6')
% ylabel('eror')
% xlabel('time')
% hold on
% 
% figure
% plot(time,error(:,7));
% legend('error 7')
% ylabel('eror')
% xlabel('time')
% hold on
% 
% figure
% plot(time,error(:,8));
% legend('error 8')
% ylabel('eror')
% xlabel('time')
% hold on
