function [vel, t]=LFPB(theta, t_u, acc)
% theta = [theta0, theta_via, theta_final]
% t = [tdo1, tdo2]
th0 = theta(1); th1 = theta(2); th2 = theta(3);
tdo1 = t_u(1); tdo2 = t_u(2) - t_u(1);

tb0 = tdo1 - sqrt(tdo1^2 - 2 * (th1 - th0) / acc);
theta_vel12 = (th1 - th0) / (tdo1 - 0.5 * tb0);

tbf =  tdo2 - sqrt(tdo2^2 + 2 * (th2 - th1) / -acc);
theta_vel23 = (th2 - th1) / (tdo2 - 0.5 * tbf);

tb1 = (theta_vel23 - theta_vel12) / acc;

tmf = tdo2 - tbf - 0.5 * tb1;
to1 =  tdo1 - tb0 - 0.5 * tb1;

vel = [theta_vel12, theta_vel23];
t = [tb0, to1, tb1, tmf, tbf];
end