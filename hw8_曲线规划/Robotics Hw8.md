## Robotics Hw8

> 姓名：张天霖
>
> 学号：21S153130

#### 1. T1

梯形速度规划(LFBS)曲线结果如图1所示, 其推导过程详见附录2， 其Matlab程序详见附录1 ‘HW8.m_LFPB节’。

<center>
    <img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);" 
    src="D:/大学/研究生课程/Robotics/Hw8/T1.svg">
    <br>
    <div style="color:orange; border-bottom: 1px solid #d9d9d9;
    display: inline-block;
    color: #999;
    padding: 2px;">图1 梯形速度规划曲线结果</div>
</center>

#### 2. T2

三次多项式规划(Cubic)曲线结果如图2所示,  其Matlab程序详见附录1 ‘HW8.m_Cubic节’。

<center>
    <img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);" 
    src="D:/大学/研究生课程/Robotics/Hw8/T2.svg">
    <br>
    <div style="color:orange; border-bottom: 1px solid #d9d9d9;
    display: inline-block;
    color: #999;
    padding: 2px;">图2 三次多项式规划曲线结果</div>
</center>

## 附录

#### 1.

```matlab
Hw8.m
%% LFPB
clc; clear;
theta = [5, 15, 40];
t_u = [1, 2];
acc = 80;

[vel, t_p]=LFPB(theta, t_u, acc);

t1 = 0:0.01:t_u(end);
th_p = []; th_vel = []; th_acc = [];

for t1 = 0:0.001:t_u(end);
    if t1 <= sum(t_p(:,1))
        temp = 0.5 * acc * t1^2 + theta(1);
%         temp_vel =  acc * t^2 ;
        
        th_p = [th_p, temp];
%         th_vel = [th_vel, temp_vel];
        
        th0_end =  th_p(end);

   elseif  t1 <= sum(t_p(:,1:2))
        temp = th0_end + vel(1) * (t1 - sum(t_p(:,1)));
%         temp_vel =   vel(1) ;
         
        th_p = [th_p, temp];
%         th_vel = [th_vel, temp_vel];
        
        th1_end =  th_p(end);
   elseif  t1 <= sum(t_p(:,1:3))
        temp = th1_end + vel(1) * (t1 - sum(t_p(:,1:2))) + 0.5 * acc * (t1 - sum(t_p(:,1:2)))^2;
%         temp_vel =   vel(1) +  acc * (t - sum(t_p(:,1:2)));
        
        th_p = [th_p, temp];
%         th_vel = [th_vel, temp_vel];
        th2_end =  th_p(end);
   elseif  t1 <= sum(t_p(:,1:4))
        temp = th2_end + vel(2) * (t1 - sum(t_p(:,1:3)));
%         temp_vel =  vel(2);
         
        th_p = [th_p, temp];
%         th_vel = [th_vel, temp_vel];
        th3_end =  th_p(end);
   elseif  t1 <= sum(t_p(:,1:5))
        temp = theta(end) - 0.5 * acc * (sum(t_p) - t1)^2;
%         temp_vel =  vel(2) - acc * (sum(t_p) - t);
        
        th_p = [th_p, temp];
%         th_vel = [th_vel, temp_vel];
        th4_end =  th_p(end);
    end
end

t1 = 0:0.001:t_u(end);
th_vel = diff(th_p) * 1000; th_vel = [th_vel, th_vel(end)];
th_vel(134) = th_vel(133); th_vel(874) = th_vel(873);
th_vel(1127) = th_vel(1126); th_vel(1613) = th_vel(1612);

th_acc = diff(th_vel) * 1000; th_acc = [th_acc, th_acc(end)];
th_acc(134) = th_acc(133); th_acc(874) = th_acc(873);
th_acc(1127) = th_acc(1126); th_acc(1613) = th_acc(1612);

figure;
subplot(3,1,1); 
plot(t1, th_p);
xlabel('t / s');
ylabel('degree');
title('theta position');

subplot(3,1,2);
plot(t1, th_vel);
title('theta vel');
xlabel('t / s');
ylabel('degree / s');

subplot(3,1,3);
plot(t1, th_acc);
title('theta acc');
xlabel('t / s');
ylabel('degree / s^2');

%% Cubic
clc; clear;
th = [5, 15];
vel = [0, 0];
tf = 2;

[a0, a1, a2, a3] = Cubic(th, vel, tf);

t1 = 0: 0.001: tf;
th1 = a0+ a1 * t1 + a2 * t1.^2 + a3 * t1.^3;

th = [15, -10];
vel = [0, 0];
[a0, a1, a2, a3] = Cubic(th, vel, tf);

t2 = tf: 0.001: 2 * tf;
th2 = a0+ a1 * (t2 - tf)  + a2 * (t2 - tf).^2 + a3 * (t2 - tf).^3;

t = [t1, t2];
theta = [th1, th2];
th_vel = diff(theta) * 1000;  th_vel = [th_vel, th_vel(end)];
th_acc = diff(th_vel) * 1000; th_acc = [th_acc, th_acc(end)];
th_acc(2000) = th_acc(1999); 

figure;
subplot(3,1,1); 
plot(t, theta);
xlabel('t / s');
ylabel('degree');
title('theta position');

subplot(3,1,2);
plot(t, th_vel);
title('theta vel');
xlabel('t / s');
ylabel('degree / s');

subplot(3,1,3);
plot(t, th_acc);
title('theta acc');
xlabel('t / s');
ylabel('degree / s^2');
```

```matlab
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
```

```matlab
function [a0, a1, a2, a3] = Cubic(th, vel, tf)
% th = [th0, thf]
%  vel = [vel0, velf]
th0 = th(1); thf = th(2);
vel0 = vel(1); velf = vel(2);

a0 = th0;
a1 = vel0;
a2 = 3 * (thf - th0) / (tf ^ 2) - 2 * vel0 / tf - velf / tf;
a3 = -2 * (thf - th0) / (tf ^ 3) + (velf + vel0) / (tf ^ 2);

end
```

