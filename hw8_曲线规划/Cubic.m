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