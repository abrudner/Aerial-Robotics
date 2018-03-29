function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% First Order Gains
kpx = 188;
kpy = 188;
kpz = 188;
kpphi = 3000;
kptht = 3000;
kppsi = 3000;

% Second Order Gains
kvx = 10;
kvy = 10;
kvz = 30;
kvphi = 50;
kvtht = 50;
kvpsi = 50;

% Desired Accelerations
r_dd_1_des = des_state.acc(1) + kvx*(des_state.vel(1) - state.vel(1)) + ...
    kpx*(des_state.pos(1) - state.pos(1));
r_dd_2_des = des_state.acc(2) + kvy*(des_state.vel(2) - state.vel(2)) + ...
    kpy*(des_state.pos(2) - state.pos(2));
r_dd_3_des = des_state.acc(3) + kvz*(des_state.vel(3) - state.vel(3)) + ...
    kpz*(des_state.pos(3) - state.pos(3));

% Desired Rotations
phi_des = (1/params.gravity)*(r_dd_1_des*sin(des_state.yaw) - r_dd_2_des*cos(...
    des_state.yaw));
tht_des = (1/params.gravity)*(r_dd_1_des*cos(des_state.yaw) + r_dd_2_des*sin(...
    des_state.yaw));
psi_des = des_state.yaw;

u1 = params.mass*(params.gravity + r_dd_3_des);
u2 = [kpphi*(phi_des - state.rot(1)) + kvphi*(-state.omega(1)); kptht*(...
    tht_des - state.rot(2)) + kvtht*(-state.omega(2)); kppsi*(...
    psi_des - state.rot(3)) + kvpsi*(des_state.yawdot - state.omega(3))];

% Thrust
F = u1;

% Moment
M = params.I*u2;

% =================== Your code ends here ===================

end
