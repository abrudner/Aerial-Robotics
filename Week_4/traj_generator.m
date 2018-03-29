function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.

%% Fill in your code here

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;

persistent coeffx coeffy coeffz  traj_time d0 waypoints0
if nargin > 2
    desired_state.pos = zeros(3,1);
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
    
    coeffx = getCoeff(waypoints(1,:)');
    coeffy = getCoeff(waypoints(2,:)');
    coeffz = getCoeff(waypoints(3,:)');
    
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2*sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    assignin('base','traj_time',traj_time);
    waypoints0 = waypoints;
    assignin('base','wp0',waypoints0);
else
    if(t>traj_time(end))
        t = traj_time(end)-0.0001;
    end
    
    t_index = find(traj_time>t,1)-1;
    t_index = max(t_index,1);
    scale = (t-traj_time(t_index))/d0(t_index);
    index = (t_index-1)*8 + 1: t_index*8;
    
    if (t==0)
        desired_state.pos = waypoints0(:,1);
    else
        t0 = polyT(8,0,scale)';
        desired_state.pos = [coeffx(index,:) coeffy(index,:) coeffz(index,:)]'*t0;
    end
    t1 = polyT(8,1,scale)';
    t2 = polyT(8,2,scale)';
    
    
    desired_state.vel = [coeffx(index,:) coeffy(index,:) coeffz(index,:)]'*t1 .* (1/d0(t_index));
    desired_state.acc = [coeffx(index,:) coeffy(index,:) coeffz(index,:)]'*t2 .* (1/d0(t_index)^2);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
    
end
end
%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

% persistent waypoints0 traj_time d0
% if nargin > 2
%     d = waypoints(:,2:end) - waypoints(:,1:end-1);
%     d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
%     traj_time = [0, cumsum(d0)];
%     waypoints0 = waypoints;
% else
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     t_index = find(traj_time >= t,1);
% 
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
%     if(t == 0)
%         desired_state.pos = waypoints0(:,1);
%     else
%         scale = t/d0(t_index-1);
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%     end
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
% end

%% OLD CODE

% if(t >= traj_time(end))
%         t = traj_time(end) - 0.0001;
%     end
%     t_index = find(traj_time>t,1) - 1;
%     
%     if(t_index == 0)
%         t_index = 1;
%     end
%     
%     if t_index >1
%         t = t-traj_time(t_index-1);
%     end
%     
%     if (t==0)
%         desired_state.pos = waypoints0(:,1);
%     else
%         
% %         desired_state.pos = (1-scale)*waypoints0(:,t_index) + scale*waypoints0(:,t_index);
%     end
%     scale = (t-traj_time(t_index))/d0(t_index);
%     t0 = polyT(8,0,scale)';
%     t1 = polyT(8,1,scale)';
%     t2 = polyT(8,2,scale)';
%     
%     index = (t_index-1)*8 + 1: t_index*8;
%     desired_state.pos = [coeffx(index,:) coeffy(index,:) coeffz(index,:)]'*t0;
%     desired_state.vel = [coeffx(index,:) coeffy(index,:) coeffz(index,:)]'*t1 .* (1/d0(t_index));
%     desired_state.acc = [coeffx(index,:) coeffy(index,:) coeffz(index,:)]'*t2 .* (1/d0(t_index)^2);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
