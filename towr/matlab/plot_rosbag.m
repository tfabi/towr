%% Plot towr rosbags in matlab
%
% The bags can be generated using
% towr_ros/src/exe/rosbag_geom_msg_extractor.cc.
%
% Author: Alexander Winkler

clc;
clear all;
close all;

%% Extract the desired 3D vectors from the bag
filePath = '~/Documents/thesis/extracted bags/matlab_universal_flat_97_58_swingx3.bag';
bag_all = rosbag(filePath);

t0 = 0.0; %bag_all.StartTime;
T  = bag_all.EndTime;

selectOptions = {'Time', [t0 T] };
bag = select(bag_all, selectOptions{:});

% base motion
bag_base_pose = select(bag, 'Topic', 'base_pose');
ts_base_pos = timeseries(bag_base_pose, 'Position.X', 'Position.Y', 'Position.Z');

bag_base_acc  = select(bag, 'Topic', 'base_acc');
ts_base_acc = timeseries(bag_base_acc, 'Z');

% endeffector motion
bag_foot_0 = select(bag, 'Topic', 'foot_pos_0');
ts_foot_0 = timeseries(bag_foot_0, 'X', 'Y', 'Z');

bag_foot_1 = select(bag, 'Topic', 'foot_pos_1');
ts_foot_1 = timeseries(bag_foot_1, 'X', 'Y', 'Z');

bag_foot_2 = select(bag, 'Topic', 'foot_pos_2');
ts_foot_2 = timeseries(bag_foot_2, 'X', 'Y','Z');

bag_foot_3 = select(bag, 'Topic', 'foot_pos_3');
ts_foot_3 = timeseries(bag_foot_3, 'X', 'Y','Z');

% endeffector forces
bag_force_0 = select(bag, 'Topic', 'foot_force_0');
ts_force_0 = timeseries(bag_force_0, 'X', 'Y','Z');

bag_force_1 = select(bag, 'Topic', 'foot_force_1');
ts_force_1  = timeseries(bag_force_1, 'X', 'Y','Z');

bag_force_2 = select(bag, 'Topic', 'foot_force_2');
ts_force_2  = timeseries(bag_force_2, 'X', 'Y','Z');

bag_force_3 = select(bag, 'Topic', 'foot_force_3');
ts_force_3  = timeseries(bag_force_3, 'X', 'Y','Z');

ts_force_total_0 = sqrt(ts_force_0.Data(:,1).^2 + ts_force_0.Data(:,2).^2 +ts_force_0.Data(:,3).^2 );
ts_force_total_1 = sqrt(ts_force_1.Data(:,1).^2 + ts_force_1.Data(:,2).^2 +ts_force_1.Data(:,3).^2 );
ts_force_total_2 = sqrt(ts_force_2.Data(:,1).^2 + ts_force_2.Data(:,2).^2 +ts_force_2.Data(:,3).^2 );
ts_force_total_3 = sqrt(ts_force_3.Data(:,1).^2 + ts_force_3.Data(:,2).^2 +ts_force_3.Data(:,3).^2 );

%% get phase status on or off

for i = 1:length(ts_force_0)
    if (bag_force_0(i) <= 0)
        phase(i) = 0;
    else
        phase(i) = 1; %in contact with ground
    end
end

%% define the plotting range and other additional quantities
t = ts_base_pos.Time; 

% base motion
base_x   = ts_base_pos.Data(:,1);
base_y   = ts_base_pos.Data(:,2);
base_z   = ts_base_pos.Data(:,3);

dt = t(2)-t(1);
for i = 1:length(base_x)-1
  base_vel_x(i+1) = (base_x(i+1) - base_x(i))/dt;
  base_vel_y(i+1) = (base_y(i+1) - base_y(i))/dt; 
  base_vel_z(i+1) = (base_z(i+1) - base_z(i))/dt; 
end 

% base acceleration
base_zdd = ts_base_acc.Data(:,1);

% foot motion
foot_0_x = ts_foot_0.Data(:,1);
foot_1_x = ts_foot_1.Data(:,1);
foot_2_x = ts_foot_2.Data(:,1);
foot_3_x = ts_foot_3.Data(:,1);

foot_0_y = ts_foot_0.Data(:,2);
foot_1_y = ts_foot_1.Data(:,2);
foot_2_y = ts_foot_2.Data(:,2);
foot_3_y = ts_foot_3.Data(:,2);

foot_0_z = ts_foot_0.Data(:,3);
foot_1_z = ts_foot_1.Data(:,3);
foot_2_z = ts_foot_2.Data(:,3);
foot_3_z = ts_foot_3.Data(:,3);

% foot force
force_0_x = ts_force_0.Data(:,1);
force_1_x = ts_force_1.Data(:,1);
force_2_x = ts_force_2.Data(:,1);
force_3_x = ts_force_3.Data(:,1);

force_0_y = ts_foot_0.Data(:,2);
force_1_y = ts_foot_1.Data(:,2);
force_2_y = ts_foot_2.Data(:,2);
force_3_y = ts_foot_3.Data(:,2);

force_0_z = ts_force_0.Data(:,3);
force_1_z = ts_force_1.Data(:,3);
force_2_z = ts_force_2.Data(:,3);
force_3_z = ts_force_3.Data(:,3);


% calculate RMSE between base-z acceleration and what should be the 
% acceleration based on the forces and gravity.
m = 20;   % weight of the robot
g = 9.81; % gravity acceleration
F_ext = force_0_z + force_1_z;
base_zdd_dynamics = 1/m*F_ext - g;
% calculate Root mean square error
base_zdd_error = base_zdd_dynamics - base_zdd;
norm_sqare = norm(base_zdd_error)^2;
n = size(t,1); % number of sampled points
RMSE = sqrt(norm_sqare/n) 

% % intervall at which dynamic constraint is enforced
% dt_dyn = 0.2;
% 
% spline_durations_f0 = [0.25 0.45 0.34 0.35 0.63 0.13 0.50 0.12 0.56 0.40];
% spline_durations_f1 = [0.62 0.41 0.59 0.14 0.39 0.13 0.78 0.36 0.23 0.46];
% spline_durations_f2 = [0.25 0.45 0.34 0.35 0.63 0.13 0.50 0.12 0.56 0.40];
% spline_durations_f3 = [0.62 0.41 0.59 0.14 0.39 0.13 0.78 0.36 0.23 0.46];
% 
% % create vector with absolute timings
% num_phases   = size(spline_durations_f0, 2);
% dt_foot_0    = spline_durations_f0;
% dt_foot_1    = spline_durations_f1;
% dt_foot_2    = spline_durations_f2;
% dt_foot_3    = spline_durations_f3;
% 
% for c = 2:num_phases
%   dt_foot_0(1,c) = dt_foot_0(c-1) + spline_durations_f0(c);
%   dt_foot_1(1,c) = dt_foot_1(c-1) + spline_durations_f1(c);
%   dt_foot_2(1,c) = dt_foot_2(c-1) + spline_durations_f2(c);
%   dt_foot_3(1,c) = dt_foot_3(c-1) + spline_durations_f3(c);
% end