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
filePath = '~/Documents/thesis/extracted bags/matlab_test_case.bag';
bag_all = rosbag(filePath);

t0 = 0.0; %bag_all.StartTime;
T  = bag_all.EndTime;

selectOptions = {'Time', [t0 T] };
bag = select(bag_all, selectOptions{:});

%% base motion
%Base position and orientation
bag_base_pose = select(bag, 'Topic', 'base_pose');
ts_base_pos = timeseries(bag_base_pose, 'Position.X', 'Position.Y', 'Position.Z', ... 
                                        'Orientation.X', 'Orientation.Y', 'Orientation.Z', 'Orientation.W');
%Base velocity                                    
bag_base_twist = select(bag, 'Topic', 'base_twist');
ts_base_twist = timeseries(bag_base_twist, 'Linear.X', 'Linear.Y', 'Linear.Z');

bag_base_acc  = select(bag, 'Topic', 'base_acc');
ts_base_acc = timeseries(bag_base_acc, 'Z');

%% endeffector motion
bag_foot_0 = select(bag, 'Topic', 'foot_pos_0');
ts_foot_0 = timeseries(bag_foot_0, 'X', 'Y', 'Z');

bag_foot_1 = select(bag, 'Topic', 'foot_pos_1');
ts_foot_1 = timeseries(bag_foot_1, 'X', 'Y', 'Z');

bag_foot_2 = select(bag, 'Topic', 'foot_pos_2');
ts_foot_2 = timeseries(bag_foot_2, 'X', 'Y','Z');

bag_foot_3 = select(bag, 'Topic', 'foot_pos_3');
ts_foot_3 = timeseries(bag_foot_3, 'X', 'Y','Z');

%% endeffector forces
bag_force_0 = select(bag, 'Topic', 'foot_force_0');
ts_force_0 = timeseries(bag_force_0, 'X', 'Y','Z');

bag_force_1 = select(bag, 'Topic', 'foot_force_1');
ts_force_1  = timeseries(bag_force_1, 'X', 'Y','Z');

bag_force_2 = select(bag, 'Topic', 'foot_force_2');
ts_force_2  = timeseries(bag_force_2, 'X', 'Y','Z');

bag_force_3 = select(bag, 'Topic', 'foot_force_3');
ts_force_3  = timeseries(bag_force_3, 'X', 'Y','Z');


%% define the plotting range and other additional quantities
t = ts_base_pos.Time; 

% base motion
% com_position = [ts_base_pos.Data(:,1) ts_base_pos.Data(:,2) ts_base_pos.Data(:,3)];

% dt = t(2)-t(1);
% for i = 1:length(com_position(:,:,1))-1
%   com_velocity(i+1, :) = (com_position(i+1, :)-com_position(i,:))/dt;
% 
% end 

% base acceleration
base_zdd = ts_base_acc.Data(:,1);

% foot motion
foot_0.position = [ts_foot_0.Data(:,1) ts_foot_0.Data(:,2) ts_foot_0.Data(:,3)];
foot_1.position = [ts_foot_1.Data(:,1) ts_foot_1.Data(:,2) ts_foot_1.Data(:,3)];
foot_2.position = [ts_foot_2.Data(:,1) ts_foot_2.Data(:,2) ts_foot_2.Data(:,3)];
foot_3.position = [ts_foot_3.Data(:,1) ts_foot_3.Data(:,2) ts_foot_3.Data(:,3)];

% foot force
foot_0.force = [ts_force_0.Data(:,1) ts_force_0.Data(:,2) ts_force_0.Data(:,3)];
foot_1.force = [ts_force_1.Data(:,1) ts_force_1.Data(:,2) ts_force_1.Data(:,3)];
foot_2.force = [ts_force_2.Data(:,1) ts_force_2.Data(:,2) ts_force_2.Data(:,3)];
foot_3.force = [ts_force_3.Data(:,1) ts_force_3.Data(:,2) ts_force_3.Data(:,3)];


%% error

m = 20.52;   % weight of the robot
g = 9.81; % gravity acceleration
F_ext = foot_0.force(:,3) + foot_1.force(:,3) + foot_2.force(:,3) + foot_3.force(:,3);
base_zdd_dynamics = 1/m*F_ext - g;
% calculate Root mean square error
base_zdd_error = base_zdd_dynamics - base_zdd;
norm_sqare = norm(base_zdd_error)^2;
n = size(t,1); % number of sampled points
RMSE = sqrt(norm_sqare/n) 
