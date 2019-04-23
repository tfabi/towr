%% get hip positions

q = [ts_base_pos.Data(:,7) ts_base_pos.Data(:,4) ts_base_pos.Data(:,5) ts_base_pos.Data(:,6)]; %imaginary part

% hip position is sum of CoM position and nominal offset*rotation
for i = 1:length(q)
rotm = quat2rotm(q(i,:));
%rotated_hip position is the sum of the CoM position and the offset from
%CoM to each of the hips
rotated_hip_LF.position(i,:) = nom_hip.universal.LF*rotm + [ts_base_pos.Data(i,1) ts_base_pos.Data(i,2) ts_base_pos.Data(i,3)];
rotated_hip_LH.position(i,:) = nom_hip.universal.LH*rotm + [ts_base_pos.Data(i,1) ts_base_pos.Data(i,2) ts_base_pos.Data(i,3)];
rotated_hip_RF.position(i,:) = nom_hip.universal.RF*rotm + [ts_base_pos.Data(i,1) ts_base_pos.Data(i,2) ts_base_pos.Data(i,3)];
rotated_hip_RH.position(i,:) = nom_hip.universal.RH*rotm + [ts_base_pos.Data(i,1) ts_base_pos.Data(i,2) ts_base_pos.Data(i,3)];
end

%% get motion of ee relative to center of mass
% first get velocity of each hip in world frame then subtract this from ee_velocity in world frame to get
% relative vel of ee in body frame

for i=1:length(q)-1
    rotated_hip_LF.velocity(i+1,:) = (rotated_hip_LF.position(i+1,:)-rotated_hip_LF.position(i,:))/dt;
    rotated_hip_LH.velocity(i+1,:) = (rotated_hip_LH.position(i+1,:)-rotated_hip_LH.position(i,:))/dt;
    rotated_hip_RF.velocity(i+1,:) = (rotated_hip_RF.position(i+1,:)-rotated_hip_RF.position(i,:))/dt;
    rotated_hip_RH.velocity(i+1,:) = (rotated_hip_RH.position(i+1,:)-rotated_hip_RH.position(i,:))/dt;
end

foot_LF_bodyframe.velocity = foot_LF.velocity - rotated_hip_LF.velocity;
foot_RF_bodyframe.velocity = foot_RF.velocity - rotated_hip_RF.velocity;
foot_LH_bodyframe.velocity = foot_LH.velocity - rotated_hip_LH.velocity;
foot_RH_bodyframe.velocity = foot_RH.velocity - rotated_hip_RH.velocity;

