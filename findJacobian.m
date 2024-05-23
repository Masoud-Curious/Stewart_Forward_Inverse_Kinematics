function Jacobian = findJacobian(platformParams, workspaceConfig, inputAngleMode)

% This function finds the inverse kinematics (actuators' length) for a parallel Stewart platform. 
% Inputs: 
% a. platformParams: it that has all data about the geometry.
% b. workspaceConfig: [X Y Z roll pitch yaw] for inputAngleMode = 1 or 2
%    workspaceConfig: [X Y Z s_x s_y s_z] for inputAngleMode = 3
% Note: Input angles are either Euler Angles or axis angle. 

% The Jacobian relates workspace velocities Xdot to jointspace velocities
% ldot as
% ldot = Jacobian * Xdot
numbOfActuators = platformParams.numbOfActuators;

%% Extract position
x = workspaceConfig(1);
y = workspaceConfig(2);
z = workspaceConfig(3);
positionVector = [x; y; z];

inputAngleVector = workspaceConfig(4:6);

rotationMatrix = Rot(inputAngleVector, inputAngleMode);

%% Platform Params
floorJointCoordinates = platformParams.floorJointCoordinates;
platformJointCoordinates = platformParams.platformJointCoordinates;
Jacobian = zeros(numbOfActuators,6);


for i = 1 : numbOfActuators

    movingPlatformJoint_i = positionVector + rotationMatrix * platformJointCoordinates(:,i); 
    R_b_i = movingPlatformJoint_i - positionVector;
    jointToJointVector_i = movingPlatformJoint_i - floorJointCoordinates(:,i);
    jointToJointUnitVector_i = jointToJointVector_i/norm(jointToJointVector_i);
    Jacobian(i,:) = [jointToJointUnitVector_i' (cross(R_b_i,jointToJointUnitVector_i))']; % tempMult is scalar and is multiplied by 
end
