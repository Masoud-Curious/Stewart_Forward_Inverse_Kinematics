function actuatorLengths = inverseKinematics(platformParams, workspaceConfig, ...
    inputAngleMode)

% This function finds the inverse kinematics (actuators' length) for a parallel Stewart platform. 
% Inputs: 
% a. platformParams: it that has all data about the geometry.
% b. workspaceConfig: [X Y Z roll pitch yaw] for inputAngleMode = 1 or 2
%    workspaceConfig: [X Y Z s_x s_y s_z] for inputAngleMode = 3
% Note: Input angles are either Euler Angles or axis angle. 

% Tip on Formulation: Inverse Kinematic of Steart Platform is obtained by
% writing a loop equation for each actuator and then solving for the
% actuators length. 
% s_i: the unit vector of actuator i 
% R: Rotation Matrix
% P: Cartesian position of the center of the moving platform
% b_i: joint coordinate of ith actuator on the moving plaform
% a_i: joint coordinate of ith actuator on the ground
% Then we have: P + R * b_i = a_i + l_i * s_i 
% l_i = norm(P + R * b_i - a_i)

numbOfActuators = platformParams.numbOfActuators;
actuatorLengths = zeros(numbOfActuators,1);

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

actuatorBodyLength = platformParams.actuatorBodyLength;

for i = 1 : numbOfActuators
    movingPlatformJoint_i = positionVector + rotationMatrix * platformJointCoordinates(:,i);
    jointToJointVector = movingPlatformJoint_i - floorJointCoordinates(:,i);
    jointToJointLength_i = sqrt(jointToJointVector' * jointToJointVector);   
    actuatorLengths(i) =  jointToJointLength_i - actuatorBodyLength;
end
