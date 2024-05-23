clc
clear
format shortG%% Platform Params

platformParams = setPlatformGeometry();

%% Sim Params
inputAngleMode = 3;
numIteration = 3;

workspaceConfig = [0 0 250 0 0 5*pi/180]';
actuatorLengths = inverseKinematics(platformParams, workspaceConfig, inputAngleMode);

%
%% Forward Kinematics
jointsVal_Current = actuatorLengths;
workspaceConfig0 = [-3 0 205 5*pi/180 0 0]'; % Initial Guess for forward kinematics

Jacobian = findJacobian(platformParams, workspaceConfig, inputAngleMode);

%
inputAngleMode = 3;
workspaceConfigA  = forwardKinematics(platformParams, jointsVal_Current, workspaceConfig0,...
    inputAngleMode, numIteration);

workspaceConfigA'
%}

