function platformParams = setPlatformGeometry()
% This function provides the geometrical params of a parallel Stewart
% Platform. All the parameters are put in platformParams structure

deg2rad = pi/180;

%% Platform Geometry Params
maxActuatorLength = 100;
actuatorBodyLength = 200 ; 

floorRadius = 250;
platformRadius = 150;
  
floorBaseAngles = [25 35 145 155 265 275] * deg2rad;
movingBaseAngles = [0 60 120 180 240 300] * deg2rad;
platformParams.numbOfActuators = length(floorBaseAngles);

platformParams.floorBaseAngles = floorBaseAngles;
platformParams.movingBaseAngles = movingBaseAngles;

platformParams.actuatorBodyLength = actuatorBodyLength;
platformParams.maxActuatorLength = maxActuatorLength;


floorJointCoordinates = floorRadius * [cos(floorBaseAngles);...
    sin(floorBaseAngles);zeros(1,length(floorBaseAngles))];
platformJointCoordinates = platformRadius * [cos(movingBaseAngles);...
    sin(movingBaseAngles);zeros(1,length(floorBaseAngles))];

platformParams.floorJointCoordinates = floorJointCoordinates;
platformParams.platformJointCoordinates = platformJointCoordinates;
