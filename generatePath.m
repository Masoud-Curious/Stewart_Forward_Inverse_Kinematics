function [actuatorHistory, workspaceData,screwAxisList,ThetaList, angleReconstructionError] = generatePath(platformParams, simTime, flagIsCoordinateEulerian, flagIsFixedFrame)

%% Generate Workspace Data 
% We give some sinsoidial motion for workspace DoFs
f1 = 0.5;
f2 = 0.2;

xData = 0 * sin(2 * pi * f1 * simTime);
yData = 0 * cos(2 * pi * f2 * simTime);
zData = 0 + 0.1 * sin(2 * pi * 1 * simTime);

rollData = 35 * sin(2 * pi * f1 * simTime) * pi/180;
pitchData = 45 * sin(2 * pi * f2 * simTime) * pi/180;
yawData = 20 * sin(2 * pi * f1 * simTime) * pi/180;
workspaceData = [xData, yData, zData, rollData, pitchData, yawData];

for i = 1 : length(simTime)
    inputAngleData = [rollData(i); pitchData(i); yawData(i)];
    R1 = Rot(inputAngleData,flagIsCoordinateEulerian, flagIsFixedFrame);
    [screwAxisList(:,i),ThetaList(i)] = screwAxisConversion(R1, 1);
    
    %%
    
    R2 = Rot(screwAxisList(:,i),0, flagIsFixedFrame);
    [angleReconstructed,~] = screwAxisConversion(R2, 0);
    angleReconstructionError(:,i) = (inputAngleData - angleReconstructed) *180/pi;

    %norm(R2 - R1,'fro')
end

% s_x = 5 * sin(2 * pi * f1 * simTime) * pi/180;
% s_y = 5 * sin(2 * pi * f2 * simTime + 0.5*pi) * pi/180;
% s_z = 5 * cos(2 * pi * f1 * simTime) * pi/180;
% 
% workspaceData = [xData, yData, zData, s_x, s_y, s_z];


actuatorHistory = zeros(length(simTime),8);

for i = 1 : length(simTime)
    workspaceConfig = workspaceData(i,:);
    [~, ~, ~, ~, actuatorLengths, ~] = inverseKinematics(platformParams, workspaceConfig, ...
        flagIsCoordinateEulerian, flagIsFixedFrame);
    actuatorHistory(i,:) = actuatorLengths';

end
workspaceData = workspaceData';