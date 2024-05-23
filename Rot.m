
function R = Rot(inputAngleData,inputAngleMode)
% This function obtains the rotation matrix accroding to the input mode
% defined by the user. 
% a. inputAngleMode = 1: Euler angles such that R = R_z * R_y * R_x
% b. inputAngleMode = 2: Euler angles such that R = R_x * R_y * R_z
% c. inputAngleMode = 3: screw axis where the angle equals
% norm(inputAngleData)

if inputAngleMode ~= 3

    roll = inputAngleData(1);
    pitch = inputAngleData(2);
    yaw = inputAngleData(3);

    R_x = [1 0 0;
            0 cos(roll) -sin(roll);
            0 sin(roll) cos(roll)];
    R_y = [cos(pitch) 0 sin(pitch);
        0 1 0
        -sin(pitch) 0 cos(pitch)];
    R_z = [cos(yaw) -sin(yaw) 0;
        sin(yaw) cos(yaw) 0;
        0 0 1];
   

    if inputAngleMode == 1
        R = R_z* R_y * R_x;
    else
        R = R_x * R_y * R_z;
    end

else
    % Screw axis format 
    theta = norm(inputAngleData);
    if abs(theta) > 0.0001
        sx = inputAngleData(1)/theta;
        sy = inputAngleData(2)/theta;
        sz = inputAngleData(3)/theta;
    
     
        cTheta = cos(theta);
        sTheta = sin(theta);
        vTheta = 1 - cos(theta);
        
        R(1,1) = sx^2 * vTheta + cTheta;
        R(2,1) = sx * sy * vTheta + sz * sTheta;
        R(3,1) = sx * sz * vTheta - sy * sTheta;
    
        R(1,2) = sx * sy * vTheta - sz * sTheta;
        R(2,2) = sy^2 * vTheta + cTheta;
        R(3,2) = sz * sy * vTheta + sx * sTheta;
    
        R(1,3) = sx * sz * vTheta + sy * sTheta;
        R(2,3) = sz * sy * vTheta - sx * sTheta;
        R(3,3) = sz^2 * vTheta + cTheta;
    else
        R = eye(3);
    end
    
end