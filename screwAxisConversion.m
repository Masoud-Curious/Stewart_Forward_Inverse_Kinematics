function [s,theta] = screwAxisConversion(R, conversionFlag)

if conversionFlag == 1
    sx_raw = R(3,2) - R(2,3);
    sy_raw = R(1,3) - R(3,1);
    sz_raw = R(2,1) - R(1,2);
    
    theta = acos(0.5 * (trace(R) - 1));
    s = [sx_raw;sy_raw;sz_raw]/(2 * sin(theta));
    s = s * theta;
else
    pitch = atan(-R(3,1)/sqrt(R(1,1)^2 + R(2,1)^2));
    yaw = atan(R(2,1)/R(1,1));
    roll = atan(R(3,2)/R(3,3));

    s = [roll; pitch; yaw];
    theta = [];
end