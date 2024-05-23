function workspaceConfig  = forwardKinematics(platformParams, actuatorsLengthList, workspaceConfig,...
    inputAngleMode, numIteration)
% This function finds the forward kinematics of Stewart Platform using
% Newton-Gauss Method. The gradient for this method is the Jacobina of the
% platform. 
% Inputs: 
% a. platformParams: a structure of geometrical data of the platform
% b. actuatorsLengthList: contains all actuators lengths
% c. workspaceConfig (in R6): the first three entries are [X Y Z] but the last three enteries are either in Euler notation or screw axis notation.  
% d. inputAngleMode: shows either Euler notation in fixed or rotating frame
% (1 or 2) or if equals 3 represents screw axis notation   
% e. numIteration: A predefined number of iterations 

% We compute thecode for pseudo-inverse of the Jacobian. This way, the code
% can also be used for overactuated platforms. 

for i = 1: numIteration
    %% Compute Jacobian and Inverse Kinematics

    actuatorLengths = inverseKinematics(platformParams, workspaceConfig, inputAngleMode);
    Jacobian = findJacobian(platformParams, workspaceConfig, inputAngleMode);
    JT_J = Jacobian' * Jacobian;

    jointError = actuatorsLengthList - actuatorLengths;
    JT_jointError = Jacobian' * jointError;
    
    %% Find Inverse Jacobian: Using Schure's Complement for Block inverse
    Jacob_A = JT_J(1:3,1:3);
    Jacob_B = JT_J(1:3,4:6);
    Jacob_C = JT_J(4:6,1:3);
    Jacob_D = JT_J(4:6,4:6);
    
    Jacob_A_inv = invanddet3by3(Jacob_A);
    Jacob_C_A_inv = Jacob_C * Jacob_A_inv;
    Jacob_X = Jacob_D - Jacob_C_A_inv * Jacob_B;
    Jacob_X_inv = invanddet3by3(Jacob_X); 
    Jacob_A_inv_B = Jacob_A_inv * Jacob_B;

    Jacob_Y = Jacob_A_inv_B * Jacob_X_inv;
    Jacob_Z = Jacob_X_inv * Jacob_C_A_inv;

    J_inv_11 = Jacob_A_inv + Jacob_A_inv_B * Jacob_Z;
    J_inv_12 = -Jacob_Y;
    J_inv_21 = -Jacob_Z;
    J_inv_22 = Jacob_X_inv;
    J_inv = [J_inv_11 J_inv_12;J_inv_21 J_inv_22];

    
    %%
    del_Workspace = J_inv* JT_jointError;
    workspaceConfig = workspaceConfig + del_Workspace;
    
end