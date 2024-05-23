function [ inverse, J, determinant, detInv] = invanddet3by3(A)
%INVANDDET3BY3 Calculates the determinant and the inverse of a 3 X 3 matrix by using 
% cofactors and adjoint matrix
A11 = det2by2([A(2,2), A(2,3); A(3,2), A(3,3)]); % Cofactors 3x3 matrix A
A12 = -(det2by2([A(2,1), A(2,3); A(3,1), A(3,3)]));
A13 = det2by2([A(2,1), A(2,2); A(3,1), A(3,2)]);
A21 = -(det2by2([A(1,2), A(1,3); A(3,2), A(3,3)]));
A22 = det2by2([A(1,1), A(1,3); A(3,1), A(3,3)]);
A23 = -(det2by2([A(1,1), A(1,2); A(3,1), A(3,2)]));
A31 = det2by2([A(1,2), A(1,3); A(2,2), A(2,3)]);
A32 = -(det2by2([A(1,1), A(1,3); A(2,1), A(2,3)]));
A33 = det2by2([A(1,1), A(1,2); A(2,1), A(2,2)]);
J = [ A11 A12 A13; A21 A22 A23; A31 A32 A33]; % Adjugate Matrix
determinant = ((A(1,1) * A11) + (A(1,2) * A12) + (A(1,3) * A13)); % Determinant of A
detInv = (1/determinant);
inverse =  detInv * (J'); % Inverse of A
if determinant==0
inverse=[];
end