function [] = JMatrix_generator()

% ----------------- file_path -----------------
file_path = 'C:\Users\10627\Desktop\globalpointer_eccv\project\method\solver\utils\ls_nls_optimizer\ej_func\jacobian_derive\';

% ----------------- symsbol -----------------
syms nx ny nz
syms nq 
syms rx ry rz
syms tx ty tz
syms P1 P2 P3


% ----------------- interference -----------------
% Expression for the rotation matrix based on the Rodrigues formula
theta=sqrt(rx^2+ry^2+rz^2);
omega =  [0 -rz ry;
         rz 0 -rx;
        -ry rx 0;];
R = eye(3) + (sin(theta)/theta)*omega + ((1-cos(theta))/theta^2)*(omega*omega);

% Expression for the point vector
P_trans = R * [P1;P2;P3] + [tx;ty;tz];


 
% ----------------- error definition -----------------
error = [nx,ny,nz] * P_trans + nq;



J_full_func = jacobian(error, [rx,ry,rz,tx,ty,tz,nx,ny,nz,nq]);
J_func = matlabFunction(J_full_func, 'File', [file_path , 'J_full_func']);
J_pose_func = jacobian(error, [rx,ry,rz,tx,ty,tz]);
J_func = matlabFunction(J_pose_func, 'File', [file_path , 'J_pose_func']);
J_plane_func = jacobian(error, [nx,ny,nz,nq]);
J_func = matlabFunction(J_plane_func, 'File', [file_path , 'J_plane_func']);
error_func = matlabFunction(error, 'File', [file_path , 'error_func']);

end
