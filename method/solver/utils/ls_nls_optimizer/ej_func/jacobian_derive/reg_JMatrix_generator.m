function [] = reg_JMatrix_generator()

% ----------------- file_path -----------------
file_path = 'C:\Users\10627\Desktop\globalpointer_eccv\method\solver\utils\ls_nls_optimizer\jacobian_derive\';

% ----------------- symsbol -----------------
syms nx ny nz

 
% ----------------- error definition -----------------
error = sqrt([nx,ny,nz] * [nx;ny;nz]) - 1;




J_plane_func = jacobian(error, [nx,ny,nz]);
J_func = matlabFunction(J_plane_func, 'File', [file_path , 'reg_J_plane_func']);
error_func = matlabFunction(error, 'File', [file_path , 'reg_error_func']);

end
