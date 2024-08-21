% This function is part of the GlobalPointer method as described in [1]. When you
% use this code, you are required to cite [1].
% 
% [1] GlobalPointer: Large-Scale Plane Adjustment with Bi-Convex Relaxation
% Author: B. Liao, Z. Zhao, L. Chen, H. Li, D. Cremers, P. Liu.
% European Conference on Computer Vision 2024 (ECCV 2024)
%
% 
% Author & Copyright (C) 2024: Bangyan Liao (liaobangyan[at]westlake[dot]edu[dot]cn)
%                              Zhenjun Zhao (ericzzj89[at]gmail[dot]com)
%                              Peidong Liu (liupeidong[at]westlake[dot]edu[dot]cn)

function [] = reg_JMatrix_generator()

    % ----------------- file_path -----------------
    file_path = 'PATH_TO_PROJECT\GlobalPointer\method\solver\utils\ls_nls_optimizer\jacobian_derive\';

    % ----------------- symsbol -----------------
    syms nx ny nz

    % ----------------- error definition -----------------
    error = sqrt([nx,ny,nz] * [nx;ny;nz]) - 1;
    J_plane_func = jacobian(error, [nx,ny,nz]);
    J_func = matlabFunction(J_plane_func, 'File', [file_path , 'reg_J_plane_func']);
    error_func = matlabFunction(error, 'File', [file_path , 'reg_error_func']);

end
