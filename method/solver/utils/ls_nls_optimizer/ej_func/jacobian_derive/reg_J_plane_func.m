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

function J_plane_func = reg_J_plane_func(nx,ny,nz)

    t2 = nx.^2;
    t3 = ny.^2;
    t4 = nz.^2;
    t5 = t2+t3+t4;
    t6 = 1.0./sqrt(t5);
    J_plane_func = [nx.*t6,ny.*t6,nz.*t6];

end
