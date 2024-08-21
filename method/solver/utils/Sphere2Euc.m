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

function  [Euc_plane_estimate_vec] = Sphere2Euc(Sphere_plane_estimate_vec, param)

    Euc_plane_estimate_vec = zeros(1, 4 * param.plane_num);

    for plane_i = 1:param.plane_num
        plane_sph_id = (plane_i - 1) * 3;
        plane_euc_id = (plane_i - 1) * 4;

        sph_vec = Sphere_plane_estimate_vec(1, plane_sph_id+1:plane_sph_id+3);

        euc_vec = [sin(sph_vec(1))*cos(sph_vec(2)),sin(sph_vec(1))*sin(sph_vec(2)),cos(sph_vec(1)),sph_vec(3)];

        Euc_plane_estimate_vec(1, plane_euc_id+1:plane_euc_id+4) = euc_vec;
    end

end