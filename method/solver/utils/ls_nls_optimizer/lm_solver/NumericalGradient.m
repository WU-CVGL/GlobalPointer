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

function [g] = NumericalGradient(x, opt_func, update_func, parallel_flag)

    parameter_dim = size(x, 2);
    g = zeros(1, parameter_dim);

    if parallel_flag
        return;
    else
        for i = 1:parameter_dim
            % --- forward ---
            x_p = zeros(1, parameter_dim);
            x_p(1, i) = 1e-6;
            x_tmp = update_func(x, x_p);
            [F_p] = opt_func(x_tmp);

            % --- backward ---
            x_n = zeros(1, parameter_dim);
            x_n(1, i) = -1e-6;
            x_tmp = update_func(x, x_n);
            [F_n] = opt_func(x_tmp);

            j_slice = (F_p - F_n) / 2e-6;
            g(:, i) = j_slice;
        end
    end
    
end
