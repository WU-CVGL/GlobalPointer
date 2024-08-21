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

function flag = OptimalCheck(total_error_list, runtime_param)

    flag = true;
    
    if size(total_error_list, 2) > 3 && total_error_list(1, end) - total_error_list(1, end - 1) > 0
        if abs(total_error_list(1, end) - total_error_list(1, end - 1)) > total_error_list(1, end)
            flag = false;
        end
    end

end