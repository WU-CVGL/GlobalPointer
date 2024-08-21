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

function X_list = plane_sdp(C_sets, param)
    
    X_list = cell(1, param.plane_num);

    yalmip('clear');

    X = sdpvar(4, 4);
    A = zeros(4, 4);
    A(1, 1) = 1;
    A(2, 2) = 1;
    A(3, 3) = 1;
    
    Constraints = [X >= 0, trace(A* X) == 1];
    C_sdp = sdpvar(4, 4);
    Objective = trace(C_sdp * X);
    ops = sdpsettings('solver','mosek','verbose',0,'debug',0);
    SDPSolver = optimizer(Constraints,Objective,ops,C_sdp,X);

    for plane_i = 1:param.plane_num
        C = C_sets{1, plane_i};
        X_list{1,plane_i} = SDPSolver(C);
    end

end