% function X_list = plane_sdp(C_sets, param)
% 
% 
%     X_list = cell(1, param.plane_num);
%     for plane_i = 1:param.plane_num
%         yalmip('clear');
%         X = sdpvar(4, 4);
%         A = zeros(4, 4);
%         A(1, 1) = 1;
%         A(2, 2) = 1;
%         A(3, 3) = 1;
%         Constraints = [X >= 0, trace(A* X) == 1];
%         C = C_sets{1, plane_i};
%         Objective = trace(C * X);
%         % Set some options for YALMIP and solver
%         ops = sdpsettings('solver','mosek','verbose',0,'debug',0);
%         sol = optimize(Constraints,Objective,ops);
%         X_list{1,plane_i} = value(X);
%     end
% end


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