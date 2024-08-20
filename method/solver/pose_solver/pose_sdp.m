function Y_list = pose_sdp(D_sets, RedundantA, RedundantB, param)
    
    Y_list = cell(1, param.lidar_pose_num);
    

    yalmip('clear');
    Y = sdpvar(13, 13);
    Constraints = [Y >= 0];
    for i = 1:21
        Constraints = [Constraints, trace(RedundantA{i} * Y) == RedundantB(i)];
    end
    Constraints = [Constraints, Y(13, 13) == 1];
    D_sdp = sdpvar(13, 13);
    Objective = [trace(D_sdp * Y)];

    ops = sdpsettings('solver','mosek','verbose',0,'debug',0);
    SDPSolver = optimizer(Constraints,Objective,ops,D_sdp,Y);


    for lidar_pose_i = 1:param.lidar_pose_num
        D = D_sets{1, lidar_pose_i};
        D = D([1:3, 5:7, 9:11, 4, 8, 12, 16], [1:3, 5:7, 9:11, 4, 8, 12, 16]);
       
        %Y_list{1,lidar_pose_i} = parfeval(backgroundPool,@SDPSolver,D);
        Y_list{1,lidar_pose_i} = SDPSolver(D);
    end


end