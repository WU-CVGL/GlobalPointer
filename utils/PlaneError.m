function [nError, dError] = PlaneError(plane, plane_gt, param)
    nError = 0;
    dError = 0;
    for plane_i = 1:param.plane_num
        n = plane.normal_vector_list{1, plane_i};
        d = plane.q_list{1, plane_i};
        n_gt = plane_gt.normal_vector_gt_list{1, plane_i};
        d_gt = -n_gt' * plane_gt.q_gt_list{1, plane_i};
        nError = nError + norm(abs(n) - abs(n_gt))^2;
        dError = dError + norm(abs(d) - abs(d_gt))^2;
    end
    nError = nError / param.plane_num;
    dError = dError / param.plane_num;
end