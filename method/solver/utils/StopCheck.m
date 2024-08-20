function flag = StopCheck(total_error_list, runtime_param)
    flag = false;
    if size(total_error_list, 2) > 1 && abs(total_error_list(1, end) - total_error_list(1, end - 1)) / total_error_list(1, end) < runtime_param.RelativeCostThreshold
        flag = true;
    end
    if abs(total_error_list(1, end)) < runtime_param.AbsoluteCostThreshold
        flag = true;
    end
    
end