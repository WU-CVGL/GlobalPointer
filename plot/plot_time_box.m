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

function plot_time_box(results, method_list, plot_list, x_axis_list, title_time, is_log10, y_axis_title, x_axis_title)
    
    epoch_size_num = size(results{1, 1}.error, 1);
    iteration_size_num = size(results{1, 1}.error, 2);
    method_size_num = size(plot_list, 2);

    x_list = [];
    y_list = [];
    label_list = [];

    for plot_i = 1:method_size_num
        x_tmp_list = [];
        y_tmp_list = [];

        plot_method = plot_list(1, plot_i);
        method_id = find(method_list == plot_method);
        time_all = results{method_id, 1}.time;

        for epoch_i = 1:epoch_size_num
            time_single = [];
            for iteration_i = 1:iteration_size_num
                if time_all(epoch_i, iteration_i) ~= 0
                    time_single = [time_single; time_all(epoch_i, iteration_i)];
                end
            end
            if is_log10
                y_tmp_list = [y_tmp_list; log10(time_single)];
            else
                y_tmp_list = [y_tmp_list; (time_single)];
            end
            x_tmp_list = [x_tmp_list; repmat(x_axis_list(1, epoch_i), size(time_single))];
        end

        x_list = [x_list; x_tmp_list];
        y_list = [y_list; y_tmp_list];
        label_list = [label_list;repmat(plot_method, size(x_tmp_list))];
    end

    x_list = categorical(x_list, x_axis_list);
    figure
    boxchart(x_list, y_list, 'GroupByColor', label_list);
    title(title_time);
    xlabel(x_axis_title);
    ylabel(y_axis_title);
    legend;
    set(gcf, 'color', 'w');

end