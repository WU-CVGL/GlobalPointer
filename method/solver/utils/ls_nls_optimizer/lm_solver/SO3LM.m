function [x_opt, Residual] = SO3LM(opt_func, update_func, x, varargin)

params = inputParser;
params.KeepUnmatched = true;
params.addParameter('MaxIteration', 200);
params.addParameter('Lambda', 0.01);
params.addParameter('tolX', 1e-6);
params.addParameter('tolFun', 1e-6);
params.addParameter('tolOpt', 1e-10);
params.addParameter('SpecifyObjectiveGradient', false);
params.addParameter('CheckGradient', false);
params.addParameter('FixFirstParameter', 0);
params.addParameter('ParallelNumericalDiff', false);
params.addParameter('ParallelErrorCalculation', false);
params.addParameter('AutoFConstruction', false);
params.addParameter('AutoFConstructionObs', cell(0));
params.addParameter('Debug', 0);
params.parse(varargin{:});

lambda = params.Results.Lambda;
tolX = params.Results.tolX;
tolFun = params.Results.tolFun;
tolOpt = params.Results.tolOpt;


residual_list = [];
iter = 0;
sqrtEps=sqrt(eps);
l2_p=norm(x);


[F,J] = Calculate_F_J(x, opt_func, update_func, params);
F = reshape(F, size(F, 1) * size(F, 2), 1);
sqSumF = dot(F,F);
Residual = sqSumF;
H = J'*J;
JtF = J'*F;
linf_JtF=max(norm(JtF,Inf),sqrtEps); 
disp_local([num2str(iter) ' : ' num2str(sqSumF)], params.Results.Debug);


while iter < params.Results.MaxIteration
    iter = iter+1;
    
    H_LM = H + sparse(1:length(x),1:length(x),lambda,length(x),length(x));
    dp = -H_LM \ JtF;

    x_LM = update_func(x, dp');    
    l2_dp=norm(dp);
    
    if (l2_dp < tolX*(sqrtEps + l2_p) )
        disp_local('Finished (tolX)', params.Results.Debug);
        break;
    end
    
    

    [F_LM,J_LM] = Calculate_F_J(x_LM, opt_func, update_func, params);
    F_LM = reshape(F_LM, size(F_LM, 1) * size(F_LM, 2), 1);
    sqSumF_LM = dot(F_LM,F_LM);
    JtF_LM = J_LM'*F_LM;
    linf_JtF_LM=max(norm(JtF_LM,Inf),sqrtEps);
    
    if (linf_JtF_LM < tolOpt * linf_JtF)
        disp_local('Finished (tolOpt)', params.Results.Debug);
        break;
    end
    
    if ( abs(sqSumF_LM - sqSumF) <= tolFun*sqSumF )
        disp_local('Finished (tolFun)', params.Results.Debug);
        break;
    end
    
    if (sqSumF_LM < sqSumF)
        lambda = lambda * 0.1;
        x = x_LM;
        sqSumF = sqSumF_LM;
        disp_local([num2str(iter) ' : ' num2str(sqSumF)], params.Results.Debug);
        Residual = [Residual, sqSumF];
        H = J_LM'*J_LM;
        JtF = JtF_LM;
    else
        lambda=lambda*10;
    end
end

if iter >= params.Results.MaxIteration
    disp_local('Finished (max_iter)', params.Results.Debug);
end

x_opt=x_LM;
end

function [F,J] = Calculate_F_J(x, opt_func, update_func, params)
    if params.Results.AutoFConstruction
        if params.Results.SpecifyObjectiveGradient
            functor = @(x_s)F_J_AutoConstruct(x_s, opt_func, params.Results.AutoFConstructionObs, params.Results.ParallelErrorCalculation);
            [F,J] = functor(x); 
            if params.Results.CheckGradient
                J_num = NumericalDiff(x, F, functor, update_func, params.Results.ParallelNumericalDiff);
                disp_local(['Check Gradient', num2str(vec(J - J_num))], params.Results.Debug);
            end
        else
            functor = @(x_s)F_AutoConstruct(x_s, opt_func, params.Results.AutoFConstructionObs, params.Results.ParallelErrorCalculation);
            [F] = functor(x); 
            J = NumericalDiff(x, F, functor, update_func, params.Results.ParallelNumericalDiff);
        end
    else
        if params.Results.SpecifyObjectiveGradient
            [F,J] = opt_func(x); 
            if params.Results.CheckGradient
                J_num = NumericalDiff(x, F, opt_func, update_func, params.Results.ParallelNumericalDiff);
                disp_local(['Check Gradient', num2str(norm(J - J_num))], params.Results.Debug);
            end
        else
            [F] = opt_func(x);
            J = NumericalDiff(x, F, opt_func, update_func, params.Results.ParallelNumericalDiff);
        end
    end
end


function disp_local(message, debug_flag)
    if debug_flag
        display(message);
    end
end