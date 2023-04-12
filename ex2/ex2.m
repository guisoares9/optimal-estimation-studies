% exercise 2 optimal estimation of dynamic systems
% author: Guilherme Soares Silvestre

close all
clear

% first dataset
load("data exercise 2/depthgauge_data_set1.mat")
estimate_from_data(x1, z1, 1)

% second dataset
load("data exercise 2/depthgauge_data_set2.mat")
estimate_from_data(x2, z2, 2)

% function for estimating and alaysing
function estimate_from_data(x, z, data_id)
    
    % scatter plot
    figure
    plot(x, x, 'Color', 'r', 'LineStyle','-.', 'LineWidth', 2)
    hold on
    plot(x, z, 'Marker','o','MarkerSize',5,'LineStyle','none','MarkerFaceColor','b', 'Color', 'b')
    
    % calculating the mean of the data
    alpha_l = mean(x.*z)./mean(z.*z);
    
    % plot linear MMSE
    x_l = alpha_l.*z;
    plot(x, x_l, '.')
    
    % unbiased estimator
    alpha_ul = (mean(x.*z)-mean(x)*mean(z))./var(z);
    beta_ul = mean(x) - alpha_ul.*mean(z);
    x_ul = alpha_ul.*z+beta_ul;
    
    % plot unbiased linear MMSE
    plot(x, x_ul, '.')
    
    legend('x_{true}', 'z', 'x_{lMMSE}', 'x_{ulMMSE}')
    t = sprintf("Scatter plot for (x%d, z%d)", data_id, data_id);
    title(t)
    xlabel("x")
    ylabel("Estimation")
    
    
    % linear MMSE performance
    e_l = x - x_l;
    bias_l = mean(e_l);
    var_l = var(e_l);
    
    % Unbiased linear MMSE performance
    e_ul = x - x_ul;
    bias_ul = mean(e_ul);
    var_ul = var(e_ul);
    
    fprintf("\nDataset %d\n", data_id)
    fprintf("Optimal linear MMSE: x_{lMMSE} = %.3f * z\n", alpha_l)
    fprintf("Overall bias: %.5f\n", bias_l)
    fprintf("Error variance: %.3f\n", var_l)
    fprintf("Optimal Unbiased Linear MMSE: x_{ulMMSE} = %.3f * z + %.3f\n", alpha_ul, beta_ul)
    fprintf("Overall bias: %.5f\n", bias_ul)
    fprintf("Error variance: %.3f\n", var_ul)

end