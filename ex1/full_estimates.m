function [x, pdf_x, pdf_z_x, pdf_x_z, x_mmse, x_mmae, x_map, x_ml, risk_mmse, risk_mmae, risk_map] = full_estimates(x_real)
    % initialization
    
    % create x vector representing the all possible real depths (x) and
    % sensored depths (z)
    x = linspace(0, 5, 10000);
    z = x;
    
    % prior knowledge params
    beta = 20;
    x_min = 1.5;
    x_max = 3;
    
    % sensor model params
    P0 = 0.95;
    P1 = 0.05;
    sigma = 0.1;
    
    % estimators params
    global delta_map;
    delta_map = 0.05;
    
    % prior knowledge
    
    % calculate the prior knowledge using by known characteristics
    pdf_x = px(x, beta, x_max, x_min);
    % calculate the sensor data based on the mistured gaussian model
    pdf_z_x = pz_x(z, x_real, P0, P1, sigma);
    % calculate the overall probability density p_z 
    overall_z = pz(x, pdf_z_x, pdf_x);
    % calculate the bayes formula
    pdf_x_z = pdf_z_x .* pdf_x / overall_z;
    % calculate estimatives
    x_mmse = mmse(x, pdf_x_z);
    x_mmae = mmae(x, pdf_x_z);
    x_map = map(x, pdf_x_z);
    x_ml = ml(x, pdf_z_x);
    % calculate risks
    risk_mmse = mmse_risk(pdf_x_z);
    risk_mmae = mmae_risk(pdf_x_z, x, x_mmae);
    risk_map = map_risk(pdf_x_z);
return
