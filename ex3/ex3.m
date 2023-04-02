% ex 3
clear 
close all

% defining the system
ux = [10 20]'; %m
x0 = [100 100]'; %m
theta = deg2rad(35); %rad
sd_theta = deg2rad(1); %rad
Cx = [25 -25; 
      -25 70];

% Figure 1
estimate_ulmmse(ux, Cx, x0, theta, sd_theta)

%%%%%%%%%%%%%%%%%%%%
% changing sd theta
% sd_theta = 2°
sd_theta = deg2rad(2);
figure
subplot(1,3,1)
estimate_ulmmse(ux, Cx, x0, theta, sd_theta)
title("\sigma_{\theta} = 2")

% sd_theta = 5°
sd_theta = deg2rad(5);
subplot(1,3,2)
estimate_ulmmse(ux, Cx, x0, theta, sd_theta)
title("\sigma_{\theta} = 5")

% sd_theta = 10°
sd_theta = deg2rad(10);
subplot(1,3,3)
estimate_ulmmse(ux, Cx, x0, theta, sd_theta)
title("\sigma_{\theta} = 10")

%%%%%%%%%%%%%%%%%%%%
% changing covariance to a factor of alpha
% reseting sd_theta to 1
sd_theta = deg2rad(1);

% alpha = 2
Cx = 2*Cx;
figure
subplot(1,3,1)
estimate_ulmmse(ux, Cx, x0, theta, sd_theta)
title("\alpha = 2")

% alpha = 5
Cx = 5*Cx;
subplot(1,3,2)
estimate_ulmmse(ux, Cx, x0, theta, sd_theta)
title("\alpha = 5")

% alpha = 10
Cx = 10*Cx;
subplot(1,3,3)
estimate_ulmmse(ux, Cx, x0, theta, sd_theta)
title("\alpha = 10")

function estimate_ulmmse(ux, Cx, x0, theta, sd_theta)
    %%%%%%%%%%%%%%%%%%%%
    % draw uncertainty
    
    % eigenvalue of Cx
    [eig_vec, eig_vals] = eig(Cx);
    disp("EigenValues of Cx = ")
    disp(eig_vals)
    disp("EigenVectors of Cx = ")
    disp(eig_vec)
    
    % unit circle
    th = 0:pi/50:2*pi;
    xelp = cos(th);
    yelp = sin(th);
    
    % scale circle accordingly
    scale_x = sqrt(eig_vals(1,1));
    scale_y = sqrt(eig_vals(2,2));
    xelp = scale_x*xelp;
    yelp = scale_y*yelp;
    
    % rotate elipse
    pts = eig_vec * [xelp; yelp];
    xelp = pts(1,:) + ux(1);
    yelp = pts(2,:) + ux(2);
    
    % plot elipse
    plot(xelp, yelp)
    hold on
    plot(ux(1), ux(2), '*')
    text(ux(1), ux(2), "\mu_{x}",...
        'HorizontalAlignment','left',...
        'VerticalAlignment','top',...
        'FontSize',12)
    line([ux(1) 3*eig_vec(1,1) + ux(1)], [ux(2) 3*eig_vec(2,1) + ux(2)])
    line([ux(1) 3*eig_vec(1,2) + ux(1)], [ux(2) 3*eig_vec(2,2) + ux(2)])
    xlabel('x')
    ylabel('y')
    title("Ship state estimation")
    axis equal
    
    %%%%%%%%%%%%%%%%%%%%
    
    % add  line of sight
    
    % plot principal line and corner lines
    line([ux(1) ux(1)+150*cos(theta)],...
        [ux(2) ux(2)+150*sin(theta)])
    line([ux(1) ux(1)+150*cos(theta+sd_theta)],...
        [ux(2) ux(2)+150*sin(theta+sd_theta)],...
        'Color','red','LineStyle','--')
    line([ux(1) ux(1)+150*cos(theta-sd_theta)],...
        [ux(2) ux(2)+150*sin(theta-sd_theta)],...
        'Color','red','LineStyle','--')
    
    % plot lighthouse real position x0
    plot(x0(1), x0(2), '*')
    text(x0(1), x0(2), 'x_0',...
        'HorizontalAlignment','left',...
        'VerticalAlignment','baseline',...
        'FontSize',12)
    
    %%%%%%%%%%%%%%%%%%%%
    
    % plot linearized uncertainty bar region of the line of sight
    
    % two parallel lines width
    % first calculate the distance
    d = norm(ux-x0);
    sd_v = d*sd_theta;
    
    % difference between the center line and the top and bottom lines in y axis
    y_diff = sd_v/cos(theta);
    
    % plot parallels lines in green
    line([ux(1) ux(1)+150*cos(theta)],...
        [ux(2)+y_diff ux(2)+150*sin(theta)+y_diff],...
        'Color','green','LineStyle','--')
    line([ux(1) ux(1)+150*cos(theta)],...
        [ux(2)-y_diff ux(2)+150*sin(theta)-y_diff],...
        'Color','green','LineStyle','--')
    
    %%%%%%%%%%%%%%%%%%%%
    
    % ulMMSE estimation
    
    % kalman measurement matrix: H
    H = [sin(theta) -cos(theta)];
    disp("Kalman gain matrix: H = ")
    disp(H)
    
    % derived measurement: z
    z = x0(1)*sin(theta) - x0(2)*cos(theta);
    disp("Derived measurement: z = ")
    disp(z)
    
    % kalman gain matrix: K
    Cv = sd_v^2;
    K = Cx*H'/(H*Cx*H'+Cv);
    disp("Kalman Gain matrix: K = ")
    disp(K)
    
    % perform estimation with ulMMSE
    x_hat = ux +K*(z-H*ux);
    disp("Estimated state with ulMMSE")
    disp(x_hat)
    
    % plot estimated state
    plot(x_hat(1), x_hat(2), "*")
    text(x_hat(1), x_hat(2), '$$x_{ulMMSE}$$',...
        'Interpreter','Latex',...
        'HorizontalAlignment','left',...
        'VerticalAlignment','bottom',...
        'FontSize',12)
    
    % calculate covariance Cxz
    Cx_z = inv(inv(Cx) + H'/Cv*H);
    disp("Posterior covariance: Cx_z = ")
    disp(Cx_z)
    
    %%%%%%%%%%%%%%%%%%%%
    
    % draw posterior uncertainty
    
    % eigenvalue of Cxz
    [eig_vec, eig_vals] = eig(Cx_z);
    disp("EigenValues of Cxz = ")
    disp(eig_vals)
    disp("EigenVectors of Cxz = ")
    disp(eig_vec)
    
    % unit circle
    th = 0:pi/50:2*pi;
    xelp = cos(th);
    yelp = sin(th);
    
    % scale circle accordingly
    scale_x = sqrt(eig_vals(1,1));
    scale_y = sqrt(eig_vals(2,2));
    xelp = scale_x*xelp;
    yelp = scale_y*yelp;
    
    % rotate elipse
    pts = eig_vec * [xelp; yelp];
    xelp = pts(1,:) + x_hat(1);
    yelp = pts(2,:) + x_hat(2);
    
    % plot elipse
    plot(xelp, yelp, 'b')

end