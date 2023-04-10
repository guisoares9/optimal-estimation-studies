% exercise 4 optimal estimation of dynamic systems
clear 
close all

% create states vector
% load position: (2, N) matrix
load 'data exercise 4'/log.mat
% lenght of xsi
N = length(xsi);

% velocity: (2, N-1) matrix
vsi = zeros(2,N-1);
for i=1:(N-1)
    vsi(:,i) = xsi(:,i+1)-xsi(:,i);
end

% acceleration: (2, N-2) matrix
asi = zeros(2,N-2);
for i=1:(N-2)
    asi(:,i) = vsi(:,i+1)-vsi(:,i);
end

% state space matrix
global x
x = [xsi(:,1:N-2); vsi(:,1:N-2); asi(:,1:N-2)];

% given accelration system matrix and covariance
F_1 = [-0.0595 -0.1530;
      -0.813 0.1716];

Cw_1 = [0.1177e-3 -0.0026e-3;
      -0.0026e-3 0.0782e-3];

% intialize calculated system matrix for full state
global F
F = [1 0 1 0 0 0;
    0 1 0 1 0 0;
    0 0 1 0 1 0;
    0 0 0 1 0 1;
    0 0 0 0 F_1(1,1) F_1(1,2);
    0 0 0 0 F_1(2,1) F_1(2,2);];

% Noise covariance matrix only in the acceleration part
global Cw
Cw =  [0 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 Cw_1(1,1) Cw_1(1,2);
      0 0 0 0 Cw_1(2,1) Cw_1(2,2)];

% starting point = 11
simulate_prediction(11)

% starting point = 31
figure
subplot(2,2,1)
simulate_prediction(31)

% starting point = 71
subplot(2,2,2)
simulate_prediction(71)

% starting point = 71
subplot(2,2,3)
simulate_prediction(71)

% starting point = 91
subplot(2,2,4)
simulate_prediction(91)

function simulate_prediction(start)
    
    global x
    global F
    global Cw

    steps = 101-start;
    x_10 = x(:,start);
    Cx_10 = zeros(6,6);
    
    % predict
    [x_ahead, Cx_ahead] = predict(F, Cw, Cx_10, x_10, steps);
    
    % plot prediction
    plot(x(1,:), x(2,:))
    hold on
    plot(x_ahead(1,:), x_ahead(2,:))
    
    plot_elipses(x_ahead, Cx_ahead)
    xlabel('x (m)')
    ylabel('y (m)')
    t=sprintf("Prediction starting in i = %d", start);
    title(t)
    legend(["Ground Truth" "Prediction"])
end

function [x_ahead, Cx_ahead] = predict(F, Cw, Cx, x, steps)
    
    % create state vector for prediction
    size_x = size(x);
    x_ahead = zeros(size_x(1), steps);
    Cx_ahead = zeros(size_x(1), size_x(1), steps);

    % start state and cov matrix with initial states
    x_ahead(:,1) = x;
    Cx_ahead(:,:, 1) = Cx;

    % loop for 'steps' times calculating the state prediction and
    % covariance for each state
    for i = 1:steps
        x_ahead(:,i+1) = F*x_ahead(:,i);
        Cx_ahead(:,:,i+1) = F*Cx_ahead(:,:,i)*F' + Cw;
    end
end

function plot_elipses(x_ahead, Cx_arr)
    
    % for each cov matrix inside Cx_Arr, calculate the eigen vector
    for index = 1:length(Cx_arr)
        Cx_curr = Cx_arr(1:2,1:2, index);
        [eig_vec, eig_vals] = eig(Cx_curr);
        
        % unit circle for further use
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
        xelp = pts(1,:) + x_ahead(1, index);
        yelp = pts(2,:) + x_ahead(2, index);

        % plot elipse
        plot(xelp, yelp)
    end
end