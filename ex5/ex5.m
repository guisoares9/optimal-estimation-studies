% exercise 5 optimal estimation of dynamic systems
% author: Guilherme Soares Silvestre

clear 
close all

% given accelration system matrix and covariance
F_1 = [0.97 0;
      0 0.97];

% process noise covariance
Cw_1 = [0.0016 0;
       0 0.0016];

% measurement noise covariance
Cn_1 = [49 0;
       0 49];

% intialize calculated system matrix for full state
global F
F = [1 0 1 0 0 0;
    0 1 0 1 0 0;
    0 0 1 0 1 0;
    0 0 0 1 0 1;
    0 0 0 0 F_1(1,1) F_1(1,2);
    0 0 0 0 F_1(2,1) F_1(2,2);];

% noise covariance matrix only in the acceleration part (proccess noise)
global Cw
Cw =  [0 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 Cw_1(1,1) Cw_1(1,2);
      0 0 0 0 Cw_1(2,1) Cw_1(2,2)];

% measurement noise
% Cn = [Cn_1(1,1) Cn_1(1,2) 0 0 0 0;
%       Cn_1(2,1) Cn_1(2,2) 0 0 0 0;
%       0 0 Cn_1(1,1) Cn_1(1,2) 0 0;
%       0 0 Cn_1(2,1) Cn_1(2,2) 0 0;
%       0 0 0 0 Cn_1(1,1) Cn_1(1,2);
%       0 0 0 0 Cn_1(2,1) Cn_1(2,2)];
Cn = [Cn_1(1,1) Cn_1(1,2) 0 0 0 0;
      Cn_1(2,1) Cn_1(2,2) 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 1 0 0;
      0 0 0 0 1 0;
      0 0 0 0 0 1];

% initial covariance
sd_x = 100; %m
sd_v = 4; %m/s
sd_a = 0.2; %m/s^2
Cx_0 = [sd_x.^2 0 0 0 0 0;
        0 sd_x.^2 0 0 0 0;
        0 0 sd_v.^2 0 0 0;
        0 0 0 sd_v.^2 0 0;
        0 0 0 0 sd_a.^2 0;
        0 0 0 0 0 sd_a.^2];

% initial mean
x_0 = [0;0;0;0;0;0];

% first define measurement matrix
H = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0];

% create states vector
% load position: (2, N) matrix
load 'data exercise 5'/zradar.mat

% lenght of xsi
N = length(z);

z_full = zeros(6, N);
z_full(1:2,:) = z;

% discrete kalman filter main loop
x_hat = zeros(6, N);
x_pred = x_hat;

Cx_hat = zeros(6, 6, N);
Cx_pred = Cx_hat;

x_pred(:,1) = x_0;
Cx_pred(:,:,1) = Cx_0;
for i = 1:N
    
    % update step
    
    % innovation
    inov = z_full(:,i) - H*x_pred(:,i);
    % innovation covariance (pre-fit residual)
    S = H*Cx_pred(:,:,i)*H' + Cn;
    % kalman gain
    K = Cx_pred(:,:,i)*H'/S;
    % a posteriori estimation
    x_hat(:,i) = x_pred(:,i) + K*inov;
    % a posteriori covariance
    Cx_hat(:,:,i) = (eye(6)-K*H)*Cx_pred(:,:,i);

    % predict step

    % prior estimate
    x_pred(:,i+1) = F*x_hat(:,i);
    % estimate covariance
    Cx_pred(:,:,i+1) = F*Cx_hat(:,:,i)*F' + Cw;
end

% plot measurements, predicted and estimated position
plot(z(1,:), z(2,:), '*')
hold on
plot(x_pred(1,:), x_pred(2,:))
plot(x_hat(1,:), x_hat(2,:))

% draw covariance ellipses
plot_elipses(x_hat, Cx_hat)

axis equal
xlabel('x (m)')
ylabel('y (m)')
title("Discrete Kalman Filter")
legend(["Measurements" "Predicted" "Estimated"])


% calculating kalman gain and covariances with matlab default method
[K_dlqe,Cx_pred_dlqe,Cx_hat_dlqe] = dlqe(F,eye(6),H,Cw,Cn);

% rerunning the system with steady state matrix
x_hat_dlqe = zeros(6, N);
x_pred_dlqe = x_hat;

x_pred_dlqe(:,1) = x_0;
for i = 1:N

    % update step

    % innovation
    inov = z_full(:,i) - H*x_pred_dlqe(:,i);
    % a posteriori estimation
    x_hat_dlqe(:,i) = x_pred_dlqe(:,i) + K_dlqe*inov;

    % predict step

    % prior estimate
    x_pred_dlqe(:,i+1) = F*x_hat_dlqe(:,i);
end


% plot measurements, predicted and estimated position for steady state
figure
plot(z(1,:), z(2,:), '*')
hold on
plot(x_pred_dlqe(1,:), x_pred_dlqe(2,:))
plot(x_hat_dlqe(1,:), x_hat_dlqe(2,:))

% draw covariance ellipses
Cx_hat_dlqe_arr = repmat(Cx_hat_dlqe, [6,6,N]);
plot_elipses(x_hat_dlqe, Cx_hat_dlqe_arr)

axis equal
xlabel('x (m)')
ylabel('y (m)')
title("Discrete Kalman Filter - Steady State")
legend(["Measurements" "Predicted" "Estimated"])

% 
% % velocity: (2, N-1) matrix
% z_v = zeros(2,N-1);
% for i=1:(N-1)
%     z_v(:,i) = z(:,i+1)-z(:,i);
% end
% 
% % acceleration: (2, N-2) matrix
% z_a = zeros(2,N-2);
% for i=1:(N-2)
%     z_a(:,i) = z_v(:,i+1)-z_v(:,i);
% end
% 
% % state space matrix
% global x
% x = [xsi(:,1:N-2); vsi(:,1:N-2); asi(:,1:N-2)];


function plot_elipses(x_arr, Cx_arr)
    
    % for each cov matrix inside Cx_Arr, calculate the eigen vector
    for index = 1:3:length(Cx_arr)
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
        xelp = pts(1,:) + x_arr(1, index);
        yelp = pts(2,:) + x_arr(2, index);

        % plot elipse
        plot(xelp, yelp)
    end
end