% exercise 6 optimal estimation of dynamic systems
% author: Guilherme Soares Silvestre

clear
close all

% extended kalman filter: the yatch case

% load measurements
load 'data exercise 6'/z_yacht.mat

% % load usefull functions
% load 'data exercise 6'/Fjacobian.m
% load 'data exercise 6'/fsys.m
% load 'data exercise 6'/Hjacobian.m
% load 'data exercise 6'/hmeas.m

% build the process noise covariance matrix
sd_w_x = 0.01; % m
sd_w_v = 0.01; % m
sd_w_a = 0.01; % m
sd_w_t = 8; % N
sd_w_phi = 0.5; % deg

Cw = eye(8).*[sd_w_x.^2;
              sd_w_x.^2;
              sd_w_v.^2;
              sd_w_v.^2;
              sd_w_a.^2;
              sd_w_a.^2;
              sd_w_t.^2;
              sd_w_phi.^2];

% build the measurement noise covariance matrix
sd_compass = 1; % deg
sd_speed = 0.3; % m/s
sd_heading = 1; % deg

Cv = [sd_compass.^2 0 0;
      0 sd_speed.^2 0;
      0 0 sd_heading.^2];

% beacon position
x0 = [5000 10000]';

% desired thrust and heading
t_des = 400; % N (t0)
phi_des = 45; % deg (phi0)
u = [t_des phi_des]';

% initial prior pose
x_hat_0 = [0 0 0 0 0 0 400 0]';

% build the initial covariance matrix
sd_x_0 = 10000; % m
sd_v_0 = 2; % m/s
sd_a_0 = 0.04; % m/s
sd_t_0 = 300; % N
sd_phi_0 = 10; % deg

Cx_0 = eye(8).*[sd_x_0.^2;
              sd_x_0.^2;
              sd_v_0.^2;
              sd_v_0.^2;
              sd_a_0.^2;
              sd_a_0.^2;
              sd_t_0.^2;
              sd_phi_0.^2];

% main loop for extended kalman filter
N = 1000;

% initialize arrays
x_hat = zeros(8, N);
Cx_hat = zeros(8,8,N);

x_hat(:,1) = x_hat_0;
Cx_hat(:,:,1) = Cx_0;
for i=1:10000
    % update
    if any(imeas(:)==i)
        imeas_idx = find(imeas==i);
        Cx = Cx_hat(:,:,i);
        x = x_hat(:,i);
        H = Hjacobian(x_hat(:,i), x0);
        z_curr = z(:,imeas_idx);

        % estimated measurement
        z_hat = hmeas(x, x0, Cv);
        
        % innovation
        S = H*Cx*H' + Cv;

        % Kalman Gain
        K = Cx*H'/S;

        % update posterior mean
        x_hat(:,i+1) = x + K*(z_curr-z_hat);

        % update posterior covariance
        Cx_hat(:,:,i+1) = Cx - K*S*K'; 
    else
    
        % predict
        x_hat(:,i+1) = fsys(x_hat(:,i), u, Cw);
        F = Fjacobian(x_hat(:,i));
        Cx_hat(:,:,i+1) = F*Cx_hat(:,:,i)*F' + Cw;
    end
end

% plot estimates
t = linspace(0,10001,10001);
figure

% plot x
subplot(2,4,1)
plot(t, x_hat(1,:))
title("Position: X")
ylim([-10000 20000])
ylabel("x (m)")

% plot y
subplot(2,4,5)
plot(t, x_hat(2,:))
title("Position: Y")
ylim([-10000 20000])
ylabel("y (m)")
xlabel("t (steps)")

% plot velocity x
subplot(2,4,2)
plot(t, x_hat(3,:))
title("Velocity: X")
ylabel("vx (m)")

% plot velocity y
subplot(2,4,6)
plot(t, x_hat(3,:))
title("Velocity: y")
ylabel("vy (m)")
xlabel("t (steps)")

% plot accel x
subplot(2,4,3)
plot(t, x_hat(4,:))
title("Accel: X")
ylabel("ax (m)")

% plot accel y
subplot(2,4,7)
plot(t, x_hat(5,:))
title("Accel: y")
ylabel("ay (m)")
xlabel("t (steps)")

% plot force
subplot(2,4,4)
plot(t, x_hat(7,:))
title("Force")
ylabel("t (N)")

% plot phi
subplot(2,4,8)
plot(t, x_hat(8,:))
title("phi")
ylabel("phi (°)")
xlabel("t (steps)")

% plot path
figure
plot(x_hat(1,:)./1000, x_hat(2,:)./1000, ".")
hold on
plot_elipses(x_hat./1000, Cx_hat./1000^2)
xlim([-10 20])
ylim([-10 20])
xlabel("x (Km)")
ylabel("y (Km)")
title("Estimated path")

function plot_elipses(x_arr, Cx_arr)
    
    % for each cov matrix inside Cx_Arr, calculate the eigen vector
    for index = 1:200:length(Cx_arr)
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