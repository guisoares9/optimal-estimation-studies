% EXERCISE 1

close all
clear all

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

% prior knowledge

% calculate the prior knowledge using by known characteristics
pdf_x = px(x, beta, x_max, x_min);

% sensor models

% example of of p_z_x x=1.5
x_real = 1.5;
% calculate the sensor data based on the mistured gaussian model
pdf_z_x = pz_x(z, x_real, P0, P1, sigma);
figure
subplot(1,2,1)
plot(x, pdf_z_x, 'r')
title("P(z|x) for X=1.5")
ylabel("pdf")
xlabel('z')

% example of p_z_x x=2
x_real = 2;
% calculate the sensor data based on the mistured gaussian model
pdf_z_x = pz_x(z, x_real, P0, P1, sigma);
subplot(1,2,2)
plot(x, pdf_z_x, 'r')
title("P(z|x) for X=2")
xlabel('z')

% posterior probabilities

% example p_x_z for x=3.1
x_real = 3.1;
% calculate the sensor data based on the mistured gaussian model
pdf_z_x = pz_x(z, x_real, P0, P1, sigma);
% calculate the overall probability density p_z 
overall_z = pz(x, pdf_z_x, pdf_x);
% calculate the bayes formula
pdf_x_z = pdf_z_x .* pdf_x / overall_z;
figure
subplot(1,2,1)
plot(z, pdf_x_z)
title("P(x|z) for z=3.1")
ylabel("pdf")
xlabel("z")

% example p_x_z for x=4
x_real = 4;
% calculate the sensor data based on the mistured gaussian model
pdf_z_x = pz_x(z, x_real, P0, P1, sigma);
% calculate the overall probability density p_z 
overall_z = pz(x, pdf_z_x, pdf_x);
% calculate the bayes formula
pdf_x_z = pdf_z_x .* pdf_x / overall_z;
subplot(1,2,2)
plot(z, pdf_x_z)
title("P(x|z) for z=4")
ylabel("pdf")
xlabel("z")

% estimators

% example p_x_z for x=3.1
x_real = 3.1;
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

disp("Estimations for z=3.1")
disp("MMSE: x_hat = ")
disp(x_mmse)
disp("MMAE: x_hat = ")
disp(x_mmae)
disp("MAP: x_hat = ")
disp(x_map)
disp("ML: x_hat = ")
disp(x_ml)

figure
plot(z, pdf_x_z)
hold on
plot(z, pdf_z_x)
x_labels = [x_mmse x_mmae x_map x_ml];
labels = ["MMSE" "MMAE" "MAP" "ML"];
xline(x_labels, '--')
xlim([x_mmse - 0.1, x_ml + 0.1])
text(x_labels, [2 3 4 5], labels)
legend(["P(x|z)" "P(z|x)"])

% example p_x_z for x=4
x_real = 4;
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

disp("Estimations for z=4")
disp("MMSE: x_hat = ")
disp(x_mmse)
disp("MMAE: x_hat = ")
disp(x_mmae)
disp("MAP: x_hat = ")
disp(x_map)
disp("ML: x_hat = ")
disp(x_ml)

figure
plot(z, pdf_x_z)
hold on
plot(z, pdf_z_x)
x_labels = [x_mmse x_mmae x_map x_ml];
labels = ["MMSE" "MMAE" "MAP" "ML"];
xline(x_labels, '--')
xlim([x_mmse - 0.1, x_ml + 0.1])
text(x_labels, [2 3 4 5], labels)
legend(["P(x|z)" "P(z|x)"])

% Risk



