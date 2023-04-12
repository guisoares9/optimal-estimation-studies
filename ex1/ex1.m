% exercise 1 optimal estimation of dynamic systems
% author: Guilherme Soares Silvestre

close all
clear

%%%%%%%%%%%%%%%%%%%%%%%%
% sensor model for x=2
[x, ~, pdf_z_x, ~, ~, ~, ~, ~, ~, ~, ~] = full_estimates(2);
% subplot of pdf_z_x
fig_sensor = figure("Name", "Sensor model");
subplot(1,2,2)
plot(x, pdf_z_x, 'r')
title("P(z|x) for X=2")
xlabel('z')

%%%%%%%%%%%%%%%%%%%%%%%%
% sensor model for x=1.5
[x, ~, pdf_z_x, ~, ~, ~, ~, ~, ~, ~, ~] = full_estimates(1.5);
% subplot of pdf_z_x
figure(fig_sensor)
subplot(1,2,1)
plot(x, pdf_z_x, 'r')
title("P(z|x) for X=1.5")
ylabel("pdf")
xlabel('z')

%%%%%%%%%%%%%%%%%%%%%%%%
% full anlysis for x=3.1
[x, ~, pdf_z_x, pdf_x_z, x_mmse, x_mmae, x_map, x_ml, risk_mmse, ...
    risk_mmae, risk_map] = full_estimates(3.1);

fprintf('\n')
disp("Estimations for z=3.1")
fprintf("MMSE: x_hat = %.2f m | Risk = %.2f m^2\n", x_mmse, risk_mmse)
fprintf("MMAE: x_hat = %.2f m | Risk = %.2f m\n", x_mmae, risk_mmae)
fprintf("MAP: x_hat = %.2f m | Risk = %.2f m\n", x_map, risk_map)
fprintf('ML: x_hat = %.2f m\n', x_ml)


fig_post = figure("Name", "Posterior probabilities");
subplot(1,2,1)
plot(x, pdf_x_z)
title("P(x|z) for z=3.1")
ylabel("pdf")
xlabel("z")

figure
plot(x, pdf_x_z)
hold on
plot(x, pdf_z_x)
x_labels = [x_mmse x_mmae x_map x_ml];
labels = ["MMSE" "MMAE" "MAP" "ML"];
xline(x_labels, '--')
xlim([x_mmse - 0.1, x_ml + 0.1])
text(x_labels, [2 3 4 5], labels)
legend(["P(x|z)" "P(z|x)"])
title("Estimations for x=3.1")

%%%%%%%%%%%%%%%%%%%%%%%%
% full analysis of x=4
[x, ~, pdf_z_x, pdf_x_z, x_mmse, x_mmae, x_map, x_ml, risk_mmse, ...
    risk_mmae, risk_map] = full_estimates(4);

% display estimates and risks
fprintf('\n')
disp("Estimations for z=4")
fprintf("MMSE: x_hat = %.2f m | Risk = %.2f m^2\n", x_mmse, risk_mmse)
fprintf("MMAE: x_hat = %.2f m | Risk = %.2f m\n", x_mmae, risk_mmae)
fprintf("MAP: x_hat = %.2f m | Risk = %.2f m\n", x_map, risk_map)
fprintf('ML: x_hat = %.2f m\n', x_ml)

% plot posterior pdf
figure(fig_post)
subplot(1,2,2)
plot(x, pdf_x_z)
title("P(x|z) for z=4")
ylabel("pdf")
xlabel("z")

% plot estimates on top of the pdf
figure
plot(x, pdf_x_z)
hold on
plot(x, pdf_z_x)
x_labels = [x_mmse x_mmae x_map x_ml];
labels = ["MMSE" "MMAE" "MAP" "ML"];
xline(x_labels, '--')
xlim([x_mmse - 0.1, x_ml + 0.1])
text(x_labels, [2 3 4 5], labels)
legend(["P(x|z)" "P(z|x)"])
title("Estimations for x=4")




