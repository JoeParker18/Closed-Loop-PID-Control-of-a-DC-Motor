% AERSP 304 - Assignment 1: DC Motor Transfer Function ID
% By: Joe Parker, Robert Danner and Luke Clemens

%% Trials 1-5
% Read step response data 
opts = detectImportOptions('motor_data.csv', 'CommentStyle', '#');
data = readmatrix('motor_data.csv', opts);

time    = data(:,1);
voltage = data(:,2);
rpm     = data(:,3);

% Split into 5 trials by voltage level
voltages = [9.600, 8.376, 7.200, 6.024, 4.800];
K_vals   = zeros(1,5);
tau_vals = zeros(1,5);

for i = 1:5
    idx = abs(voltage - voltages(i)) < 0.01;
    t_trial   = time(idx);
    rpm_trial = rpm(idx);
    
    % Reset time to start at 0.1
    t_trial = t_trial - t_trial(1) + 0.1;
    
    % Steady state = mean of last 50% of data
    ss_rpm = mean(rpm_trial(round(end*0.5):end));
    
    % DC gain K
    K_vals(i) = ss_rpm / voltages(i);
    
    % Find tau: time to reach 63.2% of steady state
    target  = 0.632 * ss_rpm;
    idx_tau = find(rpm_trial >= target, 1, 'first');
    if ~isempty(idx_tau) && idx_tau > 1
        t1 = t_trial(idx_tau-1); rpm1 = rpm_trial(idx_tau-1);
        t2 = t_trial(idx_tau);   rpm2 = rpm_trial(idx_tau);
        tau_vals(i) = t1 + (target - rpm1)/(rpm2 - rpm1) * (t2 - t1);
    else
        tau_vals(i) = t_trial(idx_tau);
    end
    
    % Individual figure for each trial
    figure(i); clf;
    plot(t_trial, rpm_trial, 'b.-', 'LineWidth', 1.5); hold on;
    yline(ss_rpm, 'r--', 'LineWidth', 1.5, 'Label', 'Steady State');
    yline(target, 'g--', 'LineWidth', 1.5, 'Label', '63.2%');
    xline(tau_vals(i), 'k--', 'LineWidth', 1.5, ...
        'Label', ['\tau = ', num2str(tau_vals(i),2), 's']);
    xlabel('Time (s)');
    ylabel('RPM');
    title(['Trial ', num2str(i), ' - ', num2str(voltages(i)), 'V Step Input']);
    legend('Measured RPM', 'Steady State', '63.2% of SS', 'Tau', ...
        'Location', 'southeast');
    grid on;
end

% Summary statistics 
K_mean   = mean(K_vals);
K_std    = std(K_vals);
tau_mean = mean(tau_vals);
tau_std  = std(tau_vals);

fprintf('\n========== RESULTS ==========\n');
fprintf('Trial |  Voltage |  SS RPM  |    K    |   tau  \n');
fprintf('------+----------+----------+---------+--------\n');
for i = 1:5
    idx = abs(voltage - voltages(i)) < 0.01;
    ss  = mean(rpm(idx & time > 5));
    fprintf('  %d   |  %.3fV  |  %.1f  |  %.3f  |  %.3f\n', ...
        i, voltages(i), ss, K_vals(i), tau_vals(i));
end
fprintf('\nMean K   = %.4f RPM/V  (std = %.4f)\n', K_mean, K_std);
fprintf('Mean tau = %.4f s      (std = %.4f)\n', tau_mean, tau_std);
fprintf('\nIdentified Transfer Function:\n');
fprintf('G(s) = %.4f / (%.4fs + 1)\n', K_mean, tau_mean);


%% CHIRP ANALYSIS

% Read chirp data
opts2 = detectImportOptions('chirp_data.csv', 'CommentStyle', '#');
chirp_data = readmatrix('chirp_data.csv', opts2);

t_chirp   = chirp_data(:,1);
v_chirp   = chirp_data(:,2);
rpm_chirp = chirp_data(:,3);

% Simulate predicted response using identified TF 
sys = tf(K_mean, [tau_mean 1]);
rpm_predicted = lsim(sys, v_chirp, t_chirp);

% Plot measured vs predicted 
figure(6); clf;
plot(t_chirp, rpm_chirp, 'b.-', 'LineWidth', 1.5, 'DisplayName', 'Measured');
hold on;
plot(t_chirp, rpm_predicted, 'r-', 'LineWidth', 2, 'DisplayName', 'Predicted');
xlabel('Time (s)');
ylabel('RPM');
title('Chirp Response: Measured vs Predicted');
legend('Location', 'best');
grid on;

% Error analysis
error     = rpm_chirp - rpm_predicted;
rmse      = sqrt(mean(error.^2));
max_err   = max(abs(error));

figure(7); clf;
plot(t_chirp, error, 'k.-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Error (RPM)');
title('Chirp Error: Measured - Predicted');
grid on;

fprintf('\n========== CHIRP ERROR ANALYSIS ==========\n');
fprintf('RMSE          = %.4f RPM\n', rmse);
fprintf('Max Error     = %.4f RPM\n', max_err);
fprintf('Mean Error    = %.4f RPM\n', mean(error));