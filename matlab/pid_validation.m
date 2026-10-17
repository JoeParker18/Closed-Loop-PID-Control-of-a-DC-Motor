% Load Arduino data
data = readmatrix('Project2_data.csv', 'CommentStyle', '#');
time_ms = data(:,1);
target = data(:,2);
rpm_actual = data(:,3);
pwm = data(:,4);

% Convert time to seconds starting from zero
time_s = (time_ms - time_ms(1)) / 1000;

% Define your transfer function and PID controller
G = tf(31.2375, [0.159 1]);
C = pid(2, 15, 0.0);
sys_closed = feedback(C * G, 1);

% Simulate step response to 240 RPM
[rpm_sim, t_sim] = step(240 * sys_closed, max(time_s));

% Plot comparison
figure;
plot(time_s, rpm_actual, 'b', 'LineWidth', 1.5);
hold on;
plot(t_sim, rpm_sim, 'r--', 'LineWidth', 1.5);
yline(240, 'k:', 'Target');
xlabel('Time (s)');
ylabel('RPM');
title('PID Controller: Actual vs Simulated Response');
legend('Actual (Arduino)', 'Simulated (MATLAB)', 'Target');
grid on;

% Compute error
rpm_sim_interp = interp1(t_sim, rpm_sim, time_s, 'linear', 'extrap');
error = rpm_actual - rpm_sim_interp;

figure;
plot(time_s, error, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('RPM Error (Actual - Simulated)');
title('Error Between Actual and Simulated Response');
grid on;