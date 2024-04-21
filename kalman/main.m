%% Init
tf = 10;
ts = 0.01;
N = tf / ts; % number of time steps
kspan = 1:1:N+1; % starts at 1 b/c of MATLAB indexing
t = (kspan - 1) .* ts; % time vector

%% Tracking Model
Ax = [1, ts, 0.5*ts^2;
      0,  1,       ts;
      0,  0,        1];
A = [Ax, zeros(size(Ax)), zeros(size(Ax));
     zeros(size(Ax)), Ax, zeros(size(Ax));
     zeros(size(Ax)), zeros(size(Ax)), Ax];

%% Simulate
xhat_hist = zeros(9, N+1); % saving estimated trajectory
red_hist = zeros(9, N+1); % saving red team trajectory
blue_hist = zeros(12, N+1); % saving blue team trajectory

% initial states
red = [5 0 0 6 1 0 -11 1 0]'; % [x xdot xddot y ydot yddot z zdot zddot]
blue = [-5 0 0 0 0 0 0 0 0 0 0 0]'; % [x y z phi theta psi xdot ydot zdot omega1 omega2 omega3]
P = diag([1 1 1 1 1 1 1 1 1] .* 1e-2); % guess
xhat = mvnrnd(red(:, 1), P)'; % sample

% start histories
xhat_hist(:, 1) = xhat;
red_hist(:, 1) = red;
blue_hist(:, 1) = blue;

for k = 2:1:N+1 % run through simulation
    % update true states
    % blue is constant for now, but you can update it here
    
    % u = [2 2 2 2] .* -4.9050;
    % blue = blue + quadrotor(k, blue, u) .* ts;

    % blue = blue_hist(:, k);
    red = A * [red(1:2); -cos(k * ts); red(4:5); -2*sin(k * ts * 2); red(7:9)];

    % take a measurement
    z = h(red, blue) + [randn randn randn]' .* [1e-1 1e-2 1e-2]' .* 1;
    
    % estimate red team position
    [xhat, P, xhat_fwd, t_fwd] = ekf(z, xhat, P, blue, A); % EKF
    t_predict = t_fwd + k * ts;

    % save data
    xhat_hist(:, k) = xhat;
    red_hist(:, k) = red;
    blue_hist(:, k) = blue;
end

%% Plot

% figure();
% plot3(red_hist(1, :), red_hist(4, :), -red_hist(7, :), '-', 'LineWidth', 2); hold on;
% plot3(xhat_hist(1, :), xhat_hist(4, :), -xhat_hist(7, :), '--', 'LineWidth', 2); grid on;
% xlabel('X [m]');
% ylabel('Y [m]');
% zlabel('Z [m]');
% title('Trajectory');
% legend('True', 'Estimated', 'location', 'best');

figure();
subplot(3, 1, 1);
plot(t, red_hist(1, :), '-', 'LineWidth', 2); hold on;
plot(t, xhat_hist(1, :), '--', 'LineWidth', 2);
plot(t_predict, xhat_fwd(1, :), ':', 'LineWidth', 2); grid on;
xlabel('Time [s]');
ylabel('X [m]');
title('X Position');
legend('True', 'Estimated', 'location', 'best');
subplot(3, 1, 2);
plot(t, red_hist(4, :), '-', 'LineWidth', 2); hold on;
plot(t, xhat_hist(4, :), '--', 'LineWidth', 2);
plot(t_predict, xhat_fwd(4, :), ':', 'LineWidth', 2); grid on;
xlabel('Time [s]');
ylabel('Y [m]');
title('Y Position');
legend('True', 'Estimated', 'location', 'best');
subplot(3, 1, 3);
plot(t, -red_hist(7, :), '-', 'LineWidth', 2); hold on;
plot(t, -xhat_hist(7, :), '--', 'LineWidth', 2);
plot(t_predict, -xhat_fwd(7, :), ':', 'LineWidth', 2); grid on;
xlabel('Time [s]');
ylabel('-Z [m]');
title('-Z Position');
legend('True', 'Estimated', 'location', 'best');

% figure();
% subplot(3, 1, 1);
% title('X Position');
% plot(t, blue_hist(1, :), '-', 'LineWidth', 2); hold on;
% xlabel('Time [s]');
% ylabel('X');
% subplot(3, 1, 2);
% title('Y Position');
% plot(t, blue_hist(2, :), '-', 'LineWidth', 2); hold on;
% xlabel('Time [s]');
% ylabel('Y');
% subplot(3, 1, 3);
% title('-Z Position');
% plot(t, blue_hist(3, :), '-', 'LineWidth', 2); hold on;
% xlabel('Time [s]');
% ylabel('-Z');