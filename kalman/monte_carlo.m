%% Init
tf = 10;
ts = 0.01;
N = tf / ts; % number of time steps
kspan = 1:1:N+1; % starts at 1 b/c of MATLAB indexing
t = (kspan - 1) .* ts; % time vector
samples = 10;

%% Tracking Model
Ax = [1, ts, 0.5*ts^2;
      0,  1,       ts;
      0,  0,        1];
A = [Ax, zeros(size(Ax)), zeros(size(Ax));
     zeros(size(Ax)), Ax, zeros(size(Ax));
     zeros(size(Ax)), zeros(size(Ax)), Ax];

%% Simulate
s_hist = zeros(3, N+1); % save sigma history

% start plot
clf;
% figure();
s_min = [1; 1; 1] .* 1e-6;
s_cell = cell(1, 3);

for sample = 1:1:samples
    xhat_hist = zeros(9, N+1); % saving estimated trajectory
    red_hist = zeros(9, N+1); % saving red team trajectory
    blue_hist = zeros(12, N+1); % saving blue team trajectory

    % initial states
    red = [5 0 0 6 1 0 -11 1 0]'; % [x xdot xddot y ydot yddot z zdot zddot]
    blue = [-5 0 0 0 0 0 0 0 0 0 0 0]'; % [x y z phi theta psi xdot ydot zdot omega1 omega2 omega3]
    P = diag([1 1 1 1 1 1 1 1 1] .* 1e-2); % guess
    xhat = mvnrnd(red, P)'; % sample
    
    % start histories
    xhat_hist(:, 1) = xhat;
    red_hist(:, 1) = red;
    blue_hist(:, 1) = blue;
    s_hist(:, 1) = sqrt([P(1, 1); P(4, 4); P(7, 7)]);
    for k = 2:1:N+1 % run through simulation
        % update true states
        % blue is constant for now, but you can update it here
        red = A * [red(1:2); -cos(k * ts); red(4:5); -2*sin(k * ts * 2); red(7:9)];
    
        % take a measurement
        z = h(red, blue) + [randn randn randn]' .* [1e-1 1e-2 1e-2]';
        
        % estimate red team position
        [xhat, P] = ekf(z, xhat, P, blue, A); % EKF
    
        % save data
        xhat_hist(:, k) = xhat;
        red_hist(:, k) = red;
        blue_hist(:, k) = blue;
        s_hist(:, k) = sqrt([P(1, 1); P(4, 4); P(7, 7)]);
    end

    % Save average lowest sigma
    s_mean = [mean(s_hist(1, :)); mean(s_hist(2, :)); mean(s_hist(3, :))];
    for c = 1:1:3
        if s_mean(c) > s_min(c)
            s_min(c) = s_mean(c);
            s_cell{c} = s_hist(c, :);
        end
    end

    % Plot History
    subplot(3, 1, 1); hold on;
    plot(t, xhat_hist(1, :) - red_hist(1, :), '-', 'LineWidth', 1);
    subplot(3, 1, 2); hold on;
    plot(t, xhat_hist(4, :) - red_hist(4, :), '-', 'LineWidth', 1);
    subplot(3, 1, 3); hold on;
    plot(t, xhat_hist(7, :) - red_hist(7, :), '-', 'LineWidth', 1);
end

%% Bounds
y_limits = [-1 1] .* 0.25;
subplot(3, 1, 1); hold on;
title('3\sigma bounds for X');
plot(t, 3 .* s_cell{1}, 'r-', 'LineWidth', 1);
ax = plot(t, -3 .* s_cell{1}, 'r-', 'LineWidth', 1);
% xlabel('Time [s]');
ylabel('X Error [m]');
legend(ax, '3\sigma Bound');
ylim(y_limits);
grid on;

subplot(3, 1, 2); hold on;
title('3\sigma bounds for Y');
plot(t, 3 .* s_cell{2}, 'r-', 'LineWidth', 1);
ax = plot(t, -3 .* s_cell{2}, 'r-', 'LineWidth', 1);
legend(ax, '3\sigma Bound');
% xlabel('Time [s]');
ylabel('Y Error [m]');
ylim(y_limits);
grid on;

subplot(3, 1, 3); hold on;
title('3\sigma bounds for Z');
plot(t, 3 .* s_cell{3}, 'r-', 'LineWidth', 1);
ax = plot(t, -3 .* s_cell{3}, 'r-', 'LineWidth', 1);
legend(ax, '3\sigma Bound');
xlabel('Time [s]');
ylabel('Z Error [m]');
ylim(y_limits);
grid on;
