function [xhat, P] = ekf(z, xhat_curr, P_curr, blue, A)
    % Filter Properties
    ts = 0.01;
    Qj = [(1/5)*ts^5, (1/4)*ts^4, (1/3)*ts^3;
          (1/4)*ts^4, (1/3)*ts^3, (1/2)*ts^2;
          (1/3)*ts^3, (1/2)*ts^2,        ts];
    Q = blkdiag(Qj, Qj, Qj);
    R = diag([1e-1 1e-2 1e-2]);

    % Predict
    x_mdl = A * xhat_curr;
    P_mdl = A * P_curr * A' + Q;

    % Set Rk
    dist = norm([x_mdl(1); x_mdl(4); x_mdl(7)] - [blue(1:3)]);
    Rk = diag([R(1, 1) + dist * (R(2, 2)^2 + R(3, 3)^2) / 2, R(2, 2), R(3, 3)]);

    % Residuals and Gains
    y = z - h(x_mdl, blue);
    blue(4:7) = eul2quat(blue(4:6)');
    H = H_fcn(blue(1), blue(2), blue(3), ...
              blue(4), blue(5), blue(6), blue(7), ...
              x_mdl(1), x_mdl(4), x_mdl(7));
    S = H * P_mdl * H' + Rk;
    K = P_mdl * H' / S;

    % Update
    xhat = x_mdl + K * y;
    P = (eye(9) - K * H) * P_mdl;
end