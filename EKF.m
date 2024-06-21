function [q0, q1, q2, q3] = EKF(p, q, r, B, mx, my, mz, ax, ay, az, dt, N_Q, N_R, N_P)
% Extended Kalman Filter for IMU data fusion
% Inputs:
%   p, q, r: Gyroscope measurements
%   B: Reference magnetic vector
%   mx, my, mz: Magnetometer measurements
%   ax, ay, az: Accelerometer measurements
%   dt: Time step
%   N_Q, N_R, N_P: Noise parameters
% Outputs:
%   q0, q1, q2, q3: Quaternion components

    persistent Q R x P firstRun
    
    if isempty(firstRun)
        Q = N_Q * eye(4);
        R = N_R * eye(6);
        x = [1; 0; 0; 0];
        P = N_P * eye(4);
        firstRun = 1;
    end
    
    % Predict step
    F = Fjacob(p, q, r, dt);
    xp = F * x;
    Pp = F * P * F' + Q;
    
    % Correct step
    H = Hjacob(x(1), x(2), x(3), x(4), B);
    S = H * Pp * H' + R;
    K = Pp * H' * (S \ eye(6));
    
    z = [ax; ay; az; mx; my; mz];
    x = xp + K * (z - H * xp);
    P = Pp - K * H * Pp;
    
    x_sc = sqrt(x(1)^2 + x(2)^2 + x(3)^2 + x(4)^2);
    q0 = x(1) / x_sc;
    q1 = x(2) / x_sc;
    q2 = x(3) / x_sc;
    q3 = x(4) / x_sc;
end
