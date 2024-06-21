% Yildiz Navigation Extended Kalman Filter algorithm for IMU Sensor
%
% Authors:
% Emre Emir Fidan
% Muhammed Yavuz Hanege
% Mehmet Emre Eyvaz
% Kerem Vatansever
%
%
% YILDIZ TECHNICAL UNIVERSITY 
%
%

clc
close all
format long

R2D = 180/pi;

%% Data Extraction
fileID = fopen('gyrotest.txt', 'r');
IMU_DATA = fscanf(fileID, '%f', [10, Inf]); % AccX_raw, AccY_raw, AccZ_raw, GyroX_raw, GyroY_raw, GyroZ_raw, MagX_raw, MagY_raw, MagZ_raw, Time(ms)
fclose(fileID);
N = size(IMU_DATA, 2);
Nsamples = N - 1;
EulerSaved = zeros(Nsamples, 3);

%% INITIALIZING
g = 9.8;
ref_mag = 50;
N_Q = 0.01;
N_R = 100;
N_P = 1;

% Logged data is inversed. Fix the sign of data

% Fix the sign of logged data
IMU_DATA(2, :) = -IMU_DATA(2, :);
IMU_DATA(4:6, :) = -IMU_DATA(4:6, :);
IMU_DATA(10, :) = IMU_DATA(10, :) / 1000;


%% Set Reference Magnetic Vector (Normalization)
M = sqrt(sum(IMU_DATA(7:9, ref_mag) .^ 2));
B = IMU_DATA(7:9, ref_mag) / M;

%% Gyro To Quaternion Init
qgyro = [1, 0, 0, 0]';
gyroEulerList = zeros(Nsamples, 3);

%% Extended Kalman Filter Algorithm
for k = 1:Nsamples-1
    % Assignments
    ax = IMU_DATA(1, k); ay = IMU_DATA(2, k); az = IMU_DATA(3, k);
    p = IMU_DATA(4, k); q = IMU_DATA(5, k); r = IMU_DATA(6, k);
    mx = IMU_DATA(7, k); my = IMU_DATA(8, k); mz = IMU_DATA(9, k);
    dt = IMU_DATA(10, k+1) - IMU_DATA(10, k);
    
    % Normalization
    G = sqrt(ax^2 + ay^2 + az^2);
    M = sqrt(mx^2 + my^2 + mz^2);
    ax = ax / G; ay = ay / G; az = az / G;
    mx = mx / M; my = my / M; mz = mz / M;
    
    % Raw Gyro Data to Quaternion
    Fq = Fjacob(p, q, r, dt);
    qgyro = Fq * qgyro;

    euler_ang = quat2eul(qgyro');
    phiGyro = euler_ang(:,1);
    thetaGyro = euler_ang(:,2);
    psiGyro = euler_ang(:,3);
    gyroEulerList(k, :) = [phiGyro, thetaGyro, psiGyro] * R2D;
    
    % Extended Kalman Filter Function
    [q0, q1, q2, q3] = EKF(p, q, r, B, mx, my, mz, ax, ay, az, dt, N_Q, N_R, N_P);
    
    % Save EKF output to list
    EulerSaved(k, :) = quat2eul([q0, q1, q2, q3]);
end

%% Final List Initialization
EulerSaved(Nsamples+1, :) = [0, 0, 0];
gyroEulerList(Nsamples+1, :) = [0, 0, 0];


%% Convert Radians to Degrees for Roll, Pitch, Yaw
PhiSaved = EulerSaved(:, 1) * R2D;
ThetaSaved = EulerSaved(:, 2) * R2D;
PsiSaved = EulerSaved(:, 3) * R2D;

%% Plot Results
plotResults(IMU_DATA,PhiSaved,ThetaSaved,PsiSaved,gyroEulerList);

%% 3D Animation of IMU State
animateIMUState(IMU_DATA,PhiSaved,ThetaSaved,PsiSaved,Nsamples);