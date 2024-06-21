function plotResults(IMU_DATA, PhiSaved, ThetaSaved, PsiSaved, gyroEulerList)
    figure;
    plot(IMU_DATA(10, :), PhiSaved, 'r', IMU_DATA(10, :), ThetaSaved, 'b', IMU_DATA(10, :), PsiSaved, 'g');
    hold on;
    plot(IMU_DATA(10, :), gyroEulerList(:, 1), 'c', IMU_DATA(10, :), gyroEulerList(:, 2), 'm', IMU_DATA(10, :), gyroEulerList(:, 3), 'y');
    refline([0 0]);
    title('Euler Angles (degrees)');
    xlabel('Time (s)');
    ylabel('Angle (degrees)');
    legend({'Phi', 'Theta', 'Psi', 'Gyro Phi', 'Gyro Theta', 'Gyro Psi'}, 'Location', 'northwest');

    figure;
    plot(IMU_DATA(10, :), IMU_DATA(1:3, :));
    refline([0 0]);
    title('Acceleration (m/s^2)');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    legend({'AccX', 'AccY', 'AccZ'}, 'Location', 'northwest');

    figure;
    plot(IMU_DATA(10, :), IMU_DATA(4:6, :));
    refline([0 0]);
    title('Angular Velocity (rad/s)');
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    legend({'GyroX', 'GyroY', 'GyroZ'}, 'Location', 'northwest');

    figure;
    plot(IMU_DATA(10, :), IMU_DATA(7:9, :));
    refline([0 0]);
    title('Magnetic Flux Density (uT)');
    xlabel('Time (s)');
    ylabel('Magnetic Flux Density (uT)');
    legend({'MagX', 'MagY', 'MagZ'}, 'Location', 'northwest');
end
