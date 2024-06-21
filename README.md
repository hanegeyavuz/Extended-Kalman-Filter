# Yildiz Navigation Extended Kalman Filter (EKF) for IMU Sensors

## Overview

This repository contains MATLAB code implementing an Extended Kalman Filter (EKF) for processing Inertial Measurement Unit (IMU) data. The EKF algorithm is used to estimate the orientation of a sensor by fusing data from accelerometers, gyroscopes, and magnetometers. This project was developed at Yildiz Technical University as part of [YildizNAV Project](https://github.com/hanegeyavuz/YildizNAV).


## Features

- **IMU Data Fusion**: Combines accelerometer, gyroscope, and magnetometer data to estimate orientation.
- **Quaternion Representation**: Uses quaternions to represent orientation internally, avoiding gimbal lock and providing smooth rotation.
- **Real-time Visualization**: Visualizes orientation estimates in real-time using MATLAB plots and animations.
- **Modular Code Structure**: Code is organized into separate functions for the EKF, Jacobian calculations, and data visualization.

## Files and Directory Structure

- `main.m`: Main script to initialize data, run the EKF, and visualize results.
- `EKF.m`: Function implementing the Extended Kalman Filter.
- `Fjacob.m`: Function to compute the Jacobian of the state transition matrix.
- `Hjacob.m`: Function to compute the Jacobian of the measurement matrix.
- `animateIMUState.m`: Function to animate the IMU state in a 3D plot.
- `plotIMUData.m`: Function to plot IMU data and EKF results.
- `gyrotest.txt`: Example IMU data file used for testing.

## Getting Started

### Prerequisites

- MATLAB (any recent version should work)
- Basic understanding of IMU sensors and Kalman filters

### Running the Code

1. Clone the repository to your local machine:
   ```sh
   git clone https://github.com/hanegeyavuz/Yildiz-Navigation-EKF-IMU.git
   cd Yildiz-Navigation-EKF-IMU
2. Open MATLAB and navigate to the project directory.
3. Run the `main.m` script:


## Description
- This project processes IMU data to estimate the orientation of a device using an Extended Kalman Filter. The data is read from a text file, normalized, and then used to compute orientation in the form of Euler angles, which are subsequently visualized using MATLAB plots.

## Contributing
- Contributions are welcome! If you have any suggestions or find any bugs, please open an Issue or submit a Pull Request.

## License
This project is licensed under the MIT License - see the `LICENSE` file for details.

## Authors
- Emre Emir Fidan
- Muhammed Yavuz Hanege
- Mehmet Emre Eyvaz
- Kerem Vatansever

## Acknowledgements
- Yildiz Technical University for supporting this project.
- All contributors and users for their valuable feedback and suggestions.
