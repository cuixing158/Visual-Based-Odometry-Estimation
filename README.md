
# Visual-Based Odometry Estimation

[![Sponsor](https://img.shields.io/badge/-Sponsor-red?style=for-the-badge&logo=GitHub-Sponsors&logoColor=white)](https://raw.githubusercontent.com/cuixing158/OpticalFlow-Visualization/refs/heads/main/README_media/sponsors.jpg)

[![View Visual-Based-Odometry-Estimation on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/171774-visual-based-odometry-estimation)

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=cuixing158/Visual-Based-Odometry-Estimation&file=VisualBasedOdometryEstimation.mlx)

![HDmap](./images/HDmap.jpg)

Concise MATLAB code for stitching panoramic images from four fisheye cameras in a vehicle's surround view system. For more information, please refer to [VisualBasedOdometryEstimation.md](./VisualBasedOdometryEstimation.md) or simply click [![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=cuixing158/Visual-Based-Odometry-Estimation&file=VisualBasedOdometryEstimation.mlx) for out-of-the-box usage!

## Features

- Real-time application construction without the need to predefine the map size
- Pixel maps correspond directly to physical maps, facilitating easy query and localization
- Based on pure image algorithm only (no IMU, GPS, WheelEncoder, etc. to assist).
- Supports embedded C/C++ code generation
