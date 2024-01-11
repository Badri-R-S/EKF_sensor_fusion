# Extended Kalman Filter and Sensor Fusion for 2-D position and velocity tracking
In this project, an Extended Kalman Filter and Sensor Fusion were utilized to estimate the state of a moving object of interest (bicycle) with noisy lidar and radar measurements. 
The available sensors were LIDAR and RADAR. LIDAR produces point cloud output from which the position of the bicycle can be found. RADAR uses the Doppler effect to give velocity output and the radial position.
The performance of the filter has been measured using the RMSE error metric. It has been measured between ground truth and the predicted state at each step.

## Workflow
The workflow for this project can be described as shown in the image:

<p align="center">
<img width="600" height="400" src="https://github.com/Badri-R-S/EKF_sensor_fusion/blob/main/Results/kalman_filter_map.png"
</p> 

### Point to note:
- For LIDAR measurements, the usual Kalman filter equations were used.
- Since the measurement model for the RADAR is non-linear (it sees the world differently), the Extended Kalman FIlter has been used.
- It was achieved by approximating the non-linear measurement matrix H using first-order Taylor series expansion.
- The measurement covariance matrix for RADAR is 3x3 since RADAR measures three quantities (unlike LIDAR which measures the x and y position directly) namely, Range (rho), Bearing (phi), Radial velocity (rho_dot).

## Folder structure
- The **src** folder contains the code that runs the Kalman filter.
- **main.cpp** - communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, and calls a function to calculate RMSE.
- **FusionEKF.cpp** - initializes the filter, calls the predict function, and calls the update function.
- **kalman_filter.cpp**- defines the predict function, the update function for lidar, and the update function for radar.
- **tools.cpp**- function to calculate RMSE and the Jacobian matrix.

## Steps to run the code
- The simulator can be downloaded from this link : [Simulator](https://github.com/udacity/self-driving-car-sim/releases/)
- The dependencies can be installed by simply running the **install-linux.sh** file that has been provided in the repository.
- You can build the workspace by running the following commands from the root of the workspace run:
  -  **mkdir build && cd build**
  -  **cmake .. && make**
- To run the program, make sure to have opened the simulator and have built the workspace. Then run **./ExtendedKF** in the build directory

## Results:
- The Lidar measurements are shown as red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles.
<p align="center">
<img width="600" height="400" src="https://github.com/Badri-R-S/EKF_sensor_fusion/blob/main/Results/Screenshot%20from%202024-01-11%2008-39-16.png"
</p> 
  
- the full video can be viewed at : [Result](https://drive.google.com/file/d/1neUnBMtkMTQONFciqeQmb8F5wN_0QcJu/view?usp=sharing)
