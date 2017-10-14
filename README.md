## Term 2 - Project 1 : Extended Kalman Filter  
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

In this project, my goal was to use the techniques and code examples presented in the Udacity course to estimate the position of a car using sensor fusion algorithm and provided Lidar/radar sensor data.  

The goals / steps of this project are the following:
- Complete the Sensor Fusion Algorithm 
- Achieve a RSME value <[.11, .11, 0.52, 0.52].
- Test Estimation RSME 
  - Using only Radar data
  - Using only Laser Data
[//]: # (Image References)

[image1]: ./Output/Dataset_1-BothSensors.jpg "Position Estimation - Using Sensor Fusion - EKF"
[image2]: ./Output/Dataset_2-BothSensors.jpg "Position Estimation - Using Sensor Fusion - EKF"
[image3]: ./Output/Dataset_1-LaserUpdateOnly.jpg "Position Estimation - Laser KF"
[image4]: ./Output/Dataset_2-LaserUpdateOnly.jpg "Position Estimation - Laser KF"
[image5]: ./Output/Dataset_1-RadarUpdateOnly.jpg "Position Estimation - Radar EKF"
[image6]: ./Output/Dataset_2-RadarUpdateOnly.jpg "Position Estimation - Radar EKF"
## Final Results
From Dataset 1 & 2 - Sensor Fusion, achieved good position tracking accuracy.

![alt text][image1]
![alt text][image2]

From Dataset 1 & 2 - Laser KF Only, achieved good position tracking accuracy, but not as good as the sensor fusion implementation.

![alt text][image3]
![alt text][image4]

From Dataset 1 & 2 - Radar EKF Only, achieved position tracking reasonably well, but not as good as the sensor fusion implementation or the Laser Kf Implementation. Which considering each sensor's Measurement noise, was the expected behavior. 

![alt text][image5]
![alt text][image6]
