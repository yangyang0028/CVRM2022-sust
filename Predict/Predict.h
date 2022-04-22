#ifndef PREDICT_H
#define PREDICT_H

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>

Eigen::Matrix3d EulerAngleToRotationMatrix(float roll, float pitch, float yaw);

Eigen::Vector3d PixelCoordinatesToCameraCoordinates(std::vector <cv::Point2d> image_point, std::string camera_parameters_file);
Eigen::Vector3d CameraCoordinatesToWorldCoordinates(const Eigen::Vector3d camera_coordinates, const Eigen::Matrix3d rotation_matrix);

Eigen::Vector3d WorldCoordinatesToCameraCoordinates(const Eigen::Vector3d world_coordinates, const Eigen::Matrix3d rotation_matrix);
cv::Point CameraCoordinatesToPixelCoordinates(const Eigen::Vector3d &camera_coordinates);

void PredictHandle();

#endif