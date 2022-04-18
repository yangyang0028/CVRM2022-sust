#include "Predict.h"

#include <opencv2/core/eigen.hpp>

static const std::vector <cv::Point3d> g_ObjectPoints = {  // 单位：m
    {-0.0675, 0.0625,  0.},
    {0.0675, 0.0625, 0.},
    {0.0675,  -0.0625, 0.},
    {-0.0675, -0.0625,  0.}
};

static cv::Mat g_CameraMatrix, g_DistCoeffs;
static cv::Mat g_Rvec, g_Tvec;

Eigen::Matrix3d EulerAngleToRotationMatrix(float roll, float pitch, float yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q.toRotationMatrix();
}

Eigen::Vector3d PixelCoordinatesToCameraCoordinates(std::vector <cv::Point2d> image_point, std::string camera_parameters_file){
    Eigen::Vector3d camera_coordinates;
    cv::FileStorage fs(camera_parameters_file, cv::FileStorage::READ);
    fs["Camera_Matrix"] >> g_CameraMatrix;
    fs["Distortion_Coefficients"] >> g_DistCoeffs;

    cv::solvePnP(g_ObjectPoints, image_point, g_CameraMatrix, g_DistCoeffs, g_Rvec, g_Tvec);
    cv::cv2eigen(g_Tvec, camera_coordinates);
    return camera_coordinates;
}

Eigen::Vector3d CameraCoordinatesToWorldCoordinates(const Eigen::Vector3d camera_coordinates, const Eigen::Matrix3d rotation_matrix){
    return (rotation_matrix.transpose()) * camera_coordinates;
}

Eigen::Vector3d WorldCoordinatesToCameraCoordinates(const Eigen::Vector3d world_coordinates, const Eigen::Matrix3d rotation_matrix){
    return rotation_matrix * world_coordinates;
}

cv::Point CameraCoordinatesToPixelCoordinates(const Eigen::Vector3d &camera_coordinates) {
    Eigen::Matrix3d camera_matrix;
    cv::cv2eigen(g_CameraMatrix, camera_matrix);
    Eigen::Vector3d image_matrix = camera_matrix * camera_coordinates / camera_coordinates(2, 0);
    return (cv::Point){int(image_matrix(0, 0)), int(image_matrix(1, 0))};
}