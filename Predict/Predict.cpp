#include "Predict.h"
#include "CVRM2022-sust.h"

#include <opencv2/core/eigen.hpp>

#include "KalmanFilter.h"




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

extern struct SerialPortRx g_serial_rx;
Eigen::Vector3d last_world_coordinates;
static uint32_t count = 0;
// time_t last_time;
void PredictHandle() {
    Eigen::Vector3d camera_coordinates;
    Eigen::Vector3d world_coordinates;
    Eigen::Vector3d predict_world_coordinates;
    Eigen::Vector3d predict_camera_coordinates;
    cv::Point predict_pixel_coordinates;

    int stateSize = 9;
    int measSize = 3;
    int controlSize = 0;
    float T = 0.1;
    KalmanFilter kf(stateSize, measSize, controlSize);

    Eigen::MatrixXd A(stateSize, stateSize);
        A<< 1, 0, 0, T, 0, 0, 1 / 2 * T*T, 0, 0,
            0, 1, 0, 0, T, 0, 0, 1 / 2 * T*T, 0,
            0, 0, 1, 0, 0, T, 0, 0, 1 / 2 * T*T,
            0, 0, 0, 1, 0, 0, T, 0, 0,
            0, 0, 0, 0, 1, 0, 0, T, 0,
            0, 0, 0, 0, 0, 1, 0, 0, T,
            0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1, 0, 0, 1;

    Eigen::MatrixXd H(measSize, stateSize);
    H<< 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0;

    Eigen::MatrixXd P(stateSize, stateSize);
    P.setIdentity();
    Eigen::MatrixXd R(measSize, measSize);
    R.setIdentity()*0.01;
    Eigen::MatrixXd Q(stateSize, stateSize);
    Q.setIdentity()*0.001;
    Eigen::VectorXd x(stateSize);
    Eigen::VectorXd u(0);
    Eigen::VectorXd z(measSize);
    z.setZero();
    Eigen::VectorXd res(stateSize);

    x << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    kf.init(x, P, R, Q);

    std::vector<cv::Point2d> aim_point(4);
    while(true) {
        Eigen::Matrix3d RotationMatrix =EulerAngleToRotationMatrix(g_serial_rx.roll, g_serial_rx.pitch, g_serial_rx.yaw);
        if(g_aim.is_find_arm) {
            pthread_mutex_lock(&g_aim_mutex);
            int x = g_aim.aim_rect.x;
            int y = g_aim.aim_rect.y;
            int width = g_aim.aim_rect.width;
            int height = g_aim.aim_rect.height;
            pthread_mutex_unlock(&g_aim_mutex);
            aim_point[0] = cv::Point2d(x, y);
            aim_point[1] = cv::Point2d(x + width, y);
            aim_point[2] = cv::Point2d(x + width, y + height);
            aim_point[3] = cv::Point2d(x, y + width);
            // g_serial_tx.x_offset = x + width/2 - g_config_info.aim_point.x;
            // g_serial_tx.y_offset = y + height/2 - g_config_info.aim_point.y;
            // g_serial_tx.shooting = 1;
            camera_coordinates = PixelCoordinatesToCameraCoordinates(aim_point, g_config_info.camera_config_dir);
            // std::cout<<g_serial_rx.roll<<" "<<g_serial_rx.pitch<<" "<<g_serial_rx.yaw<<std::endl;
            // std::cout<<atan(world_coordinates(0,0)/world_coordinates(2,0))<<" "<<atan(world_coordinates(1,0)/world_coordinates(2,0))<<std::endl;
            world_coordinates = CameraCoordinatesToWorldCoordinates(camera_coordinates, RotationMatrix);
            // std::cout << "world_coordinates " << world_coordinates(0,0)<<" "<<world_coordinates(1,0)<<" "<<world_coordinates(2,0)<< std::endl;
            // predict_world_coordinates = (world_coordinates - last_world_coordinates) * 20 + world_coordinates;
            // last_world_coordinates = predict_world_coordinates;

            kf.predict(A);
            z << world_coordinates(0,0), world_coordinates(1,0), world_coordinates(2,0);
            res << kf.update(H,z);
            // std::cout << world_coordinates(0,0) << " " <<world_coordinates(1,0) << " " << world_coordinates(2,0) 
            // << " " << res[0] << " " << res[1] << " " << res[2]
            // << " " << res[3] << " " << res[4] << " " << res[5]
            // << " " << res[6] << " " << res[7] << " " << res[8] << std::endl;

            predict_world_coordinates(0,0) = res[0];
            predict_world_coordinates(1,0) = res[1];
            predict_world_coordinates(2,0) = res[2];

            predict_camera_coordinates = WorldCoordinatesToCameraCoordinates(predict_world_coordinates, RotationMatrix);
            predict_pixel_coordinates = CameraCoordinatesToPixelCoordinates(camera_coordinates);
            g_serial_tx.x_offset = predict_pixel_coordinates.x - g_config_info.aim_point.x;
            g_serial_tx.y_offset = predict_pixel_coordinates.y - g_config_info.aim_point.y;
            if (count != g_aim.count) {
                count = g_aim.count;
                g_serial_tx.x_offset = std::max( g_serial_tx.x_offset - 0.01, 0.0);
                g_serial_tx.y_offset = std::max( g_serial_tx.y_offset - 0.01, 0.0);
            }
            g_serial_tx.shooting = 1;
            // std::cout << "predict_pixel_coordinates " << predict_pixel_coordinates.x - (x + width/2)<<" "<<predict_pixel_coordinates.y - (y + height/2)<< std::endl;
        }else {
            g_serial_tx.shooting = 0;
        }
    }
}