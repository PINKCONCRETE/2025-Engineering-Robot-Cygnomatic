#include <memory>
#include <string>
#include <cstring>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "my_msg/msg/joint_info.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_policy;
typedef message_filters::Synchronizer<approximate_policy> approximate_synchronizer;

// 相机内参和畸变系数（使用你原来的参数）
double color_d[8] = {-1.1565371751785278, -0.6290106773376465, 3.856293187709525e-05,
                    -0.00038337879232130945, -0.11774921417236328, -1.139601469039917,
                    0.6050423979759216, -0.1085730716586113};
double color_k[9] = {305.68817138671875, 0.0, 320.0602111816406,
                    0.0, 305.6888122558594, 199.68258666992188,
                    0.0, 0.0, 1.0};

// 棋盘格参数
const int BOARD_WIDTH = 10;    // 横向角点数-1
const int BOARD_HEIGHT = 7;   // 纵向角点数-1
const float SQUARE_SIZE = 0.020f; // 格子大小，单位：米

class MinerVision : public rclcpp::Node
{
public:
    MinerVision()
        : Node("MinerVision"), 
        has_chessboard_pose_(false)
    {
        // 发布处理后的图像和姿态信息
        processed_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 1);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("chessboard_pose", 1);
        keyboard_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/keyboard_input", 10,
            std::bind(&MinerVision::keyboardCallback, this, std::placeholders::_1));

        // 订阅RGB和深度图像
        rgb_sub_.subscribe(this, "/camera/color/image_raw");
        depth_sub_.subscribe(this, "/camera/depth/image_raw");
        
        // 同步订阅
        sync_ = std::make_shared<approximate_synchronizer>(approximate_policy(10), rgb_sub_, depth_sub_);
        sync_->registerCallback(std::bind(&MinerVision::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

        // 初始化相机参数
        camera_matrix_ = cv::Mat(3, 3, CV_64FC1, color_k);
        dist_coeffs_ = cv::Mat(1, 8, CV_64FC1, color_d);

        tf_buffer_  = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 初始化棋盘格3D点
        for (int i = 0; i < BOARD_HEIGHT; ++i) {
            for (int j = 0; j < BOARD_WIDTH; ++j) {
                object_points_.push_back(cv::Point3f(j * SQUARE_SIZE, i * SQUARE_SIZE, 0));
            }
        }
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg, 
                      const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);
    void keyboardCallback(const std_msgs::msg::String::SharedPtr msg);
    cv::Vec3d rotationMatrixToEulerAngles(const cv::Mat &R);
    double rad2deg(double rad) { return rad * 180.0 / CV_PI; };

    // ROS订阅和发布
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    std::shared_ptr<approximate_synchronizer> sync_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_img_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_sub_;

    // TF组件
    // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    // tf2_ros::TransformListener tf_listener_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // 相机参数
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    cv::Mat current_R_target2cam_;
    cv::Mat current_t_target2cam_;
    bool has_chessboard_pose_;
    
    // 棋盘格3D点
    std::vector<cv::Point3f> object_points_;
};

cv::Vec3d MinerVision::rotationMatrixToEulerAngles(const cv::Mat &R) {
    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) + R.at<double>(1,0) * R.at<double>(1,0));
    
    bool singular = sy < 1e-6;
    
    double x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2,1), R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    
    return cv::Vec3d(x, y, z);
}

void MinerVision::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg, 
                               const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
{
    try {
        // 转换ROS图像消息为OpenCV格式
        cv_bridge::CvImagePtr cv_rgb = cv_bridge::toCvCopy(rgb_msg, "bgr8");
        cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);

        cv::Mat rgb_image = cv_rgb->image;
        cv::Mat depth_image = cv_depth->image;
        cv::Mat processed_image = rgb_image.clone();

        // 查找棋盘格角点
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(rgb_image, cv::Size(BOARD_WIDTH, BOARD_HEIGHT), corners,
                                             cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        
        if (found) {
            // 亚像素精确化
            cv::Mat gray;
            cv::cvtColor(rgb_image, gray, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            
            // 绘制角点
            cv::drawChessboardCorners(processed_image, cv::Size(BOARD_WIDTH, BOARD_HEIGHT), corners, found);
            
            // 计算姿态
            cv::Mat rvec, tvec;
            cv::solvePnP(object_points_, corners, camera_matrix_, dist_coeffs_, rvec, tvec);
            
            // 转换为旋转矩阵
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            
            // 获取欧拉角
            cv::Vec3d euler_angles = rotationMatrixToEulerAngles(R);
            
            // 绘制坐标系
            std::vector<cv::Point3f> axis_points = {
                cv::Point3f(0, 0, 0), 
                cv::Point3f(3*SQUARE_SIZE, 0, 0),
                cv::Point3f(0, 3*SQUARE_SIZE, 0), 
                cv::Point3f(0, 0, -3*SQUARE_SIZE)
            };
            std::vector<cv::Point2f> image_points;
            cv::projectPoints(axis_points, rvec, tvec, camera_matrix_, dist_coeffs_, image_points);
            
            // 绘制坐标系
            cv::line(processed_image, image_points[0], image_points[1], cv::Scalar(0, 0, 255), 2); // X轴 - 红色
            cv::line(processed_image, image_points[0], image_points[2], cv::Scalar(0, 255, 0), 2); // Y轴 - 绿色
            cv::line(processed_image, image_points[0], image_points[3], cv::Scalar(255, 0, 0), 2); // Z轴 - 蓝色
            
            // 显示姿态信息
            std::stringstream ss;
            ss << "Position (m): X=" << std::fixed << std::setprecision(3) << tvec.at<double>(0)
               << ", Y=" << tvec.at<double>(1) << ", Z=" << tvec.at<double>(2);
            cv::putText(processed_image, ss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            ss.str("");
            ss << "Rotation (deg): X=" << std::fixed << std::setprecision(1) << rad2deg(euler_angles[0])
               << ", Y=" << rad2deg(euler_angles[1]) << ", Z=" << rad2deg(euler_angles[2]);
            cv::putText(processed_image, ss.str(), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            // 打印到ROS日志
            RCLCPP_INFO(this->get_logger(), "Chessboard pose - Position: [%.3f, %.3f, %.3f], Rotation: [%.1f, %.1f, %.1f]",
                       tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2),
                       rad2deg(euler_angles[0]), rad2deg(euler_angles[1]), rad2deg(euler_angles[2]));

            // 发布姿态信息
            Eigen::Matrix3d rotation_matrix;
            cv::cv2eigen(R, rotation_matrix);
            Eigen::Quaterniond quaternion(rotation_matrix);

            geometry_msgs::msg::Pose pose_msg;
            pose_msg.position.x = tvec.at<double>(0);
            pose_msg.position.y = tvec.at<double>(1);
            pose_msg.position.z = tvec.at<double>(2);
            pose_msg.orientation.x = quaternion.x();
            pose_msg.orientation.y = quaternion.y();
            pose_msg.orientation.z = quaternion.z();
            pose_msg.orientation.w = quaternion.w();
            
            pose_pub_->publish(pose_msg);
        }

        // 发布处理后的图像
        cv_bridge::CvImage out_msg;
        sensor_msgs::msg::Image test_image;
        out_msg.header.stamp = this->now();
        out_msg.encoding = "bgr8";
        out_msg.image = processed_image;
        out_msg.toImageMsg(test_image);
        processed_img_pub_->publish(test_image);

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void MinerVision::keyboardCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == "1") {
        if (!has_chessboard_pose_) {
            RCLCPP_WARN(this->get_logger(), "No chessboard detected!");
            return;
        }

        try {
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_->lookupTransform(
                    "base_link", 
                    "end_link",
                    this->get_clock()->now()
                    // ros::Duration(1.0)
                );
            } catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "TF Error: %s", ex.what());
                return;
            }
            
            Eigen::Quaterniond quat(
                transform.transform.rotation.w,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z
            );
            Eigen::Matrix3d R_gripper2base = quat.toRotationMatrix();
            Eigen::Vector3d t_gripper2base(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            );

            cv::Mat cv_R_gripper2base, cv_t_gripper2base;
            cv::eigen2cv(R_gripper2base, cv_R_gripper2base);
            cv::eigen2cv(t_gripper2base, cv_t_gripper2base);

            std::ofstream file("calibration_data.txt", std::ios::app);
            if (file.is_open()) {
                // 保存机械臂位姿
                file << "R_gripper2base:\n";
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        file << cv_R_gripper2base.at<double>(i, j) << " ";
                    }
                    file << "\n";
                }
                file << "t_gripper2base:\n";
                for (int i = 0; i < 3; ++i) {
                    file << cv_t_gripper2base.at<double>(i) << " ";
                }
                file << "\n";

                // 保存标定板位姿
                file << "R_target2cam:\n";
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        file << current_R_target2cam_.at<double>(i, j) << " ";
                    }
                    file << "\n";
                }
                file << "t_target2cam:\n";
                for (int i = 0; i < 3; ++i) {
                    file << current_t_target2cam_.at<double>(i) << " ";
                }
                file << "\n";
                file.close();
                RCLCPP_INFO(this->get_logger(), "Data saved successfully.");
            }
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF Error: %s", ex.what());
        }
        has_chessboard_pose_ = false;

    } else if (msg->data == "2") {
        std::vector<cv::Mat> R_gripper2base, t_gripper2base, R_target2cam, t_target2cam;
        std::ifstream file("calibration_data.txt");
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open data file!");
            return;
        }

        std::string line;
        cv::Mat temp_R, temp_t;
        int row_cnt = 0;
        std::string section;

        while (std::getline(file, line)) {
            if (line.find("R_gripper2base:") != std::string::npos) {
                temp_R = cv::Mat(3, 3, CV_64F);
                row_cnt = 0;
                section = "R_gripper2base";
            } else if (line.find("t_gripper2base:") != std::string::npos) {
                temp_t = cv::Mat(3, 1, CV_64F);
                section = "t_gripper2base";
            } else if (line.find("R_target2cam:") != std::string::npos) {
                temp_R = cv::Mat(3, 3, CV_64F);
                row_cnt = 0;
                section = "R_target2cam";
            } else if (line.find("t_target2cam:") != std::string::npos) {
                temp_t = cv::Mat(3, 1, CV_64F);
                section = "t_target2cam";
            } else {
                std::istringstream iss(line);
                if (section == "R_gripper2base" || section == "R_target2cam") {
                    for (int j = 0; j < 3; ++j) {
                        double val;
                        iss >> val;
                        temp_R.at<double>(row_cnt, j) = val;
                    }
                    if (++row_cnt == 3) {
                        if (section == "R_gripper2base") R_gripper2base.push_back(temp_R.clone());
                        else R_target2cam.push_back(temp_R.clone());
                    }
                } else if (section == "t_gripper2base" || section == "t_target2cam") {
                    for (int j = 0; j < 3; ++j) {
                        double val;
                        iss >> val;
                        temp_t.at<double>(j) = val;
                    }
                    if (section == "t_gripper2base") t_gripper2base.push_back(temp_t.clone());
                    else t_target2cam.push_back(temp_t.clone());
                }
            }
        }
        file.close();

        if (R_gripper2base.size() != t_gripper2base.size() || 
            R_target2cam.size() != t_target2cam.size() ||
            R_gripper2base.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Data inconsistent!");
            return;
        }

        cv::Mat R_cam2gripper, t_cam2gripper;
        cv::calibrateHandEye(R_gripper2base, t_gripper2base, 
                            R_target2cam, t_target2cam,
                            R_cam2gripper, t_cam2gripper,
                            cv::CALIB_HAND_EYE_TSAI);

        // RCLCPP_INFO(this->get_logger(), "Hand-Eye Calibration Result:");
        // RCLCPP_INFO(this->get_logger(), "Rotation Matrix:\n%s", cv::format(R_cam2gripper, cv::Formatter::FMT_DEFAULT).c_str());
        // RCLCPP_INFO(this->get_logger(), "Translation Vector:\n%s", cv::format(t_cam2gripper, cv::Formatter::FMT_DEFAULT).c_str());
        std::cout << "Hand-Eye Calibration Result:" << std::endl;
        std::cout << "Rotation Matrix:" << std::endl;
        std::cout << cv::format(R_cam2gripper, cv::Formatter::FMT_DEFAULT) << std::endl;
        std::cout << "Translation Vector:" << std::endl;
        std::cout << cv::format(t_cam2gripper, cv::Formatter::FMT_DEFAULT) << std::endl;

    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinerVision>());
    rclcpp::shutdown();
    return 0;
}
