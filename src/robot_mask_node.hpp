#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include <xviz/gui/offscreen.hpp>
#include <xviz/scene/renderer.hpp>
#include <xviz/robot/robot_scene.hpp>

class RobotMaskNode:  public rclcpp::Node {
public:
    RobotMaskNode(const std::string &urdf_path);

    void setupPCL() ;

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state);
    void updateTransforms(const std::map<std::string, double> &joint_positions, const builtin_interfaces::msg::Time & time);
    void publishCameraFrame() ;
    void setupModel(const std::string &model_path) ;
   
    sensor_msgs::msg::CameraInfo getCameraInfo();

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_pub_ ;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_ ;
     std::map<std::string, builtin_interfaces::msg::Time> last_publish_time_;
  
    rclcpp::Time last_callback_time_;
    std::string target_frame_, prefix_ ;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ ;
    xviz::RobotScenePtr rscene_;
    xviz::NodePtr scene_ ;

    xviz::URDFRobot robot_ ;
    std::unique_ptr<xviz::OffscreenSurface> offscreen_ ;
    xviz::Renderer renderer_ ;
    Eigen::Isometry3f camera_tr_ ;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    xviz::CameraPtr pcam_ ;
    geometry_msgs::msg::TransformStamped transform_;

    float width_ = 1280, height_ = 720, yfov_ = 43 ;//58 ;

};


