#pragma once

#include <QObject>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <xviz/robot/urdf_robot.hpp>

class RobotViewer ;

class RobotListener : public QObject, public rclcpp::Node
{
    Q_OBJECT
public:
    RobotListener(const std::string &urdf_path, RobotViewer *viewer);

signals:
  //  void setupRobot(const std::shared_ptr<xviz::URDFRobot> &robot) ;
    void setupRobot() ;
private:
    /// The last time a joint state message was received
    rclcpp::Time last_callback_time_;

    /// A map between a joint name and the last time its state was published
    std::map<std::string, builtin_interfaces::msg::Time> last_publish_time_;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state);

    void updateTransforms(const std::map<std::string, double> &joint_positions, const builtin_interfaces::msg::Time & time);
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    RobotViewer *viewer_ ;

    std::string target_frame_ ;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

Q_DECLARE_METATYPE(std::shared_ptr<xviz::URDFRobot>)
