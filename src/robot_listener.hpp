#pragma once

#include <QObject>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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
};

Q_DECLARE_METATYPE(std::shared_ptr<xviz::URDFRobot>)
