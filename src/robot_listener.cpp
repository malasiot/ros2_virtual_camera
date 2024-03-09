#include "robot_listener.hpp"
#include "robot_viewer.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <xviz/robot/urdf_robot.hpp>

using namespace std ;

RobotListener::RobotListener(const std::string &urdf_path, RobotViewer *viewer)
    : Node("minimal_subscriber"), viewer_(viewer)
{
    qRegisterMetaType<std::shared_ptr<xviz::URDFRobot> const &>();

    string urdf ;

    if ( !urdf_path.empty() ) {
        std::ifstream in(urdf_path, std::ios::in | std::ios::binary);
        if (in) {
            in.seekg(0, std::ios::end);
            urdf.resize(in.tellg());
            in.seekg(0, std::ios::beg);
            in.read(&urdf[0], urdf.size());
            in.close();

            // this->set_parameter(rclcpp::Parameter("robot_description", urdf));
        }

    } else {
        urdf = declare_parameter("robot_description", std::string());
    }

    if ( urdf.empty() ) {
        RCLCPP_ERROR(this->get_logger(), "URDF missing");
        throw std::runtime_error("robot_description parameter must not be empty");
    }

    std::shared_ptr<xviz::URDFRobot> robot = std::make_shared<xviz::URDFRobot>(xviz::URDFRobot::loadString(urdf, {}, [](const std::string &package) {
        return ament_index_cpp::get_package_share_directory(package);
    })) ;

    QMetaObject::invokeMethod(viewer_, [=]() {
       viewer_->setRobot(robot);
    }, Qt::ConnectionType::QueuedConnection);

    auto subscriber_options = rclcpp::SubscriptionOptions();
    subscriber_options.qos_overriding_options =
            rclcpp::QosOverridingOptions::with_default_policies();

    // subscribe to joint state
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states",
                rclcpp::SensorDataQoS(),
                std::bind(&RobotListener::jointStateCallback, this, std::placeholders::_1),
                subscriber_options);

}

void RobotListener::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state)
{
    rclcpp::Time now = this->now();
    if (last_callback_time_.nanoseconds() > now.nanoseconds()) {
        // force re-publish of joint ransforms
        RCLCPP_WARN(
                    get_logger(), "Moved backwards in time, re-publishing joint transforms!");
        last_publish_time_.clear();
    }
    last_callback_time_ = now;
    // determine least recently published joint
    rclcpp::Time last_published = now;
    for (size_t i = 0; i < state->name.size(); i++) {
        rclcpp::Time t(last_publish_time_[state->name[i]]);
        last_published = (t.nanoseconds() < last_published.nanoseconds()) ? t : last_published;
    }
    // note: if a joint was seen for the first time,
    //       then last_published is zero.

    // check if we need to publish
    rclcpp::Time current_time(state->header.stamp);
    double publish_freq = this->get_parameter_or<double>("update_frequency", 10.0);
    std::chrono::milliseconds publish_interval_ms =
            std::chrono::milliseconds(static_cast<uint64_t>(1000.0 / publish_freq));
    rclcpp::Time max_publish_time = last_published + rclcpp::Duration(publish_interval_ms);
    if (get_parameter_or("ignore_timestamp", false) ||
            current_time.nanoseconds() >= max_publish_time.nanoseconds())
    {
        // get joint positions from state message
        std::map<std::string, double> joint_positions;
        for (size_t i = 0; i < state->name.size(); i++) {
            joint_positions.insert(std::make_pair(state->name[i], state->position[i]));
        }
        /*
            for (const std::pair<const std::string, urdf::JointMimicSharedPtr> & i : mimic_) {
              if (joint_positions.find(i.second->joint_name) != joint_positions.end()) {
                double pos = joint_positions[i.second->joint_name] * i.second->multiplier +
                  i.second->offset;
                joint_positions.insert(std::make_pair(i.first, pos));
              }
            }
*/
        updateTransforms(joint_positions, state->header.stamp);

        // store publish time in joint map
        for (size_t i = 0; i < state->name.size(); i++) {
            last_publish_time_[state->name[i]] = state->header.stamp;
        }
    }


}

void RobotListener::updateTransforms(const std::map<std::string, double> &joint_positions, const builtin_interfaces::msg::Time &time) {
    QMetaObject::invokeMethod(viewer_, [=]() {
       viewer_->updateTransforms(joint_positions);
    }, Qt::ConnectionType::QueuedConnection);
}
