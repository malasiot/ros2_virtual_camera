#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <xviz/robot/urdf_robot.hpp>

using namespace std::chrono_literals;

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
        declare_parameter("robot_description", "");

        std::string urdf = get_parameter_or<std::string>("robot_description", "") ;
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("iiwa_description");

        if ( urdf.empty() ) {
            RCLCPP_ERROR(this->get_logger(), "URDF missing");
            throw std::runtime_error("robot_description parameter must not be empty");
        }

        RCLCPP_INFO(this->get_logger(), "%s", urdf.c_str());
        std::map<std::string, std::string> packages ;
        packages["iiwa_description"] = package_share_directory ;
        xviz::URDFRobot robot = xviz::URDFRobot::loadString(urdf, {}, packages) ;
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {

// may throw ament_index_cpp::PackageNotFoundError exception

      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  auto sub = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(sub);
  rclcpp::shutdown();
  return 0;
}


