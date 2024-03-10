#include "virtual_camera_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <xviz/scene/light.hpp>

using namespace std ;
using namespace xviz ;
using namespace Eigen ;

void VirtualCameraNode::setupModel(const std::string &urdf_path) {

    string urdf ;

    if ( !urdf_path.empty() ) {
        std::ifstream in(urdf_path, std::ios::in | std::ios::binary);
        if (in) {
            in.seekg(0, std::ios::end);
            urdf.resize(in.tellg());
            in.seekg(0, std::ios::beg);
            in.read(&urdf[0], urdf.size());
            in.close();
        }

    } else {
        urdf = declare_parameter("robot_description", std::string());
    }

    if ( urdf.empty() ) {
        RCLCPP_ERROR(this->get_logger(), "URDF missing");
        throw std::runtime_error("robot_description parameter must not be empty");
    }

    robot_ = URDFRobot::loadString(urdf, {}, [](const std::string &package) {
        return ament_index_cpp::get_package_share_directory(package);
    }) ;

    scene_ = RobotScene::fromURDF(robot_) ;

    DirectionalLight *dl = new DirectionalLight(Vector3f(1, 1, 1)) ;
    dl->setDiffuseColor(Vector3f(1, 1, 1)) ;
    scene_->setLight(LightPtr(dl)) ;
}

VirtualCameraNode::VirtualCameraNode(const std::string &urdf_path)
    : Node("virtual_camera") {

    setupModel(urdf_path) ;

    auto subscriber_options = rclcpp::SubscriptionOptions();
    subscriber_options.qos_overriding_options =
            rclcpp::QosOverridingOptions::with_default_policies();

    // subscribe to joint state
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states",
                rclcpp::SensorDataQoS(),
                std::bind(&VirtualCameraNode::jointStateCallback, this, std::placeholders::_1),
                subscriber_options);

    target_frame_ = declare_parameter<std::string>("camera_frame", "camera_optical_frame");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    double publish_freq = this->get_parameter_or<double>("update_frequency", 10.0);
    // Call on_timer function every second
    timer_ = create_wall_timer(
                std::chrono::milliseconds(long(1000/publish_freq)), std::bind(&VirtualCameraNode::fetchCameraFrame, this));

    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image", 10);


    PerspectiveCamera *pcam = new PerspectiveCamera(1, // aspect ratio
                                                    yfov_*M_PI/180,   // fov
                                                    0.01,        // zmin
                                                    10           // zmax
                                                    ) ;

    pcam_.reset(pcam) ;

    pcam_->setBgColor({1, 1, 1, 1});

    pcam_->setViewport(width_, height_) ;

    offscreen_.reset(new OffscreenSurface(QSize(width_, height_))) ;

}

void VirtualCameraNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state)
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

void VirtualCameraNode::updateTransforms(const std::map<std::string, double> &joint_positions, const builtin_interfaces::msg::Time &time) {
    for( const auto &jp: joint_positions) {
        robot_.setJointPosition(jp.first, jp.second);
    }
    map<string, Isometry3f> transforms ;
    robot_.computeLinkTransforms(transforms);

    scene_->updateTransforms(transforms) ;

}

void VirtualCameraNode::fetchCameraFrame()
{
    if ( camera_info_pub_->get_subscription_count() != 0 ) {
        sensor_msgs::msg::CameraInfo info ;
        info.width = width_;
        info.height = height_ ;
        info.header.frame_id = target_frame_ ;
        info.header.stamp = get_clock()->now() ;

        float fy = height_ / ( 2 * tan( yfov_ / 2 )) ;

        info.k.at(0) = fy;
        info.k.at(2) = width_/2;
        info.k.at(4) = fy;
        info.k.at(5) = height_/2;
        info.k.at(8) = 1;

        info.p.at(0) = info.k.at(0);
        info.p.at(1) = 0;
        info.p.at(2) = info.k.at(2);
        info.p.at(3) = 0;
        info.p.at(4) = 0;
        info.p.at(5) = info.k.at(4);
        info.p.at(6) = info.k.at(5);
        info.p.at(7) = 0;
        info.p.at(8) = 0;
        info.p.at(9) = 0;
        info.p.at(10) = 1;
        info.p.at(11) = 0;

        // set R (rotation matrix) values to identity matrix
        info.r.at(0) = 1.0;
        info.r.at(1) = 0.0;
        info.r.at(2) = 0.0;
        info.r.at(3) = 0.0;
        info.r.at(4) = 1.0;
        info.r.at(5) = 0.0;
        info.r.at(6) = 0.0;
        info.r.at(7) = 0.0;
        info.r.at(8) = 1.0;

        int coeff_size(5);
        info.distortion_model = "plumb_bob";

        info.d.resize(coeff_size);
        for (int i = 0; i < coeff_size; i++)
        {
            info.d.at(i) = 0.0;
        }

        camera_info_pub_->publish(info) ;
    }

    if ( image_pub_->get_subscription_count() != 0 ) {
        try {
            geometry_msgs::msg::TransformStamped t  = tf_buffer_->lookupTransform(
                        target_frame_, "world",  tf2::TimePointZero, tf2::durationFromSec(5));

            camera_tr_ = tf2::transformToEigen(t).cast<float>();

            // fix ROS camera frame convention
            camera_tr_(0, 0) *= -1 ; camera_tr_(0, 1) *= -1 ; camera_tr_(0, 2) *= -1 ; camera_tr_(0, 3) *= -1 ;
            camera_tr_(2, 0) *= -1 ; camera_tr_(2, 1) *= -1 ; camera_tr_(2, 2) *= -1 ; camera_tr_(2, 3) *= -1 ;

            pcam_->setViewTransform(camera_tr_.matrix());
            renderer_.render(scene_, pcam_) ;
            auto color = offscreen_->getImage() ;

            cv::Mat clr(color.height(), color.width(), CV_8UC4, (void *)color.data());
            cv::cvtColor(clr, clr, cv::COLOR_RGBA2BGR) ;

            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", clr).toImageMsg();
            msg->header.stamp = get_clock()->now();
            msg->header.frame_id = target_frame_ ;
            image_pub_->publish(*msg);

            //  cv::imwrite("/tmp/im.png", clr) ;

        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                        this->get_logger(), "Could not transform world to %s: %s",
                        target_frame_.c_str(), ex.what());
            return;
        }
    }
}
