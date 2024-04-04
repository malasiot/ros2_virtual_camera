#include "robot_mask_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <xviz/scene/light.hpp>

using namespace std ;
using namespace xviz ;
using namespace Eigen ;

void RobotMaskNode::setupModel(const std::string &urdf_path) {

    string urdf = get_parameter_or("robot_description", std::string());;

    if ( urdf.empty() && !urdf_path.empty() ) {
        std::ifstream in(urdf_path, std::ios::in | std::ios::binary);
        if (in) {
            in.seekg(0, std::ios::end);
            urdf.resize(in.tellg());
            in.seekg(0, std::ios::beg);
            in.read(&urdf[0], urdf.size());
            in.close();
        }

    }

    if ( urdf.empty() ) {
        RCLCPP_ERROR(this->get_logger(), "URDF missing");
        throw std::runtime_error("robot_description parameter must not be empty");
    }


    robot_ = URDFRobot::loadString(urdf, {}, [](const std::string &package) {
        return ament_index_cpp::get_package_share_directory(package);
    }) ;

    rscene_ = RobotScene::fromURDF(robot_) ;

    DirectionalLight *dl = new DirectionalLight(Vector3f(1, 1, 1)) ;
    dl->setDiffuseColor(Vector3f(1, 1, 1)) ;

    scene_->addChild(rscene_) ;
    scene_->setLight(LightPtr(dl));
}

static string make_topic_prefix(const string &ns, const string &name) {
    string res ;
    if ( !ns.empty() ) {
        res += '/' ;
        res += ns ;
    }

    if ( !name.empty() ) {
        res += '/' ;
        res += name ;
    }

    return res ;
}

pair<double, double> parse_frame_size(const string &str) {
    double w, h ;
    size_t idx ;
    w = stod(str, &idx) ;
    h = stod(str.substr(idx+1));
    return {w, h} ;
}

RobotMaskNode::RobotMaskNode(const std::string &urdf_path)
    : Node("virtual_camera", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false)) {

    declare_parameter("robot_description", rclcpp::PARAMETER_STRING) ;
    target_frame_ = declare_parameter("camera_frame",  "camera_optical_frame") ;
    double publish_freq = declare_parameter("update_freq", (double)10.0) ;
    yfov_ = declare_parameter("fov", yfov_) * M_PI/180.0;
    string frame_size = declare_parameter("frame_size", "1024x768") ;
    std::string camera_namespace = declare_parameter("camera_namespace", "") ;
    std::string camera_name = declare_parameter("image_topic", "robot_mask") ;

    tie(width_, height_) = parse_frame_size(frame_size) ;

    scene_ = std::make_shared<xviz::Node>() ;

    setupModel(urdf_path) ;
   
    auto subscriber_options = rclcpp::SubscriptionOptions();
    subscriber_options.qos_overriding_options =
            rclcpp::QosOverridingOptions::with_default_policies();

    // subscribe to joint state
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states",
                rclcpp::SensorDataQoS(),
                std::bind(&RobotMaskNode::jointStateCallback, this, std::placeholders::_1),
                subscriber_options);


    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    prefix_ = make_topic_prefix(camera_namespace, camera_name) ;

    string depth_camera_info_topic = prefix_ + "/camera_info" ;
    string depth_image_topic = prefix_ + "/image_raw" ;

    depth_camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(depth_camera_info_topic, 10);
    depth_image_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_image_topic, 10);

    PerspectiveCamera *pcam = new PerspectiveCamera(width_/(float)height_, // aspect ratio
                                                    yfov_,   // fov
                                                    0.01,        // zmin
                                                    10           // zmax
                                                    ) ;

    pcam_.reset(pcam) ;

    pcam_->setBgColor({1, 1, 1, 1});

    pcam_->setViewport(width_, height_) ;

    offscreen_.reset(new OffscreenSurface(QSize(width_, height_))) ;
/*
    // Initialize the transform broadcaster
    tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    geometry_msgs::msg::TransformStamped transform_;

    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    try {
        transform_ = tf_buffer_->lookupTransform(
                    "world", target_frame_,
                    tf2::TimePointZero, tf2::durationFromSec(2));

        transform_.header.frame_id = "world" ;
        transform_.child_frame_id = target_frame_ + "_world";
            tf_broadcaster_->sendTransform(transform_);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    target_frame_.c_str(), "world", ex.what());
        return;
    }
*/



}


void RobotMaskNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state)
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
    double publish_freq = get_parameter("update_freq").as_double();
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
        updateTransforms(joint_positions, state->header.stamp);

        // store publish time in joint map
        for (size_t i = 0; i < state->name.size(); i++) {
            last_publish_time_[state->name[i]] = state->header.stamp;
        }
    }


}

void RobotMaskNode::updateTransforms(const std::map<std::string, double> &joint_positions, const builtin_interfaces::msg::Time &time) {
    for( const auto &jp: joint_positions) {
        robot_.setJointPosition(jp.first, jp.second);
    }
    map<string, Isometry3f> transforms ;
    robot_.computeLinkTransforms(transforms);

    scene_->updateTransforms(transforms) ;

    publishCameraFrame();

}
sensor_msgs::msg::CameraInfo RobotMaskNode::getCameraInfo() {
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
    return info ;
}

void RobotMaskNode::publishCameraFrame() {
   
    if ( depth_camera_info_pub_->get_subscription_count() != 0 ) {
        sensor_msgs::msg::CameraInfo info = getCameraInfo() ;
        depth_camera_info_pub_->publish(info) ;
    }

    bool pub_depth_image = depth_image_pub_->get_subscription_count() != 0 ;

    if ( pub_depth_image ) {
        try {
            geometry_msgs::msg::TransformStamped t  = tf_buffer_->lookupTransform(
                        target_frame_, "world",  tf2::TimePointZero, tf2::durationFromSec(5));

            camera_tr_ = tf2::transformToEigen(t).cast<float>();

            // fix ROS camera frame convention
            camera_tr_(1, 0) *= -1 ; camera_tr_(1, 1) *= -1 ; camera_tr_(1, 2) *= -1 ; camera_tr_(1, 3) *= -1 ;
            camera_tr_(2, 0) *= -1 ; camera_tr_(2, 1) *= -1 ; camera_tr_(2, 2) *= -1 ; camera_tr_(2, 3) *= -1 ;

            pcam_->setViewTransform(camera_tr_.matrix());
            renderer_.render(scene_, pcam_) ;

             auto depth = offscreen_->getDepthBuffer(0.01, 10) ;

        cv::Mat dim(depth.height(), depth.width(), CV_16UC1, (void *)depth.data()) ;

        double min, max;
        cv::minMaxLoc(dim, &min, &max);

        sensor_msgs::msg::Image msg ;
        cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, dim).toImageMsg(msg);
        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = target_frame_ ;
        depth_image_pub_->publish(msg);


            //  cv::imwrite("/tmp/im.png", clr) ;

        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                        this->get_logger(), "Could not transform world to %s: %s",
                        target_frame_.c_str(), ex.what());
            return;
        }
    }

   

}
