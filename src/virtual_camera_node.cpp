#include "virtual_camera_node.hpp"

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

void VirtualCameraNode::setupModel(const std::string &urdf_path) {

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

    xviz::NodePtr camera_node = rscene_->findNodeByName(camera_link_name_);
    if ( camera_node ) 
    camera_node->setVisible(false) ;

    DirectionalLight *dl = new DirectionalLight(Vector3f(1, 1, 1)) ;
    dl->setDiffuseColor(Vector3f(1, 1, 1)) ;

    scene_->addChild(rscene_) ;
    scene_->setLight(LightPtr(dl));
}

void VirtualCameraNode::setupMesh(const std::string &mesh_path) {

    string mesh_path_param = get_parameter_or("mesh", std::string());;
    string path ;
    if ( !mesh_path_param.empty()) path = mesh_path_param ;
    else if ( !mesh_path.empty()) path = mesh_path ;
  
    if ( path.empty() ) return ;

    RCLCPP_ERROR(this->get_logger(), "Loading URDF mesh %s", path.c_str());

    mesh_ = URDFRobot::loadFile(path, {}, [](const std::string &package) {
        return ament_index_cpp::get_package_share_directory(package);
    }) ;

    mscene_ = RobotScene::fromURDF(mesh_) ;

    scene_->addChild(mscene_) ;
}

static string make_topic_prefix(const string &ns, const string &name) {
    string res ;
    if ( !ns.empty() ) {
        res += ns ;
        res += '/' ;
    }

    if ( !name.empty() ) {
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

VirtualCameraNode::VirtualCameraNode(const std::string &urdf_path, const std::string &mesh_path)
    : Node("virtual_camera", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false)) {

    declare_parameter("robot_description", rclcpp::PARAMETER_STRING) ;
    declare_parameter("mesh", rclcpp::PARAMETER_STRING) ;
    target_frame_ = declare_parameter("camera_frame",  "camera_optical_frame") ;
    double publish_freq = declare_parameter("update_freq", (double)10.0) ;
    yfov_ = declare_parameter("fov", yfov_) * M_PI/180.0;
    string frame_size = declare_parameter("frame_size", "1024x768") ;
    std::string camera_namespace = declare_parameter("camera_namespace", "") ;
    std::string camera_name = declare_parameter("image_topic", "virtual_camera") ;
    camera_link_name_ = declare_parameter("camera_link", "camera_link");

    tie(width_, height_) = parse_frame_size(frame_size) ;

    scene_ = std::make_shared<xviz::Node>() ;

    setupModel(urdf_path) ;
    setupMesh(mesh_path) ;

    auto subscriber_options = rclcpp::SubscriptionOptions();
    subscriber_options.qos_overriding_options =
            rclcpp::QosOverridingOptions::with_default_policies();

    // subscribe to joint state
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states",
                rclcpp::SensorDataQoS(),
                std::bind(&VirtualCameraNode::jointStateCallback, this, std::placeholders::_1),
                subscriber_options);


    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    // Call on_timer function every second
    timer_ = create_wall_timer(
                std::chrono::milliseconds(long(1000/publish_freq)), std::bind(&VirtualCameraNode::fetchCameraFrame, this));

    prefix_ = make_topic_prefix(camera_namespace, camera_name) ;

    string color_camera_info_topic = prefix_ + "/color/camera_info" ;
    string color_image_topic = prefix_ + "/color/image_raw" ;

    color_camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(color_camera_info_topic, 10);
    color_image_pub_ = create_publisher<sensor_msgs::msg::Image>(color_image_topic, 10);

    string depth_camera_info_topic = prefix_ + "/depth/camera_info" ;
    string depth_image_topic = prefix_ + "/depth/image_raw" ;

    depth_camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(depth_camera_info_topic, 10);
    depth_image_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_image_topic, 10);

    string pcl_topic = prefix_ + "/points" ;
    pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pcl_topic, 10) ;

    PerspectiveCamera *pcam = new PerspectiveCamera(width_/(float)height_, // aspect ratio
                                                    yfov_,   // fov
                                                    0.01,        // zmin
                                                    10           // zmax
                                                    ) ;

    pcam_.reset(pcam) ;

    pcam_->setBgColor({1, 1, 1, 1});

    pcam_->setViewport(width_, height_) ;

    offscreen_.reset(new OffscreenSurface(QSize(width_, height_))) ;

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

void VirtualCameraNode::updateTransforms(const std::map<std::string, double> &joint_positions, const builtin_interfaces::msg::Time &time) {
    for( const auto &jp: joint_positions) {
        robot_.setJointPosition(jp.first, jp.second);
    }
    map<string, Isometry3f> transforms ;
    robot_.computeLinkTransforms(transforms);

    scene_->updateTransforms(transforms) ;

}

template<typename T>
struct DepthTraits {};

template<>
struct DepthTraits<uint16_t>
{
    static inline bool valid(uint16_t depth) {return depth != 0;}
    static inline float toMeters(uint16_t depth) {return depth * 0.001f;}   // originally mm
    static inline uint16_t fromMeters(float depth) {return (depth * 1000.0f) + 0.5f;}
    static inline void initializeBuffer(std::vector<uint8_t> &) {}  // Do nothing
};

template<>
struct DepthTraits<float>
{
    static inline bool valid(float depth) {return std::isfinite(depth);}
    static inline float toMeters(float depth) {return depth;}
    static inline float fromMeters(float depth) {return depth;}

    static inline void initializeBuffer(std::vector<uint8_t> & buffer)
    {
        float * start = reinterpret_cast<float *>(&buffer[0]);
        float * end = reinterpret_cast<float *>(&buffer[0] + buffer.size());
        std::fill(start, end, std::numeric_limits<float>::quiet_NaN());
    }
};

void convert(
        const cv::Mat & depth,
        const cv::Mat &clr,
        const image_geometry::PinholeCameraModel & model,
        sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg,
        double range_max = 0.0,
        bool use_quiet_nan = false
        )
{
    // Use correct principal point from calibration
    float center_x = model.cx();
    float center_y = model.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = 0.001f ;
    float constant_x = unit_scaling / model.fx();
    float constant_y = unit_scaling / model.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_rgb(*cloud_msg, "rgb");
    const uint16_t * depth_row = reinterpret_cast<const uint16_t *>(&depth.data[0]);
    int row_step = depth.step / sizeof(uint16_t);
    for (int v = 0; v < static_cast<int>(cloud_msg->height); ++v, depth_row += row_step) {
        for (int u = 0; u < static_cast<int>(cloud_msg->width); ++u, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
            uint16_t depth = depth_row[u];

            // Missing points denoted by NaNs
            if ( depth == 0 ) {
                *iter_x = *iter_y = *iter_z = *iter_rgb = bad_point;
                continue;
            }

            // Fill in XYZ
            *iter_x = (u - center_x) * depth * constant_x;
            *iter_y = (v - center_y) * depth * constant_y;
            *iter_z = depth * unit_scaling ;

            // and RGB
            int rgb = 0x000000;

            if( clr.type()==CV_8UC3 ) {
                //RGB or RGBA
                if (clr.rows > v && clr.cols > u){
                    rgb |= ((int)clr.at<cv::Vec3b>(v, u)[2]) << 16;
                    rgb |= ((int)clr.at<cv::Vec3b>(v, u)[1]) << 8;
                    rgb |= ((int)clr.at<cv::Vec3b>(v, u)[0]);
                }
            }

            *iter_rgb = *reinterpret_cast<float*>(&rgb);
        }
    }
}

sensor_msgs::msg::CameraInfo VirtualCameraNode::getCameraInfo(rclcpp::Time t) {
    sensor_msgs::msg::CameraInfo info ;
    info.width = width_;
    info.height = height_ ;
    info.header.frame_id = target_frame_ ;
    info.header.stamp = t ;

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

void VirtualCameraNode::fetchCameraFrame() {
   
    rclcpp::Time ts(get_clock()->now()) ;

    bool pub_color_image = color_image_pub_->get_subscription_count() != 0 ;
    bool pub_depth_image = depth_image_pub_->get_subscription_count() != 0 ;

    if ( pub_color_image || pub_depth_image ) {
        try {
            geometry_msgs::msg::TransformStamped t  = tf_buffer_->lookupTransform(
                        target_frame_, "world",  tf2::TimePointZero, tf2::durationFromSec(5));

            camera_tr_ = tf2::transformToEigen(t).cast<float>();

            // fix ROS camera frame convention
            camera_tr_(1, 0) *= -1 ; camera_tr_(1, 1) *= -1 ; camera_tr_(1, 2) *= -1 ; camera_tr_(1, 3) *= -1 ;
            camera_tr_(2, 0) *= -1 ; camera_tr_(2, 1) *= -1 ; camera_tr_(2, 2) *= -1 ; camera_tr_(2, 3) *= -1 ;

            pcam_->setViewTransform(camera_tr_.matrix());
            renderer_.render(scene_, pcam_) ;


            //  cv::imwrite("/tmp/im.png", clr) ;

        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                        this->get_logger(), "Could not transform world to %s: %s",
                        target_frame_.c_str(), ex.what());
            return;
        }
    }

     if ( color_camera_info_pub_->get_subscription_count() != 0 ) {
        sensor_msgs::msg::CameraInfo info = getCameraInfo(ts) ;
        color_camera_info_pub_->publish(info) ;
    }

    if ( depth_camera_info_pub_->get_subscription_count() != 0 ) {
        sensor_msgs::msg::CameraInfo info = getCameraInfo(ts) ;
        depth_camera_info_pub_->publish(info) ;
    }

    if ( pub_color_image ) {
        auto color = offscreen_->getImage() ;

        cv::Mat clr(color.height(), color.width(), CV_8UC4, (void *)color.data());
        cv::cvtColor(clr, clr, cv::COLOR_RGBA2BGR) ;

        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", clr).toImageMsg();
        msg->header.stamp = ts;
        msg->header.frame_id = target_frame_ ;
        color_image_pub_->publish(*msg);
    }

    if ( pub_depth_image ) {
        auto depth = offscreen_->getDepthBuffer(0.01, 10) ;

        cv::Mat dim(depth.height(), depth.width(), CV_16UC1, (void *)depth.data()) ;

        double min, max;
        cv::minMaxLoc(dim, &min, &max);

        sensor_msgs::msg::Image msg ;
        cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, dim).toImageMsg(msg);
        msg.header.stamp = ts;
        msg.header.frame_id = target_frame_ ;
        depth_image_pub_->publish(msg);
    }

    if ( pcl_pub_->get_subscription_count() > 0 ) {

        auto color = offscreen_->getImage() ;

        cv::Mat clr(color.height(), color.width(), CV_8UC4, (void *)color.data());
        cv::cvtColor(clr, clr, cv::COLOR_RGBA2BGR) ;

        auto depth = offscreen_->getDepthBuffer(0.01, 10) ;

        cv::Mat dim(depth.height(), depth.width(), CV_16UC1, (void *)depth.data()) ;

        sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg =
                std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header = std_msgs::msg::Header();
        cloud_msg->header.stamp = ts;
        cloud_msg->header.frame_id = target_frame_ ;
        cloud_msg->height = height_;
        cloud_msg->width = width_;
        cloud_msg->is_dense = false;
        cloud_msg->is_bigendian = false;
        cloud_msg->fields.clear();
        cloud_msg->fields.reserve(2);


        sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
        pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

        // g_cam_info here is a sensor_msg::msg::CameraInfo::SharedPtr,
        // which we get from the depth_camera_info topic.
        image_geometry::PinholeCameraModel model;
        model.fromCameraInfo(getCameraInfo(ts));

        convert(dim, clr, model, cloud_msg);

        pcl_pub_->publish(*cloud_msg) ;

    }
}
