#include "virtual_camera_node.hpp"

#include <QApplication>

int main(int argc, char * argv[])
{
    QApplication app(argc, argv) ;

    rclcpp::init(argc, argv);

    auto server = std::make_shared<VirtualCameraNode>(argc > 1 ? argv[1] : "");

    rclcpp::spin(server) ;
    rclcpp::shutdown();

    return 0;
}


