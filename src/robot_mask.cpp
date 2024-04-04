#include "robot_mask_node.hpp"

#include <QApplication>

int main(int argc, char * argv[])
{
    QApplication app(argc, argv) ;

    rclcpp::init(argc, argv);

    std::string urdf_path ;

    if ( argc > 1 && strncmp(argv[1], "--ros-args", 10) != 0 ) {
        urdf_path = argv[1] ;
    
    }

    auto server = std::make_shared<RobotMaskNode>(urdf_path);

    rclcpp::spin(server) ;
    rclcpp::shutdown();

    return 0;
}
