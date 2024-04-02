#include "virtual_camera_node.hpp"

#include <QApplication>

int main(int argc, char * argv[])
{
    QApplication app(argc, argv) ;

    rclcpp::init(argc, argv);

    std::string urdf_path, mesh_path ;

    if ( argc > 1 ) urdf_path = argv[1] ;
    if ( argc > 2 ) mesh_path = argv[2] ;

    auto server = std::make_shared<VirtualCameraNode>(urdf_path, mesh_path);

    rclcpp::spin(server) ;
    rclcpp::shutdown();

    return 0;
}


