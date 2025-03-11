#include "robot_mask_node.hpp"

#include <QApplication>

int main(int argc, char * argv[])
{
    QApplication app(argc, argv) ;

    rclcpp::init(argc, argv);

    std::string urdf_path, ns ;

    if ( argc > 1 ) {
        if ( strncmp(argv[1], "--ros-args", 10) != 0 ) {
            urdf_path = argv[1] ;
        }
        else {
            for( int i=2 ; i<argc ; i++ ) {
                std::string arg = argv[i] ;
                if ( arg.substr(0, 6) == "__ns:=" ) {
                    ns = arg.substr(6) ;
                } 
            }
        }

    }

    auto server = std::make_shared<RobotMaskNode>(urdf_path, ns);

    rclcpp::spin(server) ;
    rclcpp::shutdown();

    return 0;
}
