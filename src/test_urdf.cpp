#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "robot_listener.hpp"

#include <xviz/robot/robot_scene.hpp>


#include <QMainWindow>
#include <QApplication>
#include <QDebug>

#include "qt_executor.hpp"
#include "robot_viewer.hpp"

using namespace std::chrono_literals;
using namespace std;
using namespace xviz ;
using namespace Eigen ;

using std::placeholders::_1;



int main(int argc, char * argv[])
{
    QApplication app(argc, argv) ;

    rclcpp::init(argc, argv);


    SceneViewer::initDefaultGLContext();

    QMainWindow window ;
    RobotViewer *viewer = new RobotViewer() ;
      window.setCentralWidget(viewer) ;
    window.resize(1024, 1024) ;
    window.show() ;


    auto server = std::make_shared<RobotListener>(argc > 1 ? argv[1] : "", viewer);


    std::thread t = std::thread([server]{
        rclcpp::spin(server) ;
    });



 //   QtExecutor executor;
  //  executor.add_node(server);

   // executor.start();

     app.exec();

     t.join() ;
    printf("Exited QT thread\n");
    rclcpp::shutdown();


    return 0;
}


