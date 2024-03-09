#include "robot_viewer.hpp"

#include <xviz/gui/viewer.hpp>
#include <xviz/robot/robot_scene.hpp>
#include <xviz/scene/light.hpp>
#include <xviz/scene/geometry.hpp>

#include <QDebug>

using namespace xviz ;
using namespace std;
using namespace Eigen ;

RobotViewer::RobotViewer(QWidget *parent): xviz::SceneViewer(parent) {
   // setDefaultCamera() ;

}

void RobotViewer::keyPressEvent(QKeyEvent *event) {
    int key = event->key() ;

    if ( key == Qt::Key_Q ) {
        v = urdf_->setJointPosition("left_gripper_joint", v - 0.1) ;
        qDebug() << v ;
    } else if ( key == Qt::Key_W ) {
        v = urdf_->setJointPosition("left_gripper_joint", v + 0.1) ;
    }
    map<string, Isometry3f> transforms ;
    urdf_->computeLinkTransforms(transforms);

    scene_->updateTransforms(transforms) ;

    update() ;
}


void RobotViewer::setRobot(const std::shared_ptr<xviz::URDFRobot> &robot)
{
    urdf_ = robot ;
    RobotScenePtr scene = RobotScene::fromURDF(*robot) ;

  //  map<string, Isometry3f> transforms ;
  //  robot->computeLinkTransforms(transforms);

  //  scene->updateTransforms(transforms) ;

  //  scene->addChild(makeBox("box1", {0.005, 0.005, 0.05}, {0.25, 0.15, 0.355}, {1, 0, 0, 1}));
  //  scene->addChild(makeBox("box2", {0.05, 0.05, 0.05}, {0, 0.4, 0.1}, {1, 1, 0, 1}));

    DirectionalLight *dl = new DirectionalLight(Vector3f(1.5, 2.5, 1)) ;
    dl->setDiffuseColor(Vector3f(1, 1, 1)) ;
    scene->addLightNode(LightPtr(dl)) ;

    setScene(scene) ;
    initCamera({0, 0, 0}, 1.0, ZAxis);
     camera_->setBgColor({0.4f, 0.4f, 0.4f, 1.0f});
     update() ;
}

void RobotViewer::updateTransforms(const std::map<std::string, double> &values)
{
    for( const auto &jp: values) {
        urdf_->setJointPosition(jp.first, jp.second);
    }
    map<string, Isometry3f> transforms ;
    urdf_->computeLinkTransforms(transforms);

    scene_->updateTransforms(transforms) ;

    update() ;

}

