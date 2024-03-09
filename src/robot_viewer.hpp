#include <xviz/gui/viewer.hpp>
#include <xviz/robot/urdf_robot.hpp>

class RobotViewer: public xviz::SceneViewer {
    Q_OBJECT
public:
    RobotViewer(QWidget *parent = nullptr);

    void	keyPressEvent(QKeyEvent *event) override;

public slots:
    void setRobot(const std::shared_ptr<xviz::URDFRobot> &) ;
    void updateTransforms(const std::map<std::string, double> &values);

private:
    std::shared_ptr<xviz::URDFRobot> urdf_ ;
    float v = 0 ;
};
