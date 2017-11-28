#include <QApplication>
#include "ros_interface.h"
#include "Faulharbermotor.h"
#include "qnode.h"

///Connect Signals and Slots
void qt_connections(Faulharbermotor* device, ROSInterface* itface)
{
    /// Suture device info
    QObject::connect(device, &Faulharbermotor::newSutureDeviceInfo, itface,  &ROSInterface::newSutureDeviceInfo);

    QObject::connect(itface,  &ROSInterface::EnableMotor, device, &Faulharbermotor::EnableMotor);
    QObject::connect(itface,  &ROSInterface::runSingleStitch, device, &Faulharbermotor::runSingleStitch);
    QObject::connect(itface,  &ROSInterface::SutureSpeed, device, &Faulharbermotor::SutureSpeed);
}


int main(int argc, char *argv[])
{
    QNode qtRos (argc, argv, "yumi_suture_device");

    QApplication app(argc, argv);

    Faulharbermotor suture_device;
    ROSInterface itface;

    qt_connections(&suture_device, &itface);

    qtRos.start();
    suture_device.start();

    //If one thread receives a exit signal from the user, signal the other thread to quit too
    QObject::connect(&app, &QApplication::aboutToQuit, &qtRos, &QNode::quitNow);
    QObject::connect(&qtRos, &QNode::rosShutdown, &app, &QApplication::quit);

    return app.exec();
}
