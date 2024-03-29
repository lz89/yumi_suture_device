#include <QApplication>
#include "ros_interface.h"
#include "Faulharbermotor.h"
#include "opto_force_sensor.h"
#include "qnode.h"

///Connect Signals and Slots
void qt_connections(Faulharbermotor* device, OptoForceSensor* sensor, ROSInterface* itface)
{
    qRegisterMetaType<deviceInfomation>();
    /// Suture device info
    QObject::connect(device, &Faulharbermotor::newSutureDeviceInfo, itface,  &ROSInterface::newSutureDeviceInfo);
    QObject::connect(sensor, &OptoForceSensor::newForceReading, itface,  &ROSInterface::newForceReading);

    QObject::connect(itface,  &ROSInterface::EnableMotor, device, &Faulharbermotor::EnableMotor);
    QObject::connect(itface,  &ROSInterface::runSingleStitch, device, &Faulharbermotor::runSingleStitch);
    QObject::connect(itface,  &ROSInterface::runPierceDeg, device, &Faulharbermotor::runPierceDeg);
    QObject::connect(itface,  &ROSInterface::SutureSpeed, device, &Faulharbermotor::SutureSpeed);

    QObject::connect(itface,  &ROSInterface::ConnectSensor, sensor, &OptoForceSensor::ConnectSensor);
    QObject::connect(itface,  &ROSInterface::DisconnectSensor, sensor, &OptoForceSensor::DisconnectSensor);

}


int main(int argc, char *argv[])
{
    QNode qtRos (argc, argv, "yumi_suture_device");

    QApplication app(argc, argv);

    Faulharbermotor suture_device;
    OptoForceSensor force_sensor;
    ROSInterface itface;

    qt_connections(&suture_device, &force_sensor, &itface);

    qtRos.start();
    suture_device.start();
    force_sensor.start(20);

    //If one thread receives a exit signal from the user, signal the other thread to quit too
    QObject::connect(&app, &QApplication::aboutToQuit, &qtRos, &QNode::quitNow);
    QObject::connect(&qtRos, &QNode::rosShutdown, &app, &QApplication::quit);
    QObject::connect(&app, &QApplication::aboutToQuit, &force_sensor, &OptoForceSensor::finish);
    QObject::connect(&suture_device, &Faulharbermotor::finished, &suture_device, &QObject::deleteLater);

    return app.exec();
}
