#include "ros_interface.h"

ROSInterface::ROSInterface()
{
    ros::NodeHandle nh;

    /// Subscribers
    m_sub_suture_ctrl = nh.subscribe<std_msgs::UInt32>("/yumi_suture_ctrl", 1, &ROSInterface::SutureCtrlCallback, this);

    /// Publishers
    m_pub_suture_info = nh.advertise<yumi_msg::SutureDeviceInfo>("/yumi_suture_info", 1);
    m_pub_force_sensor = nh.advertise<geometry_msgs::Vector3>("/yumi_force_sensor", 1);

}

void ROSInterface::SutureCtrlCallback(const std_msgs::UInt32ConstPtr &msg) {
    SUTURE_CTRL_CMD cmd = msg->data;
    if (cmd & SUTURE_ENABLE_MOTOR) Q_EMIT EnableMotor(true);
    if (cmd & SUTURE_DISABLE_MOTOR) Q_EMIT EnableMotor(false);

    if (cmd & SUTURE_RUN_STITCH)    Q_EMIT runSingleStitch();
    if (cmd & SUTURE_SPEED_PLUS)    Q_EMIT SutureSpeed(true);
    if (cmd & SUTURE_SPEED_MINUS)   Q_EMIT SutureSpeed(false);
    if (cmd & SUTURE_RUN_PIERCE_INIT)        Q_EMIT runPierceDeg(0);
    if (cmd & SUTURE_RUN_PIERCE_1_MM)        Q_EMIT runPierceDeg(0+8.7);
    if (cmd & SUTURE_RUN_PIERCE_2_MM)        Q_EMIT runPierceDeg(0+17.5);
    if (cmd & SUTURE_RUN_PIERCE_3_MM)        Q_EMIT runPierceDeg(0+26);
    if (cmd & SUTURE_RUN_PIERCE_4_MM)        Q_EMIT runPierceDeg(0+34.8);
    if (cmd & SUTURE_RUN_PIERCE_5_MM)        Q_EMIT runPierceDeg(0+43.5);
    if (cmd & SUTURE_RUN_PIERCE_6_MM)        Q_EMIT runPierceDeg(0+60);

    if (cmd & FORCE_SENSOR_CONNECT)          Q_EMIT ConnectSensor();
    if (cmd & FORCE_SENSOR_DISCONNECT)          Q_EMIT DisconnectSensor();

}

// 8.7 deg  ->  1.00 mm
// 17.5 deg ->  2.01 mm
// 26 deg   ->  2.99 mm
// 34.8 deg ->  4.01 mm
// 43.5 deg ->  5.01 mm
// 52.2 deg ->  6.01 mm

void ROSInterface::newSutureDeviceInfo(deviceInfomation info) {
    yumi_msg::SutureDeviceInfo msg;
    msg.msrPos.data.push_back(info.msrPos[0]);
    msg.msrPos.data.push_back(info.msrPos[1]);

    msg.cmdPos.data.push_back(info.cmdPos[0]);
    msg.cmdPos.data.push_back(info.cmdPos[1]);

    msg.msrTem.data.push_back(info.msrTem[0]);
    msg.msrTem.data.push_back(info.msrTem[1]);

    msg.msrCur.data.push_back(info.msrCur[0]);
    msg.msrCur.data.push_back(info.msrCur[1]);

    msg.connectSig.data = static_cast<unsigned char>(info.connectSig);
    msg.enableSig.data = static_cast<unsigned char>(info.enableSig);
    msg.status.data = info.status;
    msg.speedScale.data = info.speedScale;
    msg.absLockUpPos.data = info.absLockUpPos;
    msg.absLockDownPos.data = info.absLockDownPos;
    msg.absOpenPos.data = info.absOpenPos;
    msg.absClosePos.data = info.absClosePos;

    m_pub_suture_info.publish(msg);
}

void ROSInterface::newForceReading(QVector3D force) {
    geometry_msgs::Vector3 msg;
    /// Sensitivity of OMD-20-F G-100N XYZ-1.25|1.25|6.25mN
    msg.x = force.x() * 0.00125;    // Unit: Newton
    msg.y = force.y() * 0.00125;
    msg.z = force.z() * 0.00625;
    m_pub_force_sensor.publish(msg);
}
