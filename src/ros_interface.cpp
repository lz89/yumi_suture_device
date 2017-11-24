#include "ros_interface.h"

ROSInterface::ROSInterface()
{
    ros::NodeHandle nh;

    /// Subscribers
    m_sub_suture_ctrl = nh.subscribe<std_msgs::UInt32>("/yumi_suture_ctrl", 1, &ROSInterface::SutureCtrlCallback, this);

    /// Publishers
    m_pub_suture_info = nh.advertise<yumi_msg::SutureDeviceInfo>("/yumi_suture_info", 1);
}

void ROSInterface::SutureCtrlCallback(const std_msgs::UInt32ConstPtr &msg) {
    SUTURE_CTRL_CMD cmd = msg->data;
    if (cmd & SUTURE_ENABLE_MOTOR) Q_EMIT EnableMotor(true);
    if (cmd & SUTURE_DISABLE_MOTOR) Q_EMIT EnableMotor(false);

    if (cmd & SUTURE_RUN_STITCH) Q_EMIT RunStitch();
    if (cmd & SUTURE_SPEED_PLUS) Q_EMIT SutureSpeed(true);
    if (cmd & SUTURE_SPEED_MINUS) Q_EMIT SutureSpeed(false);
}

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
