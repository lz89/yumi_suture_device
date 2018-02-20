/*
    ROSInterface class

    Manage communication with ROS

    2017-11-13 Lin Zhang
    The Hamlyn Centre for Robotic Surgery,
    Imperial College, London
*/

#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Vector3.h>
#include <QtCore/QObject>
#include <QtGui/QVector3D>
#include "yumi_msg/SutureDeviceInfo.h"
#include "yumi_suture_def.h"

Q_DECLARE_METATYPE(deviceInfomation)

class ROSInterface : public QObject {
Q_OBJECT
public:
    ROSInterface();
//    ~ROSInterface() override;

    /**
     * Receive from ROS
     */
private:

    /// ROS callback
    void SutureCtrlCallback (const std_msgs::UInt32ConstPtr& msg);

    /**
     * Send to ROS
     */
private:
    ros::Publisher m_pub_suture_info;

    ros::Publisher m_pub_force_sensor;

    ros::Subscriber m_sub_suture_ctrl;

    /**
     * Qt signal slot
     */
signals:
    void EnableMotor(bool flag);
    void runSingleStitch();
    void runPierceDeg(double deg);
    // True for increase, false for decrease
    void SutureSpeed(bool flag);

    /// Force sensor
    void ConnectSensor ();
    void DisconnectSensor ();

public slots:
    void newSutureDeviceInfo(deviceInfomation info);

    void newForceReading(QVector3D force);

};


#endif // ROS_INTERFACE_H
