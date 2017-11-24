/*
    QNode class for ROS node using Qt

    2015 Lin Zhang
    The Hamlyn Centre for Robotic Surgery,
    Imperial College, London
*/

#ifndef QNODE_H
#define QNODE_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>


/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
Q_OBJECT

public:
    QNode(int argc, char** argv, const std::string &name );
    virtual ~QNode();

    void shutdown();
    void run();

    ros::NodeHandle getNodeHandle() { return *n; }

    const std::string& nodeName() { return node_name; }

public Q_SLOTS:
    //!Connect to aboutToQuit signals, to stop the thread
    void quitNow();

Q_SIGNALS:
    void rosShutdown();

protected:
    int init_argc;
    char** init_argv;
    const std::string node_name;


private:
    ros::NodeHandle *n;
};

#endif // QNODE_H
