/*
    QNode class for ROS node using Qt

    2017-11-13 Lin Zhang
    The Hamlyn Centre for Robotic Surgery,
    Imperial College, London
*/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include "include/qnode.h"
#include <std_msgs/String.h>
#include <sstream>

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv, const std::string &name ) :
    init_argc(argc)
  , init_argv(argv)
  , node_name(name)
    {
    std::cout << "Initializing Node..." << std::endl;
    ros::init(init_argc, init_argv, node_name);
    n = new ros::NodeHandle(node_name);  // Use node name as Ros Namespace
    ROS_INFO("Connected to roscore");
}

QNode::~QNode() {
    shutdown();
}
/**
 * This is called by the qt application to stop the ros node before the
 * qt app closes.
 */
void QNode::shutdown() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
}

void QNode::quitNow(){
    shutdown();
}

void QNode::run(){
    ros::Rate r(10);
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    Q_EMIT rosShutdown();
}


