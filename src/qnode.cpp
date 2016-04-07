/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/compare_app/qnode.hpp"

namespace compare_app {

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"compare_app");

	if ( ! ros::master::check() ) {
		return false;
	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

    // Add your ros communications here.
    anglesub = n.subscribe<std_msgs::Int32,QNode>("current_angle", 1000, &QNode::angleCallback, this);
    anglepub = n.advertise<std_msgs::Int32>("set_turntable_angle", 1000);
    wait_for_angle = false;
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;

	while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::setAngle(double angle)
{
    std_msgs::Int32 msg;
    //convert from 0-360 to 0-14400 here
    msg.data = angle*39.5;
    wait_for_angle.store(true);
    anglepub.publish(msg);
    while (current_angle < angle - 0.1 || current_angle > angle + 0.1); // XXX: This is not good, there should be some infinite-loop-avoidance here
    wait_for_angle.store(false);
}

void QNode::angleCallback(const std_msgs::Int32::ConstPtr &msg)
{
    current_angle = msg->data/39.5;
    Q_EMIT sendCurrentAngle(current_angle);
}

bool QNode::waitingForAngle()
{
    return wait_for_angle;
}

}  // namespace compare_app
