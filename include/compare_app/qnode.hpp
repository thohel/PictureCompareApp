/**
 * @file /include/compare_app/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/

#ifndef compare_app_QNODE_HPP_
#define compare_app_QNODE_HPP_

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <boost/atomic.hpp>
#include <std_msgs/Int32.h>

namespace compare_app {

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
    bool init();
	void run();
    void angleCallback(const std_msgs::Int32::ConstPtr &msg);
    bool waitingForAngle();

Q_SIGNALS:
    void rosShutdown();
    void sendCurrentAngle(double d);

public Q_SLOTS:
    void setAngle(double angle);

private:
	int init_argc;
    char** init_argv;
    ros::Publisher anglepub;
    ros::Subscriber anglesub;
    double current_angle;
    boost::atomic_bool wait_for_angle;
};

}  // namespace compare_app

#endif /* compare_app_QNODE_HPP_ */
