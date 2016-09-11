/**
 * @file /include/qgrnd/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qgrnd_QNODE_HPP_
#define qgrnd_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include "opencv2/opencv.hpp"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>


#include <tf/transform_datatypes.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "dji_sdk/dji_drone.h"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qgrnd {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
	cv::Mat 	ImageRecv;
	ros::Subscriber EncoderPosSub;
	ros::Subscriber tarPosPub;
	ros::Subscriber posePub;
	DJIDrone *dji_drone;
	ros::Subscriber dji_timstamp;

	image_transport::Subscriber markImageSub;
	geometry_msgs::Point EncoderPos;
	geometry_msgs::Point tarPos;
	geometry_msgs::Pose cameraPose;

	dji_sdk::GlobalPosition gpHome;
    dji_sdk::LocalPosition lpHome;

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	std::string m_saveRoot;
	std::string m_savePath;
	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
	void recordHomePoint();

Q_SIGNALS:
	void loggingUpdated();
	void imageUpdate();
	void cameraPoseUpdate();
    	void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    	QStringListModel logging_model;

    void ROS_Init(ros::NodeHandle n);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void EncoderCallBack(const geometry_msgs::PointConstPtr & msg);
	void tarPosCallBack(const geometry_msgs::PointConstPtr & msg);
	void cameraPoseCallBack(const geometry_msgs::PoseConstPtr & msg);
};

}  // namespace qgrnd

#endif /* qgrnd_QNODE_HPP_ */
