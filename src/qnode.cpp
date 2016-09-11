/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "qgrnd/qnode.hpp"
#include <QDateTime>
#include <fstream>
using std::ofstream;
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qgrnd {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv),
	m_saveRoot("/home/bobin/Documents/research/experiment/UAV")
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"qgrnd");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	ROS_Init(n);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"qgrnd");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	//ROS_INFO_STREAM("before init");
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	ROS_Init(n);
	//ROS_INFO_STREAM("after init begin start");
	start();
	return true;
}
void QNode::ROS_Init(ros::NodeHandle n){
	dji_drone = new DJIDrone(n);

	image_transport::ImageTransport it(n);

	//ROS_INFO_STREAM("after init begin subscribe message");
	//rawImageSub = it.subscribe("camera/image", 1, boost::bind(& MainWindow::showImage,this ,_1));
	markImageSub = it.subscribe("camera/markedImage", 1, boost::bind(&QNode::imageCallback,this ,_1));
	EncoderPosSub = n.subscribe("EncoderMsgs", 1 ,&QNode::EncoderCallBack,this);
	tarPosPub = n.subscribe("TarPosMessage" , 1 ,&QNode::tarPosCallBack,this);
	posePub = n.subscribe("cameraPoseMsg",1 ,&QNode::cameraPoseCallBack,this);

}
void QNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//ROS_INFO_STREAM("image coming");
	ImageRecv = cv_bridge::toCvShare(msg, "bgr8")->image;
	Q_EMIT imageUpdate();
}

void QNode::EncoderCallBack(const geometry_msgs::PointConstPtr & msg)
{
	//ROS_INFO_STREAM("EncoderPos coming");
	EncoderPos = *msg;
}
void QNode::tarPosCallBack(const geometry_msgs::PointConstPtr & msg)
{
	tarPos = *msg;
	//ROS_INFO_STREAM("tarPos coming");
}

void QNode::cameraPoseCallBack(const geometry_msgs::PoseConstPtr & msg)
{
	cameraPose = *msg;
	//ROS_INFO_STREAM("cameraPose coming");
	Q_EMIT cameraPoseUpdate();

}
void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	// while ( ros::ok() ) {

	// 	std_msgs::String msg;
	// 	std::stringstream ss;
	// 	ss << "hello world " << count;
	// 	msg.data = ss.str();
	// 	chatter_publisher.publish(msg);
	// 	log(Info,std::string("I sent: ")+msg.data);
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// 	++count;
	// }
	ros::spin();
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
// void QNode::reordPoseMsg(geometry_msgs::poseStamped poseSt)
// {
	
// }
void QNode::recordHomePoint()
{
	/* dji global_position definition
		Header header
	int32 ts
	#latitude is in angle
	float64 latitude
	#longitude is in angle
	float64 longitude
	float32 altitude
	float32 height
	#reliablity [0,5]
	int8 health 
	*/

	/* dji local position definition
	#    North(x)
	#   /
	#  /
	# /______East(y)
	# |
	# |
	# Donw (-z)
	Header header
	int32 ts
	float32 x
	float32 y
	float32 z 
	*/
	/*

	dji_drone.global_position.latitude;
	dji_drone.global_position.longitude;
	dji_drone.global_position.altitude;

	dji_drone.local_position.x;
	dji_drone.local_position.y;
	dji_drone.local_position.z;
	*/
	gpHome = dji_drone->global_position;
	lpHome = dji_drone->local_position;
	std::string timecnt = QDateTime::currentDateTime().toString("dd.MM.yyyy-hh:mm").toStdString();
	m_savePath= m_saveRoot + timecnt + "home-point.txt";
	ofstream ofs;
  	ofs.open(m_savePath, std::ios_base::out);
  	ofs << "pos gp ( latitude  longitude  altitude  height  health )\n" ;
  	ofs << gpHome.latitude << "\t" << gpHome.longitude << "\t" << gpHome.altitude << "\t" << gpHome.height <<"\n";
 	ofs << "pos lp ( latitude  longitude  altitude  height  health )\n" ;
  	ofs << lpHome.x << "\t" << lpHome.y << "\t" << lpHome.z << "\n";
  	 ofs.close();
}
void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace qgrnd


