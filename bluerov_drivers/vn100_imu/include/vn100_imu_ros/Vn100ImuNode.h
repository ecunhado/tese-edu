/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico */
#ifndef CATKIN_WS_VN100IMUNODE_H
#define CATKIN_WS_VN100IMUNODE_H

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <vector>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h> //ALWAYS need to include this


// ROS messages and stuff
#include "vn100_imu/imu_pp.h"
#include <std_msgs/Empty.h>
#include <medusa_msgs/mIMU.h>
#include <dsor_msgs/Measurement.h>
#include <nmea_msgs/Sentence.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <medusa_gimmicks_library/MedusaGimmicks.h>
#include <medusa_diagnostics_library/MedusaDiagnostics.h>
#include <nmeaSerial.h>
#include <boost/format.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
 

 class Vn100ImuNode {
 public:
 	// #############################
 	// @.@ Constructor
 	// #############################
 	Vn100ImuNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);

 	// #############################
 	// @.@ Destructor
 	// #############################
 	~Vn100ImuNode();

 	// #############################
 	// @.@ Public methods
 	// #############################
 	double nodeFrequency();

 private:

 	ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
	ros::NodeHandle nh_p_;
 	
 	// #######################
 	// @.@ Timer
 	// #######################
 	ros::Timer timer_;

     
  	std::vector<std::string> p_config_list_;
  	std::vector<std::string> p_check_list_;

  	bool p_use_position_;

  	double p_originLat_, p_originLon_;

  	ros::Publisher pub_raw_;
  	ros::Publisher pub_imu_;
  	ros::Publisher pub_diag_;
  	ros::Publisher pub_pp_;
	ros::Publisher pub_meas_filter_;

  	ros::Subscriber sub_conf_;
  	ros::Subscriber sub_tare_;

  	nmeaSerial serial_port_;
	diagnostic_msgs::DiagnosticStatus imu_state_;

  	std::string p_sPORT_;
  	int p_BAUDRATE_;
	std::string p_frame_sensor_;

 	// #######################################################################################
 	// @.@ Encapsulation the gory details of initializing subscribers, publishers and services
 	// #######################################################################################
 	void initializeSubscribers();
 	void initializePublishers();
 	void initializeTimer();
 	void loadParams();

	 void bound(medusa_msgs::mIMU& msg);
	 void configureIMU(void);
	 void reporting(const medusa_msgs::mIMU *imu_msg);
	 void pubMags(const medusa_msgs::mIMU& msg);


 	// #######################################################################################
 	// @.@ Callbacks declaration
 	// #######################################################################################
    void rawCallback(const std::string &msg);
    void confCallback(std_msgs::String str);
	void tareCallback(const std_msgs::Empty::ConstPtr& empty);
    void vnymrCallback(const std::vector<std::string>& tokens);
	void vnyiaCallback(const std::vector<std::string>& tokens);
	void vnycmCallback(const std::vector<std::string>& tokens);
	void timerCallback(const ros::TimerEvent &event);

};
#endif //CATKIN_WS_VN100IMUNODE_H
