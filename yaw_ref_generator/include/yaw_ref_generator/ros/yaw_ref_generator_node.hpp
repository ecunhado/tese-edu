#pragma once

#include <ros/ros.h>
#include "algorithm/yaw_ref_generator.hpp"
#include <medusa_gimmicks_library/MedusaGimmicks.h>
#include <memory>

/* Include the messages used by publishers */
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

/**
 * @brief     Yaw Ref Generator node
 * @author    Eduardo Cunha
 * @version   1.0a
 * @date      2022
 * @copyright MIT
 */

class YawRefGeneratorNode {

  public:

    /**
     * @brief  The constructor of the path following node
     *
     * @param nh  The public ROS node handle
     * @param nh_p The private ROS node handle
     */
    YawRefGeneratorNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p);

    /**
     * @brief The destructor of the path following node (where the subscribers, publishers
     * and services are terminated)
     */
    ~YawRefGeneratorNode();

  private:

    /**
     * @brief Node Handler attributes
     */
    ros::NodeHandle nh_;
    ros::NodeHandle nh_p_; 

    /**
     * @brief ROS Timer attributes 
     */
    ros::Timer timer_;

    /**
     * @brief Sliding Mode attribute 
     */
    std::unique_ptr<YawRefGenerator> yaw_ref_generator_;

    bool got_gamma_;
    
    /** SUBSCRIBERS **/

    /**
     * @brief ROS gamma subscriber 
     */
    ros::Subscriber gamma_sub_;

    /** PUBLISHERS **/

    /**
     * @brief ROS publisher flag 
     */
    ros::Publisher yaw_ref_pub_;
    
    /**
     * @brief Method to initialize the ROS part
     */
    void initializeSubscribers();
    void initializePublishers();
    void initializeTimer();

    /**
     * @brief Method where the logic is located in order to update the control law
     *
     * @param event  A TimerEvent from ros
     */
    void timerIterCallback(const ros::TimerEvent &event);

    void gammaCallback(const std_msgs::Float64 &msg);
};