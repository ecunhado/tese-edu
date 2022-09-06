#pragma once

#include <ros/ros.h>
#include "algorithm/sliding_mode.hpp"
#include <medusa_gimmicks_library/MedusaGimmicks.h>
#include <memory>
#include <dsor_utils/control/vehicle_state.hpp>

/* Include the messages used by publishers */
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <auv_msgs/NavigationStatus.h>
#include <auv_msgs/BodyForceRequest.h>
#include "sliding_mode/DebugSlidingMode.h"

/**
 * @brief     Sliding mode node - interfaces between ros and the actual controller
 * @author    Eduardo Cunha
 * @version   1.0a
 * @date      2022
 * @copyright MIT
 */

class SlidingModeNode {

  public:

    /**
     * @brief  The constructor of the path following node
     *
     * @param nh  The public ROS node handle
     * @param nh_p The private ROS node handle
     */
    SlidingModeNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p);

    /**
     * @brief The destructor of the path following node (where the subscribers, publishers
     * and services are terminated)
     */
    ~SlidingModeNode();

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
    std::unique_ptr<SlidingMode> sm_controller_;

    /**
     * @brief Debug message for sliding mode.
     */
    sliding_mode::DebugSlidingMode debug_msg_;

    /**
     * @brief True if a reference for yaw was received in this iteration
     */
    bool got_yaw_ref_;

    /**
     * @brief True if a reference for surge was received in this iteration
     */
    bool got_surge_ref_;

    /**
     * @brief True if a reference for sway was received in this iteration
     */
    bool got_sway_ref_;
    
    /** SUBSCRIBERS **/

    /**
     * @brief ROS state subscriber 
     */
    ros::Subscriber state_sub_;

    /**
     * @brief ROS state subscriber 
     */
    ros::Subscriber yaw_sub_;

    /**
     * @brief ROS state subscriber 
     */
    ros::Subscriber surge_sub_;

    /**
     * @brief ROS state subscriber 
     */
    ros::Subscriber sway_sub_;

    /**
     * @brief ROS gamma subscriber 
     */
    ros::Subscriber gamma_sub_;

    /**
     * @brief ROS vehicle flag subscriber 
     */
    ros::Subscriber flag_sub_;
    
    
    /** PUBLISHERS **/

    /**
     * @brief ROS publisher flag 
     */
    ros::Publisher forces_torques_pub_;

    /**
     * @brief ROS publisher flag 
     */
    ros::Publisher debug_pub_;

    
    /**
     * @brief Method to initialize the ROS part
     */
    void initializeSubscribers();
    void initializePublishers();
    void initializeTimer();
    double nodeFrequency();

    /**
     * @brief Method where the logic is located in order to update the control law
     *
     * @param event  A TimerEvent from ros
     */
    void timerIterCallback(const ros::TimerEvent &event);

    /**
     * @brief Callback method to update state
     *
     * @param msg  Message with state info
     */
    void stateCallback(const auv_msgs::NavigationStatus &msg);

    void yawCallback(const std_msgs::Float64 &msg);

    void surgeCallback(const std_msgs::Float64 &msg);

    void swayCallback(const std_msgs::Float64 &msg);

    void gammaCallback(const std_msgs::Float64 &msg);

    void flagCallback(const std_msgs::Int8 &msg);

};