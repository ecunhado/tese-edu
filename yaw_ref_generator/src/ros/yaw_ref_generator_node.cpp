#include "ros/yaw_ref_generator_node.hpp"

/**
 * @brief  The Path Following Node constructor
 *
 * @param nh  Public ros node handle
 * @param nh_p  Private ros node handle
 */
YawRefGeneratorNode::YawRefGeneratorNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p)
    : nh_(*nh), nh_p_(*nh_p) {

  ROS_INFO("in class constructor of YawRefGeneratorNode");
  this->initializeSubscribers();
  this->initializePublishers();
  this->initializeTimer();

  // create YawRefGenerator object
  this->yaw_ref_generator_ = std::make_unique<YawRefGenerator>();

  this->got_gamma_ = false;
}

/**
 * @brief  Node class destructor
 */
YawRefGeneratorNode::~YawRefGeneratorNode() {
  
  /* Stop the timer callback */
  this->timer_.stop();

  /* Shutdown the node */
  this->nh_.shutdown();
}

/**
 * @brief  Initialize all the subscribers
 */
void YawRefGeneratorNode::initializeSubscribers() {

  ROS_INFO("Initializing Subscribers for YawRefGeneratorNode");

  this->gamma_sub_ =
      nh_.subscribe("/bluerov/Gamma", 10, &YawRefGeneratorNode::gammaCallback, this);
}

/**
 * @brief  Method to initialize the publishers of the path following node
 */
void YawRefGeneratorNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for YawRefGeneratorNode");

  /* Initialize the publishers */
  this->yaw_ref_pub_ = nh_.advertise<std_msgs::Float64>("/bluerov/ref/yaw", 1);
}

/**
 * @brief  Initialize the timer callback
 */
void YawRefGeneratorNode::initializeTimer() {
  this->timer_ =
      nh_.createTimer(ros::Duration(1.0 / 10.0),
                      &YawRefGeneratorNode::timerIterCallback, this);

  /* Wait for the start service to start the Path Following */
  this->timer_.start();
}

void YawRefGeneratorNode::timerIterCallback(const ros::TimerEvent &event) {
  double yaw_ref = this->yaw_ref_generator_->computeYawRef();

  std_msgs::Float64 msg;
  msg.data = yaw_ref;
  
  // publish yaw ref if path following is being executed
  if (this->got_gamma_) {
    this->yaw_ref_pub_.publish(msg);
  }
}

void YawRefGeneratorNode::gammaCallback(const std_msgs::Float64 &msg) {
  this->yaw_ref_generator_->setGamma(msg.data);
  this->got_gamma_ = true;
}

/**
 * @brief  The main function
 *
 * @param argc  The argument count
 * @param argv  The argument array
 */
int main(int argc, char **argv) {

  ros::init(argc, argv, "yaw_ref_generator_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");

  ROS_INFO("main: instantiating an object of type YawRefGeneratorNode");

  YawRefGeneratorNode YawRefGeneratorNode(&nh, &nh_p);

  /* Going into spin and let the timer callback do all the work */
  ros::spin();

  return 0;
}
