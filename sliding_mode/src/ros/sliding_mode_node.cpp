#include "ros/sliding_mode_node.hpp"

/**
 * @brief  The Path Following Node constructor
 *
 * @param nh  Public ros node handle
 * @param nh_p  Private ros node handle
 */
SlidingModeNode::SlidingModeNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p)
    : nh_(*nh), nh_p_(*nh_p) {

  ROS_INFO("in class constructor of SlidingModeNode");
  this->initializeSubscribers();
  this->initializePublishers();
  /* NOTE: initializeServices is implemented inside PathFollowingServices.cpp */
  this->initializeTimer();

  // get parameters
  double lambda_yaw = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "lambda/yaw");
  double lambda_surge = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "lambda/surge");
  double lambda_sway = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "lambda/sway");

  double epsilon_yaw = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "epsilon/yaw");
  double epsilon_surge = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "epsilon/surge");
  double epsilon_sway = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "epsilon/sway");

  double k_c_yaw = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "k_c/yaw");
  double k_c_surge = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "k_c/surge");
  double k_c_sway = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "k_c/sway");

  double mu_yaw = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "mu/yaw");
  double mu_surge = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "mu/surge");
  double mu_sway = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "mu/sway");

  double Iz = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "bluerov/Iz");

  double m = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "bluerov/mass");

  double Xu = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "bluerov/Xu");
  double Xdu = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "bluerov/Xdu");
  double Xuu = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "bluerov/Xuu");
  
  double Yv = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "bluerov/Yv");
  double Ydv = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "bluerov/Ydv");
  double Yvv = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "bluerov/Yvv");

  double Nr = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "bluerov/Nr");
  double Ndr = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "bluerov/Ndr");
  double Nrr = MedusaGimmicks::getParameters<double>(
            this->nh_p_, "bluerov/Nrr");

  // ROS_WARN_STREAM(this->lambda_yaw_);

  // create SlidingMode object
  this->sm_controller_ = std::make_unique<SlidingMode>(lambda_yaw, lambda_surge, lambda_sway,
                                                       epsilon_yaw, epsilon_surge, epsilon_sway,
                                                       k_c_yaw, k_c_surge, k_c_sway,
                                                       mu_yaw, mu_surge, mu_sway,
                                                       Iz, m,
                                                       Xu, Xdu, Xuu,
                                                       Yv, Ydv, Yvv,
                                                       Nr, Ndr, Nrr);

}

/**
 * @brief  Node class destructor
 */
SlidingModeNode::~SlidingModeNode() {
  
  /* Stop the timer callback */
  this->timer_.stop();

  /* Shutdown the node */
  this->nh_.shutdown();
}

/**
 * @brief  Initialize all the subscribers
 */
void SlidingModeNode::initializeSubscribers() {

  ROS_INFO("Initializing Subscribers for SlidingModeNode");

  std::string state_topic =
      MedusaGimmicks::getParameters<std::string>(this->nh_p_, "topics/subscribers/state");

  std::string yaw_topic =
      MedusaGimmicks::getParameters<std::string>(this->nh_p_, "topics/subscribers/yaw");

  std::string surge_topic =
      MedusaGimmicks::getParameters<std::string>(this->nh_p_, "topics/subscribers/surge");

  std::string sway_topic =
      MedusaGimmicks::getParameters<std::string>(this->nh_p_, "topics/subscribers/sway");

  /* Initialize the subscribers */
  this->state_sub_ =
      nh_.subscribe(state_topic, 10, &SlidingModeNode::stateCallback, this);

  this->yaw_sub_ =
      nh_.subscribe(yaw_topic, 10, &SlidingModeNode::yawCallback, this);

  this->surge_sub_ =
      nh_.subscribe(surge_topic, 10, &SlidingModeNode::surgeCallback, this);

  this->sway_sub_ =
      nh_.subscribe(sway_topic, 10, &SlidingModeNode::swayCallback, this);
}

/**
 * @brief  Method to initialize the publishers of the path following node
 */
void SlidingModeNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for SlidingModeNode");

  /* Get the topic name for the publishers */
  std::string forces_torques_topic = MedusaGimmicks::getParameters<std::string>(this->nh_p_, "topics/publishers/forces_torques");
  std::string debug_topic = MedusaGimmicks::getParameters<std::string>(this->nh_p_, "topics/publishers/debug");

  /* Initialize the publishers */
  this->forces_torques_pub_ = nh_.advertise<auv_msgs::BodyForceRequest>(forces_torques_topic, 1);
  this->debug_pub_ = nh_.advertise<sliding_mode::DebugSlidingMode>(debug_topic, 1);
}

/**
 * @brief  Initialize the timer callback
 */
void SlidingModeNode::initializeTimer() {
  this->timer_ =
      nh_.createTimer(ros::Duration(1.0 / SlidingModeNode::nodeFrequency()),
                      &SlidingModeNode::timerIterCallback, this);

  /* Wait for the start service to start the Path Following */
  this->timer_.start();
}

/**
 * @brief  Setup the Node working frequency
 */
double SlidingModeNode::nodeFrequency() {

  double node_frequency =
      MedusaGimmicks::getParameters<double>(this->nh_p_, "node_frequency", 10);
  ROS_INFO("Node will run at : %lf [hz]", node_frequency);
  return node_frequency;
}

/**
 * @brief Method where the logic is located in order to update the control law
 *
 * @param event  A TimerEvent from ros
 */
void SlidingModeNode::timerIterCallback(const ros::TimerEvent &event) {

  // compute forces and torques, according to Sliding Mode Controller
  struct Tau tau =  this->sm_controller_->computeForcesTorques();

  // update body force request structure
  auv_msgs::BodyForceRequest msg;
  msg.header.stamp = ros::Time::now();
  
  msg.wrench.force.x = tau.u;
  msg.wrench.force.y = tau.v;
  msg.wrench.force.z = 0.0;

  msg.wrench.torque.x = 0.0;
  msg.wrench.torque.y = 0.0;
  msg.wrench.torque.z = tau.r;

  msg.disable_axis = {0, 0, 0, 0, 0, 0};
  
  // publish body forces and torques
  // ROS_INFO_STREAM("TAU_U: " << tau.u);
  // ROS_INFO_STREAM("TAU_V: " << tau.v);
  // ROS_INFO_STREAM("TAU_R: " << tau.r);
  this->forces_torques_pub_.publish(msg);

  // publish debug message
  this->debug_pub_.publish(this->sm_controller_->getDebugMsg());
}

/**
 * @brief  Auxiliar method to stop the algorithm
 */
void SlidingModeNode::stateCallback(const auv_msgs::NavigationStatus &msg) {

  // ROS_INFO_STREAM("State X: " << msg.position.north);

  // update state
  this->sm_controller_->updateVehicleState(msg);
  
  // update surge and sway velocities integrals
  this->sm_controller_->updateIntegralsAndDerivatives();
}

void SlidingModeNode::yawCallback(const std_msgs::Float64 &msg) {
  ROS_WARN_STREAM("updating yaw");
  this->sm_controller_->setYawRef(msg.data);
}

void SlidingModeNode::surgeCallback(const std_msgs::Float64 &msg) {
  ROS_WARN_STREAM("updating surge"); 
  this->sm_controller_->setSurgeRef(msg.data);
}

void SlidingModeNode::swayCallback(const std_msgs::Float64 &msg) {
  ROS_WARN_STREAM("updating sway");
  this->sm_controller_->setSwayRef(msg.data);
}

/**
 * @brief  The main function
 *
 * @param argc  The argument count
 * @param argv  The argument array
 */
int main(int argc, char **argv) {

  ros::init(argc, argv, "sliding_mode_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");

  ROS_INFO("main: instantiating an object of type SlidingModeNode");

  /* Instantiate the Inner Looper Controller Node*/
  SlidingModeNode SlidingModeNode(&nh, &nh_p);

  /* Going into spin and let the timer callback do all the work */
  ros::spin();

  return 0;
}
