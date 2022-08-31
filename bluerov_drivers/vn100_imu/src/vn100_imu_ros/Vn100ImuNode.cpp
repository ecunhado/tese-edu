//=================================================================================================
// Copyright (c) 2017, Medusa Team, Instituto Superior Tecnico
// All rights reserved.
//=================================================================================================

#include "Vn100ImuNode.h"

/*
#######################################################################################################################
@.@ CONSTRUCTOR: put all dirty work of initializations here
#######################################################################################################################
*/

Vn100ImuNode::Vn100ImuNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private) : nh_(*nodehandle), nh_p_(*nodehandle_private) {

    ROS_INFO("in class constructor of Vn100ImuNode");
    loadParams();
    initializePublishers();
    initializeSubscribers();

    // +.+ Handle serial port subscription
    ROS_INFO("Starting on port: %s", p_sPORT_.c_str());

    if (!serial_port_.open(p_sPORT_.c_str(), p_BAUDRATE_)) {
        ROS_ERROR_STREAM("Error opening: " + serial_port_.error_message);
        ros::shutdown();
    }

    // +.+ Give some time to the subscriber nodes
    sleep(1);

    // +.+ Configure device
    configureIMU();

    // +.+ Timer callback initialize
    initializeTimer();
}

/*
#######################################################################################################################
@.@ Destructor
#######################################################################################################################
*/

Vn100ImuNode::~Vn100ImuNode() {

    // +.+ Close serial port
    if (serial_port_.isOpen())
        serial_port_.close();

    // +.+ shutdown publishers
    pub_pp_.shutdown();
    pub_imu_.shutdown();
    pub_raw_.shutdown();
    pub_diag_.shutdown();
    pub_meas_filter_.shutdown();

    // +.+ stop timer
    timer_.stop();

    // +.+ shutdown node
    nh_.shutdown();
    nh_p_.shutdown();
}

// @.@ Load the parameters
void Vn100ImuNode::loadParams() {
    ROS_INFO("Load the Vn100ImuNode parameters");

    p_sPORT_ = MedusaGimmicks::getParameters<std::string>(nh_p_, "port", "/dev/medusa/imu");
    p_BAUDRATE_ = MedusaGimmicks::getParameters<int>(nh_p_, "baudrate", 115200);
    p_config_list_ = MedusaGimmicks::getParameters<std::vector<std::string>>(nh_p_, "config_commands");
    p_check_list_ = MedusaGimmicks::getParameters<std::vector<std::string>>(nh_p_, "check_commands");
    p_use_position_ = MedusaGimmicks::getParameters<bool>(nh_p_, "use_position");
    p_frame_sensor_ = MedusaGimmicks::getParameters<std::string>(nh_p_, "frame_id", "ahrs");

    //TODO:check this flag it was not entering in real vehicles
    p_use_position_ = false;

    //if (p_use_position_) {
        //p_originLat_ = MedusaGimmicks::getParameters<double>(nh_, "/originLat");
        //p_originLon_ = MedusaGimmicks::getParameters<double>(nh_, "/originLon");
    //}
}

// @.@ Member Helper function to set up subscribers;
void Vn100ImuNode::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers for Vn100ImuNode");
    sub_conf_ = nh_p_.subscribe(MedusaGimmicks::getParameters<std::string>(nh_p_, "topics/subscribers/conf", "conf"), 10, &Vn100ImuNode::confCallback, this);
    sub_tare_ = nh_p_.subscribe(MedusaGimmicks::getParameters<std::string>(nh_p_, "topics/subscribers/tare", "tare"), 10, &Vn100ImuNode::tareCallback, this);

    // +.+ Bind subscribers
    serial_port_.subscriberaw(boost::bind(&Vn100ImuNode::rawCallback, this, _1));
    serial_port_.subscribe("VNYMR", boost::bind(&Vn100ImuNode::vnymrCallback, this, _1));
    serial_port_.subscribe("VNYIA", boost::bind(&Vn100ImuNode::vnyiaCallback, this, _1));
    serial_port_.subscribe("VNYCM", boost::bind(&Vn100ImuNode::vnycmCallback, this, _1));
}

// @.@ Member helper function to set up publishers;
void Vn100ImuNode::initializePublishers() {
    ROS_INFO("Initializing Publishers for Vn100ImuNode"); // ---> add publishers here

    //pub_odom_ = nh_p_.advertise<nav_msgs::Odometry>("odom",10);
    pub_meas_filter_ = nh_.advertise<dsor_msgs::Measurement>(MedusaGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/meas_filter", "/measurement/orientation"), 1);
    pub_pp_ = nh_p_.advertise<vn100_imu::imu_pp>(MedusaGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/mag", "imu_pp"), 10);
    pub_imu_ = nh_p_.advertise<medusa_msgs::mIMU>(MedusaGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/data", "data"), 10);
    pub_raw_ = nh_p_.advertise<nmea_msgs::Sentence>(MedusaGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/raw", "raw"), 10);
    pub_diag_ = nh_p_.advertise<diagnostic_msgs::DiagnosticArray>(MedusaGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/diagnostics", "/diagnostics"), 100);
}

// @.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate
void Vn100ImuNode::initializeTimer() {
    timer_ = nh_.createTimer(ros::Duration(1.0 / Vn100ImuNode::nodeFrequency()), &Vn100ImuNode::timerCallback, this);
}

// @.@ Member helper function to wrap the angles of Yaw,Pitch and ROll.
void Vn100ImuNode::bound(medusa_msgs::mIMU &msg) {
    // +.+ Bound attitude between 0-360
    if (msg.Yaw < 0)
        msg.Yaw += 360;
    if (msg.Pitch < 0)
        msg.Pitch += 360;
    if (msg.Roll < 0)
        msg.Roll += 360;

    // +.+ Convert gyros to degrees
    msg.Gx *= 180 / M_PI;
    msg.Gy *= 180 / M_PI;
    msg.Gz *= 180 / M_PI;
}

// @.@ Member helper function to configure the IMU device.
void Vn100ImuNode::configureIMU(void) {
    // +.+ Write configuration commands to the device
    std_msgs::String config_str;
    for (int i = 0; i < p_config_list_.size() && serial_port_.isOpen(); i++) {
        // +.+ Before writting the settings to memory, write the model
        //if (p_use_position_ && p_config_list_[i].compare("VNWNV") == 0) {
            //std::ostringstream vn83;
            //vn83 << "VNWRG,83,1,1,0,0,1000,2016.1," << p_originLat_ << "," << p_originLon_ << ",+100.000";
            //config_str.data = vn83.str();
            //confCallback(config_str);
        //}

        // +.+ Write Configuration Command
        config_str.data = p_config_list_[i];
        confCallback(config_str);
    }

    // +.+ Write check commands to the device
    std_msgs::String check_str;
    for (int i = 0; i < p_check_list_.size(); i++) {
        check_str.data = p_check_list_[i];
        confCallback(check_str);
    }
}

// @.@ Set frequency of the node default is 2.
double Vn100ImuNode::nodeFrequency() {
    double node_frequency;
    nh_.param("node_frequency", node_frequency, 2.0); // the value must be a double.
    ROS_INFO("Node will run at : %lf [hz]", node_frequency);
    return node_frequency;
}

// @.@ Reporting the sensor state
void Vn100ImuNode::reporting(const medusa_msgs::mIMU *imu_msg) {

    //** Instantiate diagnostic message
    diagnostic_msgs::DiagnosticArray diag_msg;
    diag_msg.header.stamp = imu_msg->header.stamp;
    diag_msg.status.push_back(MedusaDiagnostics::setDiagnosisMsg(diagnostic_msgs::DiagnosticStatus::OK,
                                                                 "/Sensors/IMU", "Running", "VN-100"));

    MedusaDiagnostics::addKeyValue(&diag_msg, "Roll", boost::str(boost::format("%.0f") % (imu_msg->Roll)), 0);
    MedusaDiagnostics::addKeyValue(&diag_msg, "Ptich", boost::str(boost::format("%.0f") % (imu_msg->Pitch)), 0);
    MedusaDiagnostics::addKeyValue(&diag_msg, "Yaw", boost::str(boost::format("%.0f") % (imu_msg->Yaw)), 0);

    pub_diag_.publish(diag_msg);
}

/*
#######################################################################################################################
@.@ Callbacks Section / Methods
#######################################################################################################################
*/

// @.@ Iteration via timer callback. It will handle connection dropout
void Vn100ImuNode::timerCallback(const ros::TimerEvent &event) {
    if (!serial_port_.isOpen()) {
        // +.+ Instantiate string to store the message
        std::ostringstream str;

        // +.+ Try to open
        if (!serial_port_.open(p_sPORT_.c_str(), p_BAUDRATE_)) {
            str << "Error opening: " << serial_port_.error_message;
            ROS_ERROR_STREAM_THROTTLE(2.0, str.str());
        } else {
            str << "Port closed: " << serial_port_.error_message;
            ROS_WARN_STREAM_THROTTLE(2.0, str.str());

            // +.+ If open, try to configure device
            configureIMU();
        }
        // +.+ Publish to raw
        rawCallback(str.str());
    }
}

// @.@ rawCallback is called whenever data bytes are available it serves has debug purpose.
void Vn100ImuNode::rawCallback(const std::string &msg) {
    // +.+  Instantiate and fill message
    nmea_msgs::Sentence raw;
    raw.header.stamp = ros::Time::now();
    raw.sentence = msg;
    pub_raw_.publish(raw);
}

// @.@ Configure Callback to send messages to the device
void Vn100ImuNode::confCallback(std_msgs::String str) {
    /*
     * Due to the fact that VN100 take some time to recover
     * after a reset/restore, send the data two times
   */
    if (!serial_port_.write_with_ack(str.data, str.data.substr(0, 5), "VNERR",
                                     3000)) {
        if (!serial_port_.write_with_ack(str.data, str.data.substr(0, 5), "VNERR",
                                         3000))
            ROS_ERROR_STREAM("Error sending message: [" + str.data + "]");
    }
}

// @.@ TARE Callback to send Tare messages.
void Vn100ImuNode::tareCallback(const std_msgs::Empty::ConstPtr &empty) {
    std_msgs::String tare_msg;
    // +.+ Send Tare command
    tare_msg.data = "VNTAR";
    confCallback(tare_msg);
    // +.+ Write to memory
    tare_msg.data = "VNWNV";
    confCallback(tare_msg);
}

/*
* vnymrCallback is called whenever a $VNYMR arrives
* $VNYMR,-170.94,+007.34,+000.70,-1.7866,+0.1747,+2.2113,-00.085,
*        +00.005,-09.847,+00.0003,-00.0012,-00.0001*61
*/
void Vn100ImuNode::vnymrCallback(const std::vector<std::string> &tokens) {
    // +.+ Instantiate message
    medusa_msgs::mIMU imu;
    imu.header.stamp = ros::Time::now();
    imu.Yaw = boost::lexical_cast<double>(tokens[1]);
    imu.Pitch = boost::lexical_cast<double>(tokens[2]);
    imu.Roll = boost::lexical_cast<double>(tokens[3]);
    imu.Mx = boost::lexical_cast<double>(tokens[4]);
    imu.My = boost::lexical_cast<double>(tokens[5]);
    imu.Mz = boost::lexical_cast<double>(tokens[6]);
    imu.Ax = boost::lexical_cast<double>(tokens[7]);
    imu.Ay = boost::lexical_cast<double>(tokens[8]);
    imu.Az = boost::lexical_cast<double>(tokens[9]);
    imu.Gx = boost::lexical_cast<double>(tokens[10]);
    imu.Gy = boost::lexical_cast<double>(tokens[11]);
    imu.Gz = boost::lexical_cast<double>(tokens[12]);

    // +.+ pub measurement message
    dsor_msgs::Measurement meas;
    meas.header.stamp = ros::Time::now();
    meas.header.frame_id = p_frame_sensor_;
    meas.value.emplace_back(imu.Roll * MedusaGimmicks::PI / 180);
    meas.value.emplace_back(imu.Pitch * MedusaGimmicks::PI / 180);
    meas.value.emplace_back(imu.Yaw * MedusaGimmicks::PI / 180);
    meas.value.emplace_back(imu.Gx);
    meas.value.emplace_back(imu.Gy);
    meas.value.emplace_back(imu.Gz);

    meas.noise.emplace_back(0.001);
    meas.noise.emplace_back(0.001);
    meas.noise.emplace_back(0.001);
    meas.noise.emplace_back(0.001);
    meas.noise.emplace_back(0.001);
    meas.noise.emplace_back(0.001);

    // +.+ Bound IMU values
    bound(imu);

    // +.+ Publish
    pub_imu_.publish(imu);

    // +.+ pub_meas_.publish(odom);
    pub_meas_filter_.publish(meas);

    // +.+ Pub mags
    pubMags(imu);

    // Report diagnostics
    reporting(&imu);
}

/*
* vnyiaCallback is called whenever a $VNYIA arrives
* $VNYIA,-170.94,+007.34,+000.70,-1.7866,+0.1747,+2.2113,-00.085,
*        +00.005,-09.847,+00.0003,-00.0012,-00.0001*61
*/
void Vn100ImuNode::vnyiaCallback(const std::vector<std::string> &tokens) {
    // +.+ Instantiate message
    medusa_msgs::mIMU imu;
    imu.header.stamp = ros::Time::now();
    imu.Yaw = boost::lexical_cast<double>(tokens[1]);
    imu.Pitch = boost::lexical_cast<double>(tokens[2]);
    imu.Roll = boost::lexical_cast<double>(tokens[3]);
    imu.Ax = boost::lexical_cast<double>(tokens[4]);
    imu.Ay = boost::lexical_cast<double>(tokens[5]);
    imu.Az = boost::lexical_cast<double>(tokens[6]);
    imu.Gx = boost::lexical_cast<double>(tokens[7]);
    imu.Gy = boost::lexical_cast<double>(tokens[8]);
    imu.Gz = boost::lexical_cast<double>(tokens[9]);

    // +.+ Measurement message filter
    dsor_msgs::Measurement meas;
    meas.header.stamp = ros::Time::now();
    meas.header.frame_id = p_frame_sensor_;
    meas.value.emplace_back(imu.Roll * MedusaGimmicks::PI / 180);
    meas.value.emplace_back(imu.Pitch * MedusaGimmicks::PI / 180);
    meas.value.emplace_back(imu.Yaw * MedusaGimmicks::PI / 180);
    meas.value.emplace_back(imu.Gx);
    meas.value.emplace_back(imu.Gy);
    meas.value.emplace_back(imu.Gz);
    meas.noise.emplace_back(0.001);
    meas.noise.emplace_back(0.001);
    meas.noise.emplace_back(0.001);
    meas.noise.emplace_back(0.001);
    meas.noise.emplace_back(0.001);
    meas.noise.emplace_back(0.001);

    // +.+ Bound IMU values
    bound(imu);

    // +.+ pub meas filter
    pub_meas_filter_.publish(meas);

    // +.+ Report
    reporting(&imu);
}

// @.@ vnycmCallback is called whenever a $VNYCM arrives
void Vn100ImuNode::vnycmCallback(const std::vector<std::string> &tokens) {
    /*
    * The $VNYCM message is only available in firmware 2.0.0.0
    * (15) YPR, Magnet, Accel [m/s^2], Uncomp. Gyros [rad/s], temp
    */

    // Instantiate message
    medusa_msgs::mIMU imu;
    imu.header.stamp = ros::Time::now();
    imu.Yaw = boost::lexical_cast<double>(tokens[1]);
    imu.Pitch = boost::lexical_cast<double>(tokens[2]);
    imu.Roll = boost::lexical_cast<double>(tokens[3]);
    imu.Mx = boost::lexical_cast<double>(tokens[4]);
    imu.My = boost::lexical_cast<double>(tokens[5]);
    imu.Mz = boost::lexical_cast<double>(tokens[6]);
    imu.Ax = boost::lexical_cast<double>(tokens[7]);
    imu.Ay = boost::lexical_cast<double>(tokens[8]);
    imu.Az = boost::lexical_cast<double>(tokens[9]);
    imu.Gx = boost::lexical_cast<double>(tokens[10]);
    imu.Gy = boost::lexical_cast<double>(tokens[11]);
    imu.Gz = boost::lexical_cast<double>(tokens[12]);

    // +.+ Measurement message filter
    dsor_msgs::Measurement meas;
    meas.header.stamp = ros::Time::now();
    meas.header.frame_id = p_frame_sensor_;
    meas.value.emplace_back(imu.Roll * MedusaGimmicks::PI / 180);
    meas.value.emplace_back(imu.Pitch * MedusaGimmicks::PI / 180);
    meas.value.emplace_back(imu.Yaw * MedusaGimmicks::PI / 180);
    meas.value.emplace_back(imu.Gx);
    meas.value.emplace_back(imu.Gy);
    meas.value.emplace_back(imu.Gz);

    meas.noise.emplace_back(0.001);
    meas.noise.emplace_back(0.001);
    meas.noise.emplace_back(0.001);
    meas.noise.emplace_back(0.001);
    meas.noise.emplace_back(0.001);
    meas.noise.emplace_back(0.001);

    // +.+ Bound IMU values
    bound(imu);

    // +.+ Pub message for filter
    pub_meas_filter_.publish(meas);

    // +.+ Publish Mags
    pubMags(imu);

    // +.+ Report
    reporting(&imu);
}

// @.@ Publish magnetometer values to validate calibration
void Vn100ImuNode::pubMags(const medusa_msgs::mIMU &msg) {
    /* Publish only magnetometer data */
    vn100_imu::imu_pp pp_msg;
    pp_msg.header.stamp = msg.header.stamp;
    pp_msg.mag_x = msg.Mx;
    pp_msg.mag_y = msg.My;
    pp_msg.mag_z = msg.Mz;
    pp_msg.mag_abs = sqrt(msg.Mx * msg.Mx + msg.My * msg.My + msg.Mz * msg.Mz);
    pub_pp_.publish(pp_msg);
}

/*
######################################################################################################################
 @.@ Main
#####################################################################################################################
 */
int main(int argc, char **argv) {

    // +.+ ROS set-ups:
    ros::init(argc, argv, "imu"); //node name

    // +.+ create a node handle; need to pass this to the class constructor
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ROS_INFO("main: instantiating an object of type Vn100ImuNode");
    // +.+ instantiate an DsorDepthCellNode class object and pass in pointer to nodehandle for constructor to use
    Vn100ImuNode vn100ImuNode(&nh, &nh_p);

    // +.+ Added to work with timer -> going into spin; let the callbacks do all the work
    ros::spin();

    return EXIT_SUCCESS;
}
