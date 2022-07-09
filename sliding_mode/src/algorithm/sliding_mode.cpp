#include "algorithm/sliding_mode.hpp"

SlidingMode::SlidingMode(double lambda_yaw, double lambda_surge, double lambda_sway,
                         double epsilon_yaw, double epsilon_surge, double epsilon_sway,
                         double k_c_yaw, double k_c_surge, double k_c_sway,
                         double mu_yaw, double mu_surge, double mu_sway,
												 double Iz, double m,
												 double Xu, double Xdu, double Xuu,
												 double Yv, double Ydv, double Yvv,
												 double Nr, double Ndr, double Nrr) {
	
	// attributes (coefficients, parameters, etc.)
	this->lambda_yaw_ = lambda_yaw;
	this->lambda_surge_ = lambda_surge;
	this->lambda_sway_ = lambda_sway;

	this->epsilon_yaw_ = epsilon_yaw;
	this->epsilon_surge_ = epsilon_surge;
	this->epsilon_sway_ = epsilon_sway;

	this->k_c_yaw_ = k_c_yaw;
	this->k_c_surge_ = k_c_surge;
	this->k_c_sway_ = k_c_sway;

	this->mu_yaw_ = mu_yaw;
	this->mu_surge_ = mu_surge;
	this->mu_sway_ = mu_sway;

	this->Iz_ = Iz;
	this->m_ = m;

	this->Xu_ = Xu;
	this->Xdu_ = Xdu;
	this->Xuu_ = Xuu;
	
	this->Yv_ = Yv;
	this->Ydv_ = Ydv;
	this->Yvv_ = Yvv;

	this->Nr_ = Nr;
	this->Ndr_ = Ndr;
	this->Nrr_ = Nrr;

	this->m_u_ = this->m_ - this->Xdu_;
	this->m_v_ = this->m_ - this->Ydv_;
	this->m_r_ = this->Iz_ - this->Ndr_;
	this->m_uv_ = this->m_u_ - this->m_v_;

	// updated every iteration
	this->d_u_ = 0;
	this->d_v_ = 0;
	this->d_r_ = 0;

	// reference values
	this->yaw_ref_ = 0;
	this->surge_ref_ = 0;
	this->sway_ref_ = 0;

	// create Vehicle State object
	this->state_ = DSOR::VehicleState();

	// errors
	this->alpha_error_ = 0;
	this->beta_error_ = 0;
	this->yaw_error_ = 0;
	this->surge_error_ = 0;
	this->sway_error_ = 0;
	this->d_yaw_error_ = 0;


	// surge velocity integral
	this->alpha_ = 0;

	// sway velocity integral
	this->beta_ = 0;

	
	// previous surge value
	this->last_surge_ = 0;

	// previous sway value
	this->last_sway_ = 0;

	
	// surge_ref velocity integral
	this->alpha_ref_ = 0;

	// sway_ref velocity integral
	this->beta_ref_ = 0;
	
	
	// surge_ref velocity derivative
	this->d_surge_ref_ = 0;

	// sway_ref velocity derivative
	this->d_sway_ref_ = 0;
	
	// yaw_ref derivative
	this->d_yaw_ref_ = 0;

	// yaw_ref second derivative
	this->dd_yaw_ref_ = 0;

	
	// previous surge_ref value
	this->last_surge_ref_ = 0;

	// previous sway_ref value
	this->last_sway_ref_ = 0;

	// previous yaw_ref value
	this->last_yaw_ref_ = 0;

	// previous derivative of yaw_ref value
	this->last_d_yaw_ref_ = 0;


	// previous timestamp
	this->last_timestamp_ = ros::Time::now();

	// initial time
	this->init_time_ = this->last_timestamp_;

	// iteration
	this->update_iteration_ = 0;
	this->forces_torques_iteration_ = 0;


	// sliding surfaces
	this->s_surge_0_ = 0;
	this->s_surge_0_init_ = 0;
	this->s_surge_ = 0;

	this->s_sway_0_ = 0;
	this->s_sway_0_init_ = 0;
	this->s_sway_ = 0;

	this->s_yaw_0_ = 0;
	this->s_yaw_0_init_ = 0;
	this->s_yaw_ = 0;
}

SlidingMode::~SlidingMode() {
    
}

/**
 * @brief  Updates the vehicle's state
 */
void SlidingMode::updateVehicleState(const auv_msgs::NavigationStatus &msg) {
  
	// assign msg data to this->state_ (rosmsg info auv_msgs/NavigationStatus)
	// x y z
  this->state_.eta1[0] = msg.position.north;
  this->state_.eta1[1] = msg.position.east;
  this->state_.eta1[2] = msg.position.depth;
	// roll pitch yaw
  this->state_.eta2[0] = this->DegreesToRadians(msg.orientation.x);
  this->state_.eta2[1] = this->DegreesToRadians(msg.orientation.y);
  this->state_.eta2[2] = this->DegreesToRadians(msg.orientation.z);
	// u v w
  this->state_.v1[0] = msg.body_velocity.x;
  this->state_.v1[1] = msg.body_velocity.y;
  this->state_.v1[2] = msg.body_velocity.z;
	// p q r
  this->state_.v2[0] = this->DegreesToRadians(msg.orientation_rate.x);
  this->state_.v2[1] = this->DegreesToRadians(msg.orientation_rate.y);
  this->state_.v2[2] = this->DegreesToRadians(msg.orientation_rate.z);

	// update dynamics coefficients
  this->d_u_ = - this->Xu_ - this->Xuu_*abs(this->state_.v1[0]);
	this->d_v_ = - this->Yv_ - this->Yvv_*abs(this->state_.v1[1]);
	this->d_r_ = - this->Nr_ - this->Nrr_*abs(this->state_.v2[2]);
}

/**
 * @brief  Updates integrals and derivatives of surge and sway velocities and yaw;
 * integrals use the trapezoidal approximation
 */
void SlidingMode::updateIntegralsAndDerivatives() {
  // Current time
	// We don't need to subtract the time at the start of the program (init_time)
	// because here we only use the difference between last and current time, which
	// will be the same either way
  ros::Time current_time = ros::Time::now();

  // if after first iteration
  if (this->update_iteration_ > 0) {

    // integrals of CURRENT body velocities
    this->alpha_ += 0.5*(this->last_surge_ + this->state_.v1[0])*(current_time - this->last_timestamp_).toSec();
    this->beta_ += 0.5*(this->last_sway_ + this->state_.v1[1])*(current_time - this->last_timestamp_).toSec();

    // integrals of REFERENCE body velocities
    this->alpha_ref_ += 0.5*(this->last_surge_ref_ + this->surge_ref_)*(current_time - this->last_timestamp_).toSec();
    this->beta_ref_ += 0.5*(this->last_sway_ref_ + this->sway_ref_)*(current_time - this->last_timestamp_).toSec();

    // derivative of REFERENCE body velocities
		// double d_surge_ref = this->d_surge_ref_;
    this->d_surge_ref_ = (this->surge_ref_ - this->last_surge_ref_)/(current_time - this->last_timestamp_).toSec();
    // double d_sway_ref = this->d_sway_ref_;
		this->d_sway_ref_ = (this->sway_ref_ - this->last_sway_ref_)/(current_time - this->last_timestamp_).toSec();
		// prevent large spikes in the derivative of surge REFERENCE
		// this->d_surge_ref_ = (abs(this->d_surge_ref_) > d_surge_ref)

    // derivative of yaw REFERENCE
    this->d_yaw_ref_ = (this->yaw_ref_ - this->last_yaw_ref_)/(current_time - this->last_timestamp_).toSec();
		// prevent large spikes in the derivative of yaw REFERENCE
		this->d_yaw_ref_ = (abs(this->d_yaw_ref_) > 10*abs(this->last_d_yaw_ref_)) ? this->last_d_yaw_ref_ : this->d_yaw_ref_;

		// if after second iteration
		if (this->update_iteration_ > 1) {
			
			// second derivative of yaw REFERENCE
			this->dd_yaw_ref_ = (this->d_yaw_ref_ - this->last_d_yaw_ref_)/(current_time - this->last_timestamp_).toSec();
		}		
  }

	// Anti windup
	// Zeros the alpha and beta errors when error is/crosses zero
	if ((this->state_.v1[0] - this->surge_ref_)*(this->last_surge_ - this->last_surge_ref_) < 0) {
		this->alpha_ = this->alpha_ref_;
	}

	if ((this->state_.v1[1] - this->sway_ref_)*(this->last_sway_ - this->last_sway_ref_) < 0) {
		this->beta_ = this->beta_ref_;
	}

  // update values for next iteration
  this->last_timestamp_ = current_time;
  
  this->last_surge_ = this->state_.v1[0];
  this->last_sway_ = this->state_.v1[1];

  this->last_surge_ref_ = this->surge_ref_;
  this->last_sway_ref_ = this->sway_ref_;
	this->last_yaw_ref_ = this->yaw_ref_;
	this->last_d_yaw_ref_ = this->d_yaw_ref_;

	// new iteration
	this->update_iteration_++;
}

Tau SlidingMode::computeForcesTorques() {

	// compute current time in seconds since the beginning of the simulation
	double current_time = (ros::Time::now() - this->init_time_).toSec();

	// update errors
	this->updateError();

	// create Tau instance
	struct Tau tau;

	tau.u = this->computeTauU(current_time);
	// tau.u = 0;
	
	tau.v = this->computeTauV(current_time);
	// tau.v = 0;

	tau.r = this->computeTauR(current_time);
	// tau.r = 0;

	this->buildDebugMessage(tau.u, tau.v, tau.r);

	this->forces_torques_iteration_++;

	return tau;
}

void SlidingMode::updateError() {
	
	// update error based on current state and references
	this->alpha_error_ = this->alpha_ - this->alpha_ref_;
	this->beta_error_ = this->beta_ - this->beta_ref_;
	this->yaw_error_ = this->computeYawError(this->state_.eta2[2], this->yaw_ref_);
	this->surge_error_ = this->state_.v1[0] - this->surge_ref_;
	this->sway_error_ = this->state_.v1[1] - this->sway_ref_;
	this->d_yaw_error_ = this->state_.v2[2] - this->d_yaw_ref_;
}

double SlidingMode::computeYawError(double yaw, double yaw_ref) {
	
	// compute error given the fact that yaw is in [0, 2PI]
	// the discontinuity introduces errors in the common error computation

	double error = yaw - yaw_ref;
	double error2 = error - this->MathSign(error)*2*M_PI;
	
	return (abs(error) > abs(error2)) ? error2 : error;
}

void SlidingMode::buildDebugMessage(double tau_u, double tau_v, double tau_r) {

	// update debug message
	this->debug_msg_.header.stamp = this->last_timestamp_;
	this->debug_msg_.alpha_error = this->alpha_error_;
	this->debug_msg_.beta_error = this->beta_error_;
	this->debug_msg_.yaw_error = this->yaw_error_;
	this->debug_msg_.surge_error = this->surge_error_;
	this->debug_msg_.sway_error = this->sway_error_;
	this->debug_msg_.d_yaw_error = this->d_yaw_error_;
	this->debug_msg_.tau_u = tau_u;
	this->debug_msg_.tau_v = tau_v;
	this->debug_msg_.tau_r = tau_r;
	this->debug_msg_.s_surge = this->s_surge_;
	this->debug_msg_.s_sway = this->s_sway_;
	this->debug_msg_.s_yaw = this->s_yaw_;
	this->debug_msg_.s_yaw_0 = this->s_yaw_0_;
}

double SlidingMode::computeTauU(double current_time) {

	// compute surge sliding surface component s0
	this->s_surge_0_ = this->surge_error_ + this->lambda_surge_*this->alpha_error_;
	
	// keep initial value of s0
	if (this->forces_torques_iteration_ == 0) {
		this->s_surge_0_init_ = this->s_surge_0_;
	}

	// compute surge sliding surface
	this->s_surge_ = this->s_surge_0_ - this->s_surge_0_init_*exp(-this->k_c_surge_*current_time);

	// compute tau0 (continuous component of tau)
	double tauU_0 = - this->m_v_*this->state_.v1[1]*this->state_.v2[2] + this->d_u_*this->state_.v1[0]
									- this->m_u_*(- this->d_surge_ref_ + this->lambda_surge_*this->surge_error_
																+ this->k_c_surge_*this->s_surge_0_);

	// ROS_WARN_STREAM("\n1 " << - this->m_v_*this->state_.v1[1]*this->state_.v2[2] << "\n" <<
	// 								"2 " << this->d_u_*this->state_.v1[0] << "\n" <<
	// 								"3 " << - this->m_u_*(- this->d_surge_ref_ + this->lambda_surge_*this->surge_error_
	// 																			+ this->k_c_surge_*this->s_surge_0_) << "\n");

	// ROS_WARN_STREAM("\n3.1 " << - this->d_surge_ref_ << "\n" <<
	// 								"3.2 " << this->lambda_surge_*this->surge_error_ << "\n" <<
	// 								"3.3 " << this->k_c_surge_*this->s_surge_0_ << "\n");

	// compute tau1 (discontinuous component of tau)
	double tauU_1 = this->m_u_*this->epsilon_surge_*this->MathSign(this->s_surge_);

	ROS_WARN_STREAM("TAU_U " << tauU_0 + tauU_1);

	// NOT IMPLEMENTED YET
	// LOW PASS FILTER tauU_1

	return tauU_0 + tauU_1;
}

double SlidingMode::computeTauV(double current_time) {

	// compute sway sliding surface component s0
	this->s_sway_0_ = this->sway_error_ + this->lambda_sway_*this->beta_error_;
	
	// keep initial value of s0
	if (this->forces_torques_iteration_ == 0) {
		this->s_sway_0_init_ = this->s_sway_0_;
	}

	// compute sway sliding surface
	this->s_sway_ = this->s_sway_0_ - this->s_sway_0_init_*exp(-this->k_c_sway_*current_time);

	// compute tau0 (continuous component of tau)
	double tauV_0 = this->m_u_*this->state_.v1[0]*this->state_.v2[2] + this->d_v_*this->state_.v1[1]
									- this->m_v_*(- this->d_sway_ref_ + this->lambda_sway_*this->sway_error_
																+ this->k_c_sway_*this->s_sway_0_);

	// compute tau1 (discontinuous component of tau)
	double tauV_1 = this->m_v_*this->epsilon_sway_*this->MathSign(this->s_sway_);

	ROS_WARN_STREAM("TAU_V " << tauV_0 + tauV_1);

	// NOT IMPLEMENTED YET
	// LOW PASS FILTER tauV_1

	return tauV_0 + tauV_1;
}

double SlidingMode::computeTauR(double current_time) {

	// compute yaw sliding surface component s0
	this->s_yaw_0_ = this->d_yaw_error_ + this->lambda_yaw_*this->yaw_error_;
	
	// keep initial value of s0
	if (this->forces_torques_iteration_ == 0) {
		this->s_yaw_0_init_ = this->s_yaw_0_;
	}

	// compute yaw sliding surface
	this->s_yaw_ = this->s_yaw_0_ - this->s_yaw_0_init_*exp(-this->k_c_yaw_*current_time);

	// compute tau0 (continuous component of tau)
	double tauR_0 = - this->m_uv_*this->state_.v1[0]*this->state_.v1[1] + this->d_r_*this->state_.v2[2]
									- this->m_r_*(- this->dd_yaw_ref_ + this->lambda_yaw_*this->d_yaw_error_
																+ this->k_c_yaw_*this->s_yaw_0_);

	// ROS_WARN_STREAM("\n1 " << - this->m_uv_*this->state_.v1[0]*this->state_.v1[1] << "\n" <<
	// 								"2 " << this->d_r_*this->state_.v2[2] << "\n" <<
	// 								"3 " << - this->m_r_*(- this->dd_yaw_ref_ + this->lambda_yaw_*this->d_yaw_error_
	// 																			+ this->k_c_yaw_*this->s_yaw_0_) << "\n");

	// ROS_WARN_STREAM("\n3.1 " << - this->dd_yaw_ref_ << "\n" <<
	// 								"3.2 " << this->lambda_yaw_*this->d_yaw_error_ << "\n" <<
	// 								"3.3 " << this->k_c_yaw_*this->s_yaw_0_ << "\n");

	// compute tau1 (discontinuous component of tau)
	double tauR_1 = this->m_r_*this->epsilon_yaw_*this->MathSign(this->s_yaw_);

	ROS_WARN_STREAM("TAU_R " << tauR_0 + tauR_1);

	// NOT IMPLEMENTED YET
	// LOW PASS FILTER tauR_1

	return tauR_0 + tauR_1;
	
	// double tauR = - this->m_r_*(this->epsilon_yaw_*this->MathSign(this->s_yaw_0_) + this->lambda_yaw_*this->d_yaw_error_ - this->dd_yaw_ref_) 
	// 							- this->m_uv_*this->state_.v1[0]*this->state_.v1[1] + this->d_r_*this->state_.v2[2];
	// return tauR;
}

double SlidingMode::MathSign(double value) {
	return (value < 0) ? -1 : 1;
}

double SlidingMode::DegreesToRadians(double value) {
	return 2*M_PI*value/360;
}

double SlidingMode::RadiansToDegrees(double value) {
	return 360*value/(2*M_PI);
}

void SlidingMode::setYawRef(double value) {
  this->yaw_ref_ = this->DegreesToRadians(value);
}

void SlidingMode::setSurgeRef(double value) {
  this->surge_ref_ = value;
}

void SlidingMode::setSwayRef(double value) {
  this->sway_ref_ = value;
}

sliding_mode::DebugSlidingMode SlidingMode::getDebugMsg() {
	return this->debug_msg_;
}