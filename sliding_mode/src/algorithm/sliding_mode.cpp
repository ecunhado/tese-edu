#include "algorithm/sliding_mode.hpp"

SlidingMode::SlidingMode(double lambda_yaw, double lambda_surge, double lambda_sway,
                         double epsilon_yaw, double epsilon_surge, double epsilon_sway,
                         double k_c_yaw, double k_c_surge, double k_c_sway,
                         double mu_yaw, double mu_surge, double mu_sway,
												 double Iz, double m,
												 double Xu, double Xdu, double Xuu,
												 double Yv, double Ydv, double Yvv,
												 double Nr, double Ndr, double Nrr,
												 double node_frequency) {
	
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

	// path gamma
	this->gamma_ = -1; // if gamma is -1, no path following is being done
	this->last_gamma_ = -1;

	// vehicle flag
	this->vehicle_flag_ = -1; // unknown

	// create Vehicle State object
	this->state_ = DSOR::VehicleState();

	// create low-pass filters
	double lpf_dt;
	lpf_dt = 1.0 / node_frequency;

  this->lpf_R_ = std::make_unique<LowPassFilter>(lpf_dt, 2*M_PI/this->mu_yaw_); // *0.5
	this->lpf_U_ = std::make_unique<LowPassFilter>(lpf_dt, 2*M_PI/this->mu_surge_);
	this->lpf_V_ = std::make_unique<LowPassFilter>(lpf_dt, 2*M_PI/this->mu_sway_);

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

	this->tauR_0_ = 0;
	this->tauR_1_bf = 0;
	this->tauR_1_af = 0;
	this->tauU_1_bf = 0;
	this->tauU_1_af = 0;
	this->tauV_1_bf = 0;
	this->tauV_1_af = 0;

	this->started_control_yaw_ = false;
	this->started_control_surge_ = false;
	this->started_control_sway_ = false; // false if path following publishes sway references
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
    this->d_surge_ref_ = (this->surge_ref_ - this->last_surge_ref_)/(current_time - this->last_timestamp_).toSec();
		this->d_sway_ref_ = (this->sway_ref_ - this->last_sway_ref_)/(current_time - this->last_timestamp_).toSec();

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
	// Anti windup - ZEROING
	// Zeros the alpha and beta errors when error is/crosses zero if alpha/beta errors are too large
	if (false) {
		if ((this->state_.v1[0] - this->surge_ref_)*(this->last_surge_ - this->last_surge_ref_) < 0 &&
				 abs(this->alpha_ - this->alpha_ref_) > 0.1) {
			this->alpha_ = this->alpha_ref_;
		}

		if ((this->state_.v1[1] - this->sway_ref_)*(this->last_sway_ - this->last_sway_ref_) < 0 &&
				 abs(this->beta_ - this->beta_ref_) > 0.1) {
			this->beta_ = this->beta_ref_;
		}
	}

	// Anti windup - SATURATION
	if (true) {
		// alpha
		double alpha_error = this->alpha_ - this->alpha_ref_;
		if (alpha_error > 0.05) {
			this->alpha_ = this->alpha_ref_ + 0.05;
		} else if (alpha_error < -0.05) {
			this->alpha_ = this->alpha_ref_ - 0.05;
		}
		// beta
		double beta_error = this->beta_ - this->beta_ref_;
		if (beta_error > 0.05) {
			this->beta_ = this->beta_ref_ + 0.05;
		} else if (beta_error < -0.05) {
			this->beta_ = this->beta_ref_ - 0.05;
		}
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

void SlidingMode::checkResetAlphaBetaErrors(bool got_surge_ref, bool got_sway_ref) {
	if (!got_surge_ref) {
    this->alpha_ = this->alpha_ref_;
  }

  if (!got_sway_ref) {
    this->beta_ = this->beta_ref_;
  }
}

Tau SlidingMode::computeForcesTorques() {

	// compute current time in seconds since the beginning of the simulation
	double current_time = (ros::Time::now() - this->init_time_).toSec();

	// update errors
	this->updateError();

	// create Tau instance
	struct Tau tau;

	tau.u = this->started_control_surge_ ? this->computeTauU(current_time) : 0;
	tau.v = this->started_control_sway_ ? this->computeTauV(current_time) : 0;
	tau.r = this->started_control_yaw_ ? this->computeTauR(current_time) : 0;
	// tau.u = 0;
	// tau.v = 0;
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
	this->debug_msg_.yaw_ref_true = this->RadiansToDegrees(this->yaw_ref_);
	this->debug_msg_.surge_ref_true = this->surge_ref_;
	this->debug_msg_.sway_ref_true = this->sway_ref_;
	this->debug_msg_.alpha_error = this->alpha_error_;
	this->debug_msg_.beta_error = this->beta_error_;
	this->debug_msg_.yaw_error = this->RadiansToDegrees(this->yaw_error_);
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
	this->debug_msg_.tauR_0 = this->tauR_0_;
	this->debug_msg_.tauR_1_bf = this->tauR_1_bf;
	this->debug_msg_.tauR_1_af = this->tauR_1_af;
	this->debug_msg_.tauU_1_bf = this->tauU_1_bf;
	this->debug_msg_.tauU_1_af = this->tauU_1_af;
	this->debug_msg_.tauV_1_bf = this->tauV_1_bf;
	this->debug_msg_.tauV_1_af = this->tauV_1_af;
	this->debug_msg_.param_yaw_eps = this->epsilon_yaw_;
	this->debug_msg_.param_yaw_kc = this->k_c_yaw_;
	this->debug_msg_.param_yaw_lambda = this->lambda_yaw_;
	this->debug_msg_.param_surge_eps = this->epsilon_surge_;
	this->debug_msg_.param_surge_kc = this->k_c_surge_;
	this->debug_msg_.param_surge_lambda = this->lambda_surge_;
	this->debug_msg_.param_sway_eps = this->epsilon_sway_;
	this->debug_msg_.param_sway_kc = this->k_c_sway_;
	this->debug_msg_.param_sway_lambda = this->lambda_sway_;
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

	// compute tau1 (discontinuous component of tau)
	double tauU_1 = - this->m_u_*this->epsilon_surge_*this->MathSign(this->s_surge_);

	// tau1 value before filter
	this->tauU_1_bf = tauU_1;

	// LOW PASS FILTER tauU_1
	tauU_1 = this->lpf_U_->update(tauU_1);
	
	// tau1 value after filter
	this->tauU_1_af = tauU_1;

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

	// ROS_WARN_STREAM("1: " << this->m_u_*this->state_.v1[0]*this->state_.v2[2]);
	// ROS_WARN_STREAM("2: " << this->d_v_*this->state_.v1[1]);
	// ROS_WARN_STREAM("3: " << - this->m_v_*(- this->d_sway_ref_ + this->lambda_sway_*this->sway_error_
	// 												 + this->k_c_sway_*this->s_sway_0_));

	// ROS_WARN_STREAM("3.1: " << - this->d_sway_ref_);
	// ROS_WARN_STREAM("3.2: " << this->lambda_sway_*this->sway_error_);
	// ROS_WARN_STREAM("3.3: " << this->k_c_sway_*this->s_sway_0_);

	// compute tau1 (discontinuous component of tau)
	double tauV_1 = - this->m_v_*this->epsilon_sway_*this->MathSign(this->s_sway_);

	// tau1 value before filter
	this->tauV_1_bf = tauV_1;

	// LOW PASS FILTER tauV_1
	tauV_1 = this->lpf_V_->update(tauV_1);
	
	// tau1 value after filter
	this->tauV_1_af = tauV_1;

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

	this->tauR_0_ = tauR_0;

	// ROS_WARN_STREAM("1: " << - this->m_uv_*this->state_.v1[0]*this->state_.v1[1]);
	// ROS_WARN_STREAM("2: " << this->d_r_*this->state_.v2[2]);
	// ROS_WARN_STREAM("3: " << - this->m_r_*(- this->dd_yaw_ref_ + this->lambda_yaw_*this->d_yaw_error_
	// 												 + this->k_c_yaw_*this->s_yaw_0_));

	// ROS_WARN_STREAM("3.1: " << - this->dd_yaw_ref_);
	// ROS_WARN_STREAM("3.2: " << this->lambda_yaw_*this->d_yaw_error_);
	// ROS_WARN_STREAM("3.3: " << this->k_c_yaw_*this->s_yaw_0_);

	// compute tau1 (discontinuous component of tau)
	double tauR_1 = - this->m_r_*this->epsilon_yaw_*this->MathSign(this->s_yaw_);

	// tau1 value before filter
	this->tauR_1_bf = tauR_1;

	// LOW PASS FILTER tauR_1
	tauR_1 = this->lpf_R_->update(tauR_1);
	
	// tau1 value after filter
	this->tauR_1_af = tauR_1;
	
	return tauR_0 + tauR_1;
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
	// from [-180 180] to [0 2PI]
	value = (value < 0) ? value + 360 : value;

	if (this->vehicle_flag_ != 6) { // no path following
		this->yaw_ref_ = this->DegreesToRadians(value);
	} else { // during path following
		// only update reference if gamma is not switching back to a previous section of the path
		if (floor(this->gamma_) >= floor(this->last_gamma_)) {
			this->yaw_ref_ = this->DegreesToRadians(value);
			this->last_gamma_ = this->gamma_;
		}
	}
}

void SlidingMode::setSurgeRef(double value) {
  if (this->vehicle_flag_ != 6) { // no path following
		this->surge_ref_ = value;
	} else { // during path following
		// only update reference if gamma is not switching back to a previous section of the path
		if (floor(this->gamma_) >= floor(this->last_gamma_)) {
			this->surge_ref_ = value;
			this->last_gamma_ = this->gamma_;
		}
	}
}

void SlidingMode::setSwayRef(double value) {
	if (this->vehicle_flag_ != 6) { // no path following
		this->sway_ref_ = value;
	} else { // during path following
		// only update reference if gamma is not switching back to a previous section of the path
		if (floor(this->gamma_) >= floor(this->last_gamma_)) {
			this->sway_ref_ = value;
			this->last_gamma_ = this->gamma_;
		}
	}
}

void SlidingMode::setGamma(double value) {
  this->gamma_ = value;
}

void SlidingMode::setFlag(int value) {
	this->vehicle_flag_ = value;

	// when vehicle is not executing path following, last_gamma_ should be reset
	if (value != 6) {
		this->last_gamma_ = -1;
	}

	ROS_INFO_STREAM("FLAG: " << value);
}

sliding_mode::DebugSlidingMode SlidingMode::getDebugMsg() {
	return this->debug_msg_;
}

void SlidingMode::startControlSurge() {
	this->started_control_surge_ = true;
}

void SlidingMode::startControlSway() {
	this->started_control_sway_ = true;
}

void SlidingMode::startControlYaw() {
	this->started_control_yaw_ = true;
}

std::string SlidingMode::setParams(std::string DOF, double k_c, double lambda, double epsilon) {
	double default_value = 0.0;
	std::string DOF_lowercase = DOF;
	DOF_lowercase[0] = std::tolower(DOF_lowercase[0]);

	// define which parameters need to be updated
	std::map<std::string, bool> change_param;
	change_param["k_c"] = k_c != default_value ? true : false;
	change_param["lambda"] = lambda != default_value ? true : false;
	change_param["epsilon"] = epsilon != default_value ? true : false;

	// if no parameters need to be updated
	if (!change_param["k_c"] && !change_param["lambda"] && !change_param["epsilon"]) {
		return "No " + DOF_lowercase + " SM parameters were changed.";
	}

	// update parameters according to what controller (DOF) is specified
	if (DOF == "yaw" || DOF == "Yaw") {
		this->k_c_yaw_ = change_param["k_c"] ? k_c : this->k_c_yaw_;
		this->lambda_yaw_ = change_param["lambda"] ? lambda : this->lambda_yaw_;
		this->epsilon_yaw_ = change_param["epsilon"] ? epsilon : this->epsilon_yaw_;
	} else if (DOF == "surge" || DOF == "Surge") {
		this->k_c_surge_ = change_param["k_c"] ? k_c : this->k_c_surge_;
		this->lambda_surge_ = change_param["lambda"] ? lambda : this->lambda_surge_;
		this->epsilon_surge_ = change_param["epsilon"] ? epsilon : this->epsilon_surge_;
	} else if (DOF == "sway" || DOF == "Sway") {
		this->k_c_sway_ = change_param["k_c"] ? k_c : this->k_c_sway_;
		this->lambda_sway_ = change_param["lambda"] ? lambda : this->lambda_sway_;
		this->epsilon_sway_ = change_param["epsilon"] ? epsilon : this->epsilon_sway_;
	}

	// build output string message
	std::string message = "Changed " + DOF_lowercase + " SM params to:";
	message += change_param["k_c"] ? " k_c = " + std::to_string(k_c) : "";
	message += change_param["lambda"] ? " lambda = " + std::to_string(lambda) : "";
	message += change_param["epsilon"] ? " epsilon = " + std::to_string(epsilon) : "";

	return message;
}