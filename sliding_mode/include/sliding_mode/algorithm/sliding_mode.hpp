#pragma once

#include <ros/ros.h>
#include <dsor_utils/control/vehicle_state.hpp>
#include <dsor_utils/filters/lowpass_filter.hpp>
#include <auv_msgs/NavigationStatus.h>
#include "sliding_mode/DebugSlidingMode.h"

struct Tau {
    double u;
    double v;
    double r;
};

class SlidingMode {
    public:
        SlidingMode(double lambda_yaw, double lambda_surge, double lambda_sway,
                    double epsilon_yaw, double epsilon_surge, double epsilon_sway,
                    double k_c_yaw, double k_c_surge, double k_c_sway,
                    double mu_yaw, double mu_surge, double mu_sway,
                    double Iz, double m,
                    double Xu, double Xdu, double Xuu,
                    double Yv, double Ydv, double Yvv,
                    double Nr, double Ndr, double Nrr,
                    double node_frequency);
        ~SlidingMode();

        void updateVehicleState(const auv_msgs::NavigationStatus &msg);
        void updateIntegralsAndDerivatives();
        Tau computeForcesTorques();
        void updateError();
        double computeYawError(double yaw, double yaw_ref);
        void buildDebugMessage(double tau_u, double tau_v, double tau_r);
        sliding_mode::DebugSlidingMode getDebugMsg();
        double computeTauU(double current_time);
        double computeTauV(double current_time);
        double computeTauR(double current_time);
        void setYawRef(double value);
        void setSurgeRef(double value);
        void setSwayRef(double value);
        void setGamma(double value);
        void setFlag(int value);
        void startControlSurge();
        void startControlSway();
        void startControlYaw();
        std::string setParams(std::string DOF, double k_c, double lambda, double epsilon);

    private:
        double MathSign(double value);
        double DegreesToRadians(double value);
        double RadiansToDegrees(double value);

        /**
         * @brief Debug message for sliding mode.
         */
        sliding_mode::DebugSlidingMode debug_msg_;

        /**
         * @brief Vehicle state attribute 
         */
        DSOR::VehicleState state_;

        /**
         * @brief Low Pass Filter for discontinuous part of
         *        surge control law 
         */
        std::unique_ptr<LowPassFilter> lpf_U_;
        
        /**
         * @brief Low Pass Filter for discontinuous part of
         *        sway control law 
         */
        std::unique_ptr<LowPassFilter> lpf_V_; 
        
        /**
         * @brief Low Pass Filter for discontinuous part of
         *        yaw control law 
         */
        std::unique_ptr<LowPassFilter> lpf_R_;

        /**
         * @brief Yaw Reference.
         */
        double yaw_ref_;
        
        /**
         * @brief Surge Reference.
         */
        double surge_ref_;
        
        /**
         * @brief Sway Reference.
         */
        double sway_ref_;

        /**
         * @brief Gamma from the current path.
         */
        double gamma_;

        /**
         * @brief Last gamma from the current path.
         */
        double last_gamma_;

        /**
         * @brief Last gamma from the current path.
         */
        int vehicle_flag_;

        /**
         * @brief Integral of surge velocity.
         */
        double alpha_;
        
        /**
         * @brief Integral of sway velocity.
         */
        double beta_;

        /**
         * @brief Surge_ref velocity derivative.
         */
        double d_surge_ref_;

        /**
         * @brief Sway_ref velocity derivative.
         */
        double d_sway_ref_;
        
        /**
         * @brief Yaw_ref derivative.
         */
        double d_yaw_ref_;

        /**
         * @brief Yaw_ref second derivative.
         */
        double dd_yaw_ref_;

        /**
         * @brief Error for the integral of surge velocity.
         */
        double alpha_error_;

        /**
         * @brief Error for the integral of sway velocity.
         */
        double beta_error_;

        /**
         * @brief Error of yaw.
         */
        double yaw_error_;

        /**
         * @brief Error of surge velocity.
         */
        double surge_error_;

        /**
         * @brief Error of sway velocity.
         */
        double sway_error_;

        /**
         * @brief Error of yaw velocity.
         */
        double d_yaw_error_;

        /**
         * @brief Surge value in previous iteration.
         */
        double last_surge_;

        /**
         * @brief Sway value in previous iteration.
         */
        double last_sway_;
        
        /**
         * @brief Surge ref velocity integral.
         */
        double alpha_ref_;

        /**
         * @brief Sway ref velocity integral
         */
        double beta_ref_;
        
        /**
         * @brief Previous surge ref value.
         */
        double last_surge_ref_;

        /**
         * @brief Previous sway ref value.
         */
        double last_sway_ref_;

        /**
         * @brief Previous yaw ref value.
         */
        double last_yaw_ref_;

        /**
         * @brief Previous derivative of yaw ref value.
         */
        double last_d_yaw_ref_;

        /**
         * @brief Time stamp in previous iteration.
         */
        ros::Time last_timestamp_;

        /**
         * @brief Initial time stamp.
         */
        ros::Time init_time_;

        /**
         * @brief Derivatives/Integrals update current iteration.
         */
        int update_iteration_;

        /**
         * @brief Forces/Torques computation current iteration.
         */
        int forces_torques_iteration_;

        /**
         * @brief Sliding Surface component for surge.
         */
        double s_surge_0_;

        /**
         * @brief Initial value of sliding surface component for surge.
         */
        double s_surge_0_init_;

        /**
         * @brief Sliding Surface for surge.
         */
        double s_surge_;

        /**
         * @brief Sliding Surface component for sway.
         */
        double s_sway_0_;

        /**
         * @brief Initial value of sliding surface component for sway.
         */
        double s_sway_0_init_;

        /**
         * @brief Sliding Surface for sway.
         */
        double s_sway_;

        /**
         * @brief Sliding Surface component for yaw.
         */
        double s_yaw_0_;

        /**
         * @brief Initial value of sliding surface component for yaw.
         */
        double s_yaw_0_init_;

        /**
         * @brief Sliding Surface for yaw.
         */
        double s_yaw_;

        /**
         * @brief Sliding Mode Parameter.
         */
        double lambda_yaw_;
        
        /**
         * @brief Sliding Mode Parameter.
         */
        double lambda_surge_;
        
        /**
         * @brief Sliding Mode Parameter.
         */
        double lambda_sway_;

        /**
         * @brief Sliding Mode Parameter.
         */
        double epsilon_yaw_;
        
        /**
         * @brief Sliding Mode Parameter.
         */
        double epsilon_surge_;
        
        /**
         * @brief Sliding Mode Parameter.
         */
        double epsilon_sway_;

        /**
         * @brief Sliding Mode Parameter.
         */
        double k_c_yaw_;
        
        /**
         * @brief Sliding Mode Parameter.
         */
        double k_c_surge_;
        
        /**
         * @brief Sliding Mode Parameter.
         */
        double k_c_sway_;

        /**
         * @brief Sliding Mode Parameter.
         */
        double mu_yaw_;
        
        /**
         * @brief Sliding Mode Parameter.
         */
        double mu_surge_;
        
        /**
         * @brief Sliding Mode Parameter.
         */
        double mu_sway_;

        /**
         * @brief Inertia about the Z axis.
         */
        double Iz_;
        
        /**
         * @brief Vehicle mass.
         */
        double m_;
    
        /**
         * @brief Linear Damping.
         */
        double Xu_;

        /**
         * @brief Hydrodynamic Added Mass.
         */
        double Xdu_;
        
        /**
         * @brief Quadratic Damping (?).
         */
        double Xuu_;
        
        /**
         * @brief Linear Damping.
         */
        double Yv_;
        
        /**
         * @brief Hydrodynamic Added Mass.
         */
        double Ydv_;
        
        /**
         * @brief Quadratic Damping (?).
         */
        double Yvv_;

        /**
         * @brief Linear Damping.
         */
        double Nr_;
        
        /**
         * @brief Hydrodynamic Added Mass.
         */
        double Ndr_;
        
        /**
         * @brief Quadratic Damping (?).
         */
        double Nrr_;
        
        /**
         * @brief Dynamic equations' coefficient.
         */
        double m_u_;

        /**
         * @brief Dynamic equations' coefficient.
         */
        double m_v_;

        /**
         * @brief Dynamic equations' coefficient.
         */
        double m_r_;

        /**
         * @brief Dynamic equations' coefficient.
         */
        double m_uv_;

        /**
         * @brief Dynamic equations' coefficient.
         */
        double d_u_;

        /**
         * @brief Dynamic equations' coefficient.
         */
        double d_v_;

        /**
         * @brief Dynamic equations' coefficient.
         */
        double d_r_;

        double tauR_1_bf = 0;
        double tauR_1_af = 0;
        double tauU_1_bf = 0;
        double tauU_1_af = 0;
        double tauV_1_bf = 0;
        double tauV_1_af = 0;

        /**
         * @brief Started control of surge.
         */
        bool started_control_surge_;

        /**
         * @brief Started control of sway.
         */
        bool started_control_sway_;

        /**
         * @brief Started control of yaw.
         */
        bool started_control_yaw_;
};