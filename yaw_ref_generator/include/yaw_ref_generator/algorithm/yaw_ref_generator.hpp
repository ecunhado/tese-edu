#pragma once

#include <ros/ros.h>

class YawRefGenerator {
    public:
        YawRefGenerator();
        ~YawRefGenerator();

        double computeYawRef();
        void setGamma(double value);
    private:
        
        /**
         * @brief Gamma from the current path.
         */
        double gamma_;
};