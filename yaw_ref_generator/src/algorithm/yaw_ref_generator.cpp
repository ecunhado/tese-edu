#include "algorithm/yaw_ref_generator.hpp"

YawRefGenerator::YawRefGenerator() {
	this->gamma_ = 0.0;
}

YawRefGenerator::~YawRefGenerator() {
    
}

double YawRefGenerator::computeYawRef() {
	double yaw_ref = (1.0 - this->gamma_/2.0)*360 + 270;

	yaw_ref = (yaw_ref >= 360) ? yaw_ref - 360 : yaw_ref;

	return yaw_ref;
}

void YawRefGenerator::setGamma(double value) {
  this->gamma_ = value;
}