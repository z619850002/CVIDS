#include "../../include/optimizer/smooth_euler_optimizer.h"

using namespace std;


double NormalizeAngleDouble(const double& angle_degrees) {
  if (angle_degrees > double(180.0))
  	return angle_degrees - double(360.0);
  else if (angle_degrees < double(-180.0))
  	return angle_degrees + double(360.0);
  else
  	return angle_degrees;
}


void YawPitchRollToRotationMatrixDouble(const double yaw, const double pitch, const double roll, Eigen::Matrix3d & R)
{

	double y = yaw / double(180.0) * double(M_PI);
	double p = pitch / double(180.0) * double(M_PI);
	double r = roll / double(180.0) * double(M_PI);


	R(0 , 0) = cos(y) * cos(p);
	R(0 , 1) = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
	R(0 , 2) = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
	R(1 , 0) = sin(y) * cos(p);
	R(1 , 1) = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
	R(1 , 2) = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
	R(2 , 0) = -sin(p);
	R(2 , 1) = cos(p) * sin(r);
	R(2 , 2) = cos(p) * cos(r);
}
