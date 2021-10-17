#ifndef DELTAROBOT_KINEMATICS_H
#define DELTAROBOT_KINEMATICS_H

#include <vector>
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>

namespace DeltaRobot
{
	// define default delta parameters
#define DEFAULT_R       0.200 
#define DEFAULT_r       0.045 
#define DEFAULT_L1      0.350 
#define DEFAULT_L2      0.800  
#define DEFAULT_H       0.000 

//define delta robot joint max and min value
#define JOINT_MIN       - 2 * M_PI / 9
#define JOINT_MAX       2 * M_PI / 5

//class
	class DeltaRobotKinematics
	{
	public:
		//delta robot geometric parameters
		double alpha1, alpha2, alpha3; //actuators angles (rad)
		double R, r, L1, L2; //active arm and passive arm radius and length
		double H; //actuator circle to top mounting plate length

	public:
		//construct function
		DeltaRobotKinematics();
		DeltaRobotKinematics(double iR, double ir, double iL1, double iL2, double iH);
		~DeltaRobotKinematics();

		bool ForwardKinematics(Eigen::Matrix<double, 6, 1> th, Eigen::Matrix<double, 6, 1> &pos);
		bool InverseKinematics(Eigen::Matrix<double, 6, 1> pos, Eigen::Matrix<double, 6, 1>& th);
		bool InceptionTriangle(Eigen::Matrix<double, 6, 1> p1, Eigen::Matrix<double, 6, 1> p2, 
			Eigen::Matrix<double, 6, 1> p3, Eigen::Matrix<double, 3, 15> &LX);
		bool InceptionCircle(Eigen::Matrix<double, 6, 1> p, double r,
			std::vector<Eigen::Vector3d> &circle_point);
	};


} // namespace DeltaRobot

#endif //DELTAROBOT_KINEMATICS_H
