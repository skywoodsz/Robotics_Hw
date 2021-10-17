/*
* author: Zhang Tianlin 
* Date: 2021.10.17
* function: 实现Delat机械臂的正逆运动学以及直线和圆形插值.
*/

#include "windows.h"
#include "DeltaRobotKinematics.h"
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
using namespace std;


void Delta_robot_FK_test(DeltaRobot::DeltaRobotKinematics delta_robot)
{

	Eigen::Matrix<double, 6, 1> theta = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> pos = Eigen::Matrix<double, 6, 1>::Zero();
	
	theta<<-8.556, -28.456, 18.676, 202.19, 0.0, 0.0;
	theta = theta / 180 * M_PI;


	// 单位: rad m
	delta_robot.ForwardKinematics(theta, pos);

	std::cout << "The pos of Delta robot FK is: " << std::endl;

	
	for (size_t i = 0; i < pos.size(); i++)
	{
		if (i > 2)
		{
			std::cout<< pos[i] / M_PI * 180 << std::endl;
		}
		else
		{
			std::cout << pos[i] * 1000 << std::endl;
		}
	}
	

		
}

void Delta_robot_IK_test(DeltaRobot::DeltaRobotKinematics delta_robot)
{

	
	Eigen::Matrix<double, 6, 1> theta = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> pos = Eigen::Matrix<double, 6, 1>::Zero();

	Eigen::Vector3d pos_temp = { -175.239, 78.127, -600.793 };
	Eigen::Vector3d th_temp = { 0.0, 180.0, 18.325 };
	pos_temp /=  1000;
	th_temp = th_temp / 180 * M_PI;

	pos << pos_temp, th_temp;

	// 单位: rad m
	delta_robot.InverseKinematics(pos, theta);

	
	std::cout <<"The pos of Delta robot IK is: \n"
		<<theta / M_PI  * 180<< std::endl;
	
}

void Delta_robot_InceptionTriangle_test(DeltaRobot::DeltaRobotKinematics delta_robot)
{
	Eigen::Matrix<double, 3, 15> LX;
	Eigen::Matrix<double, 6, 1> pos1, pos2, pos3;

	pos1 << 21.741, 185.457, -560.968, 0, 180, 22.190;
	pos2 << -175.239, 78.127, -600.793, 0, 180, 18.325;
	pos3 << 3.616, -131.889, -578.122, 0, 180, 18.998;

	delta_robot.InceptionTriangle(pos1, pos2, pos3, LX);

	vector<Eigen::Matrix<double, 6, 1>> th;
	Eigen::Matrix<double, 6, 1> th_temp;

	/*std::cout << LX.block(0, 0, 3, 1) << std::endl;*/

	int r = LX.cols();
	for (size_t i = 0; i < r; i++)
	{
		Eigen::Matrix<double, 6, 1> pos_temp;
		pos_temp << LX.block(0, i, 3, 1) / 1000, 0, M_PI, 0;

		delta_robot.InverseKinematics(pos_temp, th_temp);
		th.push_back(th_temp);
	}

	ofstream fout("../InceptionTriangleResults.txt");
	fout << "FK: mm degree\n"<< endl;
	for (int i = 0; i < r; ++i)
	{
		fout << LX.block(0, i, 3, 1).transpose()<<"  "
			<<0.0<<"  "<<180.0<<"  "<<0.0 << endl;
	}
	
	fout << "/********/ \n" << endl;
	fout << "IK: degree\n" << endl;
	for (int i = 0; i < th.size(); ++i)
	{
		fout << th[i].transpose() * 180 / M_PI << endl;
	}
	fout << "\n" << endl << endl;
}
void Delta_robot_InceptionCircle_test(DeltaRobot::DeltaRobotKinematics delta_robot)
{
	Eigen::Matrix<double, 6, 1> p;
	double r = 80;
	Eigen::Matrix<double, 3, 36> Circle = Eigen::Matrix<double, 3, 36>::Zero();
	vector<Eigen::Vector3d> circle_point;

	p << -54.339, -25.648, -573.122, 0.0, 180.0, 21.923;
	// mm
	delta_robot.InceptionCircle(p, r, circle_point);
	ofstream fout("../InceptionCircleResults.txt");
	for (int i = 0; i < circle_point.size(); ++i)
	{
		fout << circle_point[i].transpose() << endl;
	}
	fout << "\n" << endl << endl;
}
int main()
{
	DeltaRobot::DeltaRobotKinematics delta_robot;
	// Delta_robot_FK_test(delta_robot);
	// Delta_robot_IK_test(delta_robot);
	// Delta_robot_InceptionTriangle_test(delta_robot);
	// Delta_robot_InceptionCircle_test(delta_robot);
	
	system("pause");
	return 0;
}