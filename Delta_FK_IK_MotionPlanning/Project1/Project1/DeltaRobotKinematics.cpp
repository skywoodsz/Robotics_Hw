#include "DeltaRobotKinematics.h"

using namespace std;
namespace DeltaRobot
{
	
	DeltaRobotKinematics::DeltaRobotKinematics()
	{
		alpha1 = 0;
		alpha2 = 2 * M_PI / 3;
		alpha3 = 4 * M_PI / 3;
		H = DEFAULT_H;
		R = DEFAULT_R;
		r = DEFAULT_r;
		L1 = DEFAULT_L1;
		L2 = DEFAULT_L2;

		std::cout << "The param of the delta robot:\n"
			<<"alpha: \n"<< alpha1 * 180 / M_PI<<", "
			<<", "<< alpha2 * 180 / M_PI 
			<<", "<< alpha3 * 180 / M_PI << "\n"
			<<"R: "<< R <<"\n"
			<<"r: " << r << "\n"
			<< "L1: " << L1 << "\n"
			<< "L2: " << L2 << "\n"
			<< "H: " << H	<< std::endl;
		
	}
	DeltaRobotKinematics:: ~DeltaRobotKinematics() {}

	/*
	* Delta robot Forward Kinematics
	* input: Eigen::Vector4d theta
	* output Eigen::Vector3d pos
	*/
	bool DeltaRobotKinematics::ForwardKinematics(Eigen::Matrix<double, 6, 1> th, 
		Eigen::Matrix<double, 6, 1> &pos)
	{
		double th1 = th[0];
		double th2 = th[1];
		double th3 = th[2];
		double th4 = th[3];

		double Px, Py, Pz;

		if ((th1 < JOINT_MIN) || (th1 > JOINT_MAX) || (th2 < JOINT_MIN) || (th2 > JOINT_MAX) || (th3 < JOINT_MIN) || (th3 > JOINT_MAX))
		{
			return(false);
		}


		//calculate B matrix
		double b11 = (r - R - L1 * std::cos(th1)) * std::cos(alpha1);
		double b21 = (r - R - L1 * std::cos(th2)) * std::cos(alpha2);
		double b31 = (r - R - L1 * std::cos(th3)) * std::cos(alpha3);
		double b12 = (r - R - L1 * std::cos(th1)) * std::sin(alpha1);
		double b22 = (r - R - L1 * std::cos(th2)) * std::sin(alpha2);
		double b32 = (r - R - L1 * std::cos(th3)) * std::sin(alpha3);
		double b13 = L1 * std::sin(th1);
		double b23 = L1 * std::sin(th2);
		double b33 = L1 * std::sin(th3);
		//calculate M
		double M1 = b11 * b11 + b12 * b12 + b13 * b13 - b21 * b21 - b22 * b22 - b23 * b23;
		double M2 = b11 * b11 + b12 * b12 + b13 * b13 - b31 * b31 - b32 * b32 - b33 * b33;
		//calculate W
		double W1 = ((b32 - b12)*M1 - (b22 - b12)*M2) / (2 * (b21 - b11)*(b32 - b12) - 2 * (b31 - b11)*(b22 - b12));
		double W2 = ((b23 - b13)*(b32 - b12) - (b33 - b13)*(b22 - b12)) / ((b21 - b11)*(b32 - b12) - (b31 - b11)*(b22 - b12));
		double W3 = ((b31 - b11)*M1 - (b21 - b11)*M2) / (2 * (b22 - b12)*(b31 - b11) - 2 * (b32 - b12)*(b21 - b11));
		double W4 = ((b23 - b13)*(b31 - b11) - (b33 - b13)*(b21 - b11)) / ((b22 - b12)*(b31 - b11) - (b32 - b12)*(b21 - b11));
		//calculate T
		double T1 = W2 * W2 + W4 * W4 + 1;
		double T2 = W1 * W2 + W3 * W4 + b11 * W2 + b12 * W4 - b13;
		double T3 = W1 * W1 + W3 * W3 + 2 * b11*W1 + 2 * b12*W3 + b11 * b11 + b12 * b12 + b13 * b13 - L2 * L2;
		//calculate Pz
		double Pz1 = (T2 + std::sqrt(T2*T2 - T1 * T3)) / T1;
		double Pz2 = (T2 - std::sqrt(T2*T2 - T1 * T3)) / T1;
		if (Pz1 < 0)
		{
			Pz = Pz1;

		}
		else
		{
			Pz = Pz2;
		}
		
		//calculate Px and Py
		Px = W1 - W2 * Pz;
		Py = W3 - W4 * Pz;
		//convert to the top plane coordinate
		Pz = Pz - H;

		pos << Px, Py, Pz, 0.0, M_PI, th4;
		//return
		return(true);
	}

	/*
	* Delta robot Inverse Kinematics
	* input: Eigen::Vector3d pos
	* output Eigen::Vector3d th
	*/
	bool DeltaRobotKinematics::InverseKinematics(Eigen::Matrix<double, 6, 1> pos, 
		Eigen::Matrix<double, 6, 1>& th)
	{
		double Px, Py, Pz;
		double th1, th2, th3;

		Px = pos[0]; 
		Py = pos[1];
		Pz = pos[2];
		
		//variables declare
		double I1, I2, I3;
		double J1, J2, J3;
		double K1, K2, K3;

		//convert to the top plane coordinate
		Pz = Pz + H;
		//calculate I
		I1 = 2 * L1 * ((Px + r * std::cos(alpha1) - R * std::cos(alpha1))*std::cos(alpha1) + (Py + r * std::sin(alpha1) - R * std::sin(alpha1))*std::sin(alpha1));
		I2 = 2 * L1 * ((Px + r * std::cos(alpha2) - R * std::cos(alpha2))*std::cos(alpha2) + (Py + r * std::sin(alpha2) - R * std::sin(alpha2))*std::sin(alpha2));
		I3 = 2 * L1 * ((Px + r * std::cos(alpha3) - R * std::cos(alpha3))*std::cos(alpha3) + (Py + r * std::sin(alpha3) - R * std::sin(alpha3))*std::sin(alpha3));
		//calculate J
		J1 = 2 * L1 * Pz;
		J2 = J1;
		J3 = J1;
		//calculate K
		K1 = pow(Px + r * cos(alpha1) - R * cos(alpha1), 2) + pow(Py + r * sin(alpha1) - R * sin(alpha1), 2) + pow(Pz, 2) + pow(L1, 2) - pow(L2, 2);
		K2 = pow(Px + r * cos(alpha2) - R * cos(alpha2), 2) + pow(Py + r * sin(alpha2) - R * sin(alpha2), 2) + pow(Pz, 2) + pow(L1, 2) - pow(L2, 2);
		K3 = pow(Px + r * cos(alpha3) - R * cos(alpha3), 2) + pow(Py + r * sin(alpha3) - R * sin(alpha3), 2) + pow(Pz, 2) + pow(L1, 2) - pow(L2, 2);
		//check the point if or not in workspace
		if (((J1 * J1 + I1 * I1 - K1 * K1) < 0) || ((J2 * J2 + I2 * I2 - K2 * K2) < 0) || ((J3 * J3 + I3 * I3 - K3 * K3) < 0))
		{
			//out the workspace
			return(false);
		}
		else
		{
			th1 = 2 * atan((-J1 - sqrt(J1*J1 + I1 * I1 - K1 * K1)) / (K1 + I1));
			th2 = 2 * atan((-J2 - sqrt(J2*J2 + I2 * I2 - K2 * K2)) / (K2 + I2));
			th3 = 2 * atan((-J3 - sqrt(J3*J3 + I3 * I3 - K3 * K3)) / (K3 + I3));
			//check the joint variables if or not in limit
			if ((th1 < JOINT_MIN) || (th1 > JOINT_MAX) || (th2 < JOINT_MIN) || (th2 > JOINT_MAX) || (th3 < JOINT_MIN) || (th3 > JOINT_MAX))
			{
				return(false);
			}
		}
		
		th[0] = th1;
		th[1] = th2;
		th[2] = th3;

		th[3] = pos[5];
		th[4] = 0;
		th[5] = 0;

		return(true);
	}
	/*
	* Line Inception
	* x = x0 + mt
	* y = y0 + nt
	* z = z0 + pt
	*/
	bool DeltaRobotKinematics::InceptionTriangle(Eigen::Matrix<double, 6, 1> p1,
		Eigen::Matrix<double, 6, 1> p2, Eigen::Matrix<double, 6, 1> p3,
		Eigen::Matrix<double, 3, 15> &LX)
	{
		Eigen::Vector3d P1, P2, P3;
		Eigen::Matrix<double, 5, 1> t;

		t << 0, 0.25, 0.5, 0.75, 1.0;
		
		for (size_t i = 0; i < P1.size(); i++)
		{
			P1[i] = p1[i];
			P2[i] = p2[i];
			P3[i] = p3[i];
		}
		
		// derict vector
		Eigen::Vector3d v1, v2, v3;

		v1 = P2 - P1;
		v2 = P3 - P2;
		v3 = P1 - P3;
	
		Eigen::Matrix<double, 3, 5> L1_X, L1_X0, L2_X, L2_X0, L3_X, L3_X0;
		
		// Line1 
		L1_X0 << P1, P1, P1, P1, P1;
		L1_X = L1_X0 + v1 * t.transpose();

		// Line2
		L2_X0 << P2, P2, P2, P2, P2;
		L2_X = L2_X0 + v2 * t.transpose();

		// Line2
		L3_X0 << P3, P3, P3, P3, P3;
		L3_X = L3_X0 + v3 * t.transpose();
		
		//std::cout << "L1_X:\n"<< L1_X << std::endl;

		LX << L1_X, L2_X, L3_X;

		return true;
	}
	/*
	* Circle Incepution
	* x = p[0] + rcos(th)
	* y = p[1] + rsin(th)
	*/
	bool DeltaRobotKinematics::InceptionCircle(Eigen::Matrix<double, 6, 1> p, double r,
		std::vector<Eigen::Vector3d> &circle_point)
	{
		Eigen::VectorXf th;
		th.setLinSpaced(36, 0, 2 * M_PI);

		for (size_t i = 0; i < th.size(); i++)
		{
			Eigen::Vector3d point_temp;
			point_temp[0] = p[0] + r * cos(th(i));
			point_temp[1] = p[1] + r * sin(th(i));
			point_temp[2] = p[2];
			
			circle_point.push_back(point_temp);
		}

		return true;
	}

	bool JustTest()
	{
		std::cout << "test successful!" << std::endl;
		return true;
	}
}