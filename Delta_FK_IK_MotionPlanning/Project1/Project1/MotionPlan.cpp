#include <iostream>
#include <fstream>
#include "MotionPlan.h"
//#include "HLrobotconfig.h"
#include <algorithm>
#include <Windows.h>
#include <Eigen/Dense>
#include "DeltaRobotKinematics.h"

using namespace std;
//using namespace HLRobot;
using namespace Eigen;

/********************************************************************
ABSTRACT:	构造函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/

CHLMotionPlan::CHLMotionPlan()
{
	

	for (int i = 0; i < 6; i++)
	{
		mJointAngleBegin[i] = 0;
		mJointAngleEnd[i] = 0;
	}

	for (int i = 0; i < 16; i++)
	{
		mStartMatrixData[i] = 0;
		mEndMatrixData[i] = 0;
	}

	mSampleTime = 0.001;
	mVel = 0;
	mAcc = 0;
	mDec = 0;

	
}

/********************************************************************
ABSTRACT:	析构函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
CHLMotionPlan::~CHLMotionPlan()
{

}

/********************************************************************
ABSTRACT:	设置采样时间

INPUTS:		sampleTime			采样时间，单位S

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetSampleTime(double sampleTime)
{
	if (sampleTime < 0.001)
	{
		mSampleTime = 0.001;
	}
	else
	{
		mSampleTime = sampleTime;
	}
}

/********************************************************************
ABSTRACT:	设置运动参数

INPUTS:		vel			速度，单位m/s
			acc			加速度，单位m/s/s
			dec			减速度，单位m / s / s

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetProfile(double vel, double acc, double dec, double t)
{
	mVel = vel;
	mAcc = acc;
	mDec = dec;
	tf_all = t;
}

/********************************************************************
ABSTRACT:	设置规划的起始单位和技术点位

INPUTS:		startPos			起始点位笛卡尔坐标
			endPos				结束点位笛卡尔坐标

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetPlanPoints(PosStruct startPos, PosStruct endPos)
{
	double startAngle[3], endAngle[3];

	startAngle[0] = startPos.yaw * M_PI / 180;
	startAngle[1] = startPos.pitch * M_PI / 180;
	startAngle[2] = startPos.roll * M_PI / 180;

	endAngle[0] = endPos.yaw * M_PI / 180;
	endAngle[1] = endPos.pitch * M_PI / 180;
	endAngle[2] = endPos.roll * M_PI / 180;

	mStartMatrixData[0] = cos(startAngle[0])*cos(startAngle[1])*cos(startAngle[2]) - sin(startAngle[0])*sin(startAngle[2]);
	mStartMatrixData[1] = -cos(startAngle[0])*cos(startAngle[1])*sin(startAngle[2]) - sin(startAngle[0])*cos(startAngle[2]);
	mStartMatrixData[2] = cos(startAngle[0])*sin(startAngle[1]);
	mStartMatrixData[3] = startPos.x / 1000;

	mStartMatrixData[4] = sin(startAngle[0])*cos(startAngle[1])*cos(startAngle[2]) + cos(startAngle[0])*sin(startAngle[2]);
	mStartMatrixData[5] = -sin(startAngle[0])*cos(startAngle[1])*sin(startAngle[2]) + cos(startAngle[0])*cos(startAngle[2]);
	mStartMatrixData[6] = sin(startAngle[0])*sin(startAngle[1]);
	mStartMatrixData[7] = startPos.y / 1000;

	mStartMatrixData[8] = -sin(startAngle[1])*cos(startAngle[2]);
	mStartMatrixData[9] = sin(startAngle[1])*sin(startAngle[2]);
	mStartMatrixData[10] = cos(startAngle[1]);
	mStartMatrixData[11] = startPos.z / 1000;

	mStartMatrixData[12] = 0;
	mStartMatrixData[13] = 0;
	mStartMatrixData[14] = 0;
	mStartMatrixData[15] = 1;

	mEndMatrixData[0] = cos(endAngle[0])*cos(endAngle[1])*cos(endAngle[2]) - sin(endAngle[0])*sin(endAngle[2]);
	mEndMatrixData[1] = -cos(endAngle[0])*cos(endAngle[1])*sin(endAngle[2]) - sin(endAngle[0])*cos(endAngle[2]);
	mEndMatrixData[2] = cos(endAngle[0])*sin(endAngle[1]);
	mEndMatrixData[3] = endPos.x / 1000;

	mEndMatrixData[4] = sin(endAngle[0])*cos(endAngle[1])*cos(endAngle[2]) + cos(endAngle[0])*sin(endAngle[2]);
	mEndMatrixData[5] = -sin(endAngle[0])*cos(endAngle[1])*sin(endAngle[2]) + cos(endAngle[0])*cos(endAngle[2]);
	mEndMatrixData[6] = sin(endAngle[0])*sin(endAngle[1]);
	mEndMatrixData[7] = endPos.y / 1000;

	mEndMatrixData[8] = -sin(endAngle[1])*cos(endAngle[2]);
	mEndMatrixData[9] = sin(endAngle[1])*sin(endAngle[2]);
	mEndMatrixData[10] = cos(endAngle[1]);
	mEndMatrixData[11] = endPos.z / 1000;

	mEndMatrixData[12] = 0;
	mEndMatrixData[13] = 0;
	mEndMatrixData[14] = 0;
	mEndMatrixData[15] = 1;

	double angle1, angle2, angle3, angle4, angle5, angle6;
	// 正逆解
	/*HLRobot::SetRobotEndPos(startPos.x, startPos.y, startPos.z, startPos.yaw, startPos.pitch, startPos.roll, startPos.config);
	HLRobot::GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6);*/

	Eigen::Matrix<double, 6, 1> theta = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> pos = Eigen::Matrix<double, 6, 1>::Zero();
	pos << startPos.x / 1000, startPos.y / 1000, startPos.z / 1000,
		startAngle[2], startAngle[1], startAngle[0];

	
	DeltaRobot::DeltaRobotKinematics::InverseKinematics(pos, theta);

	std::cout << "start point:" << endl;

	for (size_t i = 0; i < theta.size(); i++)
	{
		mJointAngleBegin[i] = theta[i] * 180 / M_PI;
		std::cout << mJointAngleBegin[i]  << std::endl;
	}

	/*mJointAngleBegin[0] = angle1;
	mJointAngleBegin[1] = angle2;
	mJointAngleBegin[2] = angle3;
	mJointAngleBegin[3] = angle4;
	mJointAngleBegin[4] = angle5;
	mJointAngleBegin[5] = angle6;*/

	theta = Eigen::Matrix<double, 6, 1>::Zero();
	pos = Eigen::Matrix<double, 6, 1>::Zero();

	pos << endPos.x / 1000, endPos.y / 1000, endPos.z / 1000,
		endAngle[2], endAngle[1], endAngle[0];

	DeltaRobot::DeltaRobotKinematics::InverseKinematics(pos, theta);

	std::cout << "end point:" << endl;
	for (size_t i = 0; i < theta.size(); i++)
	{
		mJointAngleEnd[i] = theta[i] * 180 / M_PI;
		std::cout << mJointAngleEnd[i] << std::endl;
	}
	
	/*HLRobot::SetRobotEndPos(endPos.x, endPos.y, endPos.z, endPos.yaw, endPos.pitch, endPos.roll, endPos.config);
	HLRobot::GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6);
	mJointAngleEnd[0] = angle1;
	mJointAngleEnd[1] = angle2;
	mJointAngleEnd[2] = angle3;
	mJointAngleEnd[3] = angle4;
	mJointAngleEnd[4] = angle5;
	mJointAngleEnd[5] = angle6;*/

}

/********************************************************************
ABSTRACT:	运动轨迹规划部分（以关节空间为例）

INPUTS:		pos						二维位置向量

OUTPUTS:	pos						二维位置向量（第一维：位置个数，第二维：每个轴的关节角度，单位弧度）

RETURN:		<none>
***********************************************************************/

/******
 * 参考步骤
 * 步骤1：创建文件并写入初始角度
 * 步骤2：计算每个轴旋转的角度
 * 步骤3：计算每个轴移动到终止点所需要时间
 * 步骤4：根据采样时间计算离散点位数
 * 步骤5：根据离散点位数得到每刻的关节角数值
 * 步骤6：将每时刻的关节角数值写入到文件
 */
void CHLMotionPlan::GetPlanPoints()
{
	// step1: 创建文件并写入初始角度
	ofstream fout("../data.txt");
	fout << mJointAngleBegin[0] << "  "
			<< mJointAngleBegin[1] << "  "
			<< mJointAngleBegin[2] << "  "
			<< mJointAngleBegin[3] << "  "
			<< mJointAngleBegin[4] << "  "
			<< mJointAngleBegin[5] << "  ";
	fout << endl;//保存初始的时间、六个关节角度

	// step2 3 4: 各轴梯形速度规划
	/*vector<double>t1, t2, t12, td12, n_sample, th_vel;*/
	vector<double> tf, tb, n_sample;
	for (size_t i = 0; i < 6; i++)
	{
		double tf_temp, tb_temp;
		tf_temp = tf_all;
		Planning2(mJointAngleBegin[i], mJointAngleEnd[i], tf_temp, tb_temp);

		tf.push_back(tf_temp);
		tb.push_back(tb_temp);
		n_sample.push_back(tf_temp / mSampleTime);
		/*std::cout << "n_sample: " << std::endl;
		std::cout << tf_temp / mSampleTime << std::endl;*/
	}


	// step5 6 预测&写入
	double th_end_up = 0;
	vector<vector<double>> th_all;
	for (size_t i = 0; i < n_sample.size(); i++)
	{
		double t = 0;
		vector<double> th;
		for (size_t j = 0; j < n_sample[i]; j++)
		{
			double th_temp;
			Predict2(mJointAngleBegin[i], mJointAngleEnd[i], t,
						tf[i], tb[i], th_temp);
			th.push_back(th_temp);
			t = j * mSampleTime;
			//std::cout <<"th_temp: "<<th_temp << std::endl;
			//std::cout <<"t: "<< t << std::endl;
		}
		th_all.push_back(th);
	}
	std::cout << "th_all_size: " << th_all.size() << std::endl;
	std::cout << "th_size: " << th_all[0].size() << std::endl;

	for (size_t i = 0; i < th_all[0].size(); i++)
	{
		for (size_t j = 0; j < th_all.size(); j++)
		{
			fout << th_all[j][i] << " ";
		}
		fout << endl;
	}
	fout << "\n" << endl << endl;
	
}

// 无via点笛卡尔空间直线规划
void CHLMotionPlan::GetPlanPoints_line()
{
	ofstream fout("../data_Descartes.txt");
	ofstream fout2("../data_Descartes_th.txt");
	// 做 point_size - 1次规划
	for (size_t i = 0; i < 4 - 1; i++)
	{
		double p1[6], p2[6];
		p1[0] = point[i].x; 	p2[0] = point[i + 1].x;
		p1[1] = point[i].y;		p2[1] = point[i + 1].y;
		p1[2] = point[i].z;		p2[3] = point[i + 1].yaw;
		p1[3] = point[i].yaw;	p2[4] = point[i + 1].pitch;
		p1[4] = point[i].pitch; p2[5] = point[i].roll;
		p1[5] = point[i].roll;	p2[2] = point[i + 1].z;

		std::cout << "st_now: " << point[i].x << " " << point[i].y << " "
			<< point[i].z << " " << point[i].yaw << std::endl;

		// 做6轴规划
		vector<double> tf, tb, n_sample;
		vector<bool> neg;
		for (size_t i = 0; i < 4; i++)
		{
			double tf_temp, tb_temp;
			bool nagetive = false;
			tf_temp = tf_all;
			std::cout << "tf_all:" << tf_all<<std::endl;
			if (p2[i] - p1[i] < 0)
			{
				nagetive = true;
				p1[i] = -p1[i];
				p2[i] = -p2[i];
			}
			neg.push_back(nagetive);

			Planning2(p1[i], p2[i], tf_temp, tb_temp);

			n_sample.push_back(tf_temp / mSampleTime);

	
			tf.push_back(tf_temp);
			tb.push_back(tb_temp);
			
			std::cout << "tf: " << tf[i] << std::endl;
			std::cout << "tb: " << tb[i] << std::endl;
			/*std::cout << "n_sample: " << std::endl;
			std::cout << tf_temp / mSampleTime << std::endl;*/
		}
		
		double th_end_up = 0;
		vector<vector<double>> th_all;
		for (size_t i = 0; i < 4; i++)
		{
			double t = 0;
			vector<double> th;
			for (size_t j = 0; j < n_sample[i]; j++)
			{
				double th_temp;
				/*if (neg[i])
				{
					p1[i] = -p1[i];
					p2[i] = -p2[i];
				}*/

				Predict2(p1[i], p2[i], t,
					tf[i], tb[i], th_temp);

				if (neg[i])
				{
					th_temp = -th_temp;
				}
				th.push_back(th_temp);
				t = j * mSampleTime;
				/*std::cout <<"tf: "<<tf[0] << std::endl;
				std::cout <<"tb: "<< tb[0] << std::endl;*/
			}
			th_all.push_back(th);
		}
		std::cout << "th_all_size: " << th_all.size() << std::endl;
		std::cout << "th_size: " << th_all[0].size() << std::endl;

		for (size_t i = 0; i < th_all[0].size(); i++)
		{
			for (size_t j = 0; j < th_all.size(); j++)
			{
				fout << th_all[j][i] << " ";
			}
			fout << endl;
		}
		// fout << "\n" << endl << endl;


		Eigen::Matrix<double, 6, 1> theta = Eigen::Matrix<double, 6, 1>::Zero();
		Eigen::Matrix<double, 6, 1> pos = Eigen::Matrix<double, 6, 1>::Zero();
		
		
		for (size_t i = 0; i < th_all[0].size(); i++)
		{
			pos << th_all[0][i] / 1000, th_all[1][i] / 1000, th_all[2][i] / 1000,
				0, M_PI, th_all[3][i] * M_PI / 180;
			DeltaRobot::DeltaRobotKinematics::InverseKinematics(pos, theta);
		
			for (size_t j = 0; j < theta.size(); j++)
			{
				fout2 << theta[j] * 180 / M_PI << " ";
			}
			fout2 << endl;
		}
		// fout2 << "\n" << endl << endl;
	}
	fout.close();
	fout2.close();

}



void CHLMotionPlan::Planning2(double th1, double th2, double &t, double &tb)
{
	double temp;
	if (mAcc < 4 * abs(th2 - th1) / (t * t))
	{
		mAcc = 4 * (th2 - th1) / (t * t) * 2;
	}
	temp = sqrt(mAcc * mAcc * t * t - 4 * mAcc * (th2 - th1));
	tb = 0.5 * t - temp / (2 * mAcc);

	std::cout << "mAcc: " << mAcc << std::endl;
	std::cout << "mVel: " << mAcc * tb << std::endl;
	std::cout << "temp: " << temp << std::endl;
}

/*
* 无via点梯形速度规划
*/
void CHLMotionPlan::Predict2(double th_st, double th_end, double t,
	double tf, double tb, double &th)
{
	if (t < tb)
	{
		th = th_st + 0.5 * mAcc * t * t;
	}
	else if (t < tf - tb)
	{
		th = th_st + 0.5 * mAcc * tb * tb + mAcc * tb * (t - tb);
	}
	else
	{
		th = th_end - 0.5 * mAcc * (tf - t) * (tf - t);
	}
}


/*
* 梯形速度规划 有设置速度与加速度求各段时间
* Input: th1 起始值， th2 末端值
* Output td12 用时时长, t1 加速时长, t2 减速时长, t12 直线时长
*/
void CHLMotionPlan::Planning(double th1, double th2,
double &t1, double &t2, double &t12, double &td12, double &th_vel)
{
	


	/*double th_acc;
	th_acc = math_sign(th2 - th1) * mAcc;
	t1 = td12 - sqrt(td12 * td12 - 2 * (th2 - th1) / th_acc);
	th_vel = (th2 - th1) / td12 - 0.5 * t1;
	t2 = (th_vel - 0) / th_acc;
	t12 = td12 - t1 - 0.5 * t2;*/

	/*double th_acc, th_vel;
	th_vel = mVel;
	th_acc = math_sign(th2 - th1) * mAcc;

	t1 = (th_vel - 0) / th_acc;
	t2 = (th_vel - 0) / th_acc;
	td12 = (t1 * t1 + 2 * (th2 - th1) / th_acc) / (2 * t1);
	t12 = td12 - t1 - t2; */
}

/*
* 梯形速度预测
* Input： t1 二次过度时长， t12 直线段时长，
*		t2 末尾二次过度时长，t 当前时间
*		th_vel 直线速度, th_st 开始角度, 
*		th_end 结束角度， th_end_up 上一阶段结束角度
* Output: th 预测角度
*/
void CHLMotionPlan::PathPredict(double t1, double t12, double t2, double t,
	double th_vel, double th_st, double th_end, double &th_end_up,
	double &th, double &vel)
{
	/*
	* 1st二次: 0 < t < T1
	* 直线: T1 < t < T12
	* 2st二次: T12 < t < T2
	*/
	double T1, T12, T2; // 时刻

	T1 = t1; T12 = T1 + t12; T2 = T12 + t2;
	//std::cout << "T1: "<< T1 << std::endl;
	//std::cout << "T12: " << T12 << std::endl; // 2.74049
	//std::cout << "T2: " << T2 << std::endl;
	if (t < T1) // 1st二次
	{
		th = th_st + 0.5 * mAcc * t * t;
		th_end_up = th;
		/*std::cout << "th_end_up: " << th_end_up << std::endl;*/
	}
	else if ((T1 <= t) & (t < T12))// 直线
	{
		th = th_end_up + mAcc * T1 * (t - T1);
	}
	else
	{
		th = th_end - 0.5 * mAcc * (T2 - t) * (T2 - t);
	}
}

double CHLMotionPlan::math_sign(double th)
{
	return th / abs(th);
}

void CHLMotionPlan::InitPickPut(PosStruct p1, PosStruct p2, PosStruct p3, PosStruct p4)
{
	PickPutP1 = p1; // mm deg
	PickPutP2 = p2;
	PickPutP3 = p3;
	PickPutP4 = p4;

	point.push_back(p1);
	point.push_back(p2);
	point.push_back(p3);
	point.push_back(p4);
}