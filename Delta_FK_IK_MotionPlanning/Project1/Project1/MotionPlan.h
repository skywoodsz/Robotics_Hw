#pragma once
#include <vector>
#include "MotionPlan.h"
#include "DeltaRobotKinematics.h"
#include <cmath>
#include <math.h>
using namespace std;

struct PosStruct
{
	double x;				// x坐标，单位mm
	double y;				// y坐标，单位mm
	double z;				// z坐标，单位mm
	double yaw;				// yaw坐标，单位度
	double pitch;			// pitch坐标，单位度
	double roll;			// roll坐标，单位度
};

class CHLMotionPlan : public DeltaRobot::DeltaRobotKinematics
{
public:
	double mJointAngleBegin[6];					//起始点位的关节角度,单位度
	double mJointAngleEnd[6];					//结束点位的关节角度，单位度
	double mStartMatrixData[16];				//起始点位的转换矩阵数组
	double mEndMatrixData[16];					//结束点位的转换矩阵数组
	double mSampleTime;							//采样点位，单位S
	double mVel;								//速度，单位m/s
	double mAcc;								//加速度，单位m/s/s
	double mDec;								//减速度，单位m / s / s
	bool mConfig[3];							//机器人姿态
	double tf_all; //总时长
	PosStruct PickPutP1, PickPutP2, PickPutP3, PickPutP4;
	vector<PosStruct> point;

	

public:
	CHLMotionPlan();
	virtual ~CHLMotionPlan();

	void SetSampleTime(double sampleTime);		//设置采样时间
	void SetPlanPoints(PosStruct startPos, PosStruct endPos);		//输入起始点位和结束点位的笛卡尔坐标
	void SetProfile(double vel, double acc, double dec, double t);			//设置运动参数，速度、加速度和减速度
	void GetPlanPoints();											//关节空间梯形速度规划
	void GetPlanPoints_line();       								//笛卡尔空间直线轨迹梯形速度规划
	void Planning(double th1, double th2,
		double &t1, double &t2, double &t12, double &td12, double &th_vel); // 梯形规划
	void PathPredict(double t1, double t12, double t2, double t,
		double th_vel, double th_st, double th_end, double &th_end_up,
		double &th, double &vel); // 梯形规划预测
	double math_sign(double th); // 符号函数
	void Planning2(double th1, double th2, double &t, double &tb);
	void Predict2(double th1, double th2, double t,
		double tf, double tb, double &th);
	void InitPickPut(PosStruct p1, PosStruct p2, PosStruct p3, PosStruct p4);
};

