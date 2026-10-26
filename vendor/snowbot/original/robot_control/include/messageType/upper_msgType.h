#ifndef __upper_msgType__H__
#define __upper_msgType__H__

#include <iostream>
#include <vector>
#include <string>
#include <map>

#include <cmath>
#include <array>

#include "robot_control/msgType.h"

#

/*
        by 霜云喵
==========================
消上位机控制息传输类型及其配套信息处理函数
==========================

==========================
*/

//------------------------
//限位传输消息类型
struct JntLimMsg_trans
{
	bool ifUseJntLim;
	int limJntNum;
	std::vector<int> limJntID;
	std::vector<double> limJntAng;
};

//限位传输消息类型对应函数
extern std::map<int, std::string> JntLabelDict;
std::vector<std::string> JntID_to_label(std::vector<int> JntID);

//------------------------
/*	单关节点位控制参数
------------------
*/
struct SingleDot_ctlMsg{
	//单关节轨迹时间
	std::vector<double> JntTrajTime_L;
	std::vector<double> JntTrajTime_R;

	//关节加速时间占比
	std::vector<double> JntAccRate_L;
	std::vector<double> JntAccRate_R;

	//关节点位控制静止信息包装
	bool ifReDefineStabJnt;
	std::vector<runinStabMsgWrap> lArm_stabMsg;
	std::vector<runinStabMsgWrap> rArm_stabMsg;

	//是否启用循环点位控制
	int ifUseTemp;
};

/*	目标位置传输消息类型
--------------------
传输协议：
	1. 若为笛卡尔空间位置控制模式
	lArm_desMsg，rArm_desMsg -- 前3个数为末端位置坐标；后4个数为末端姿态欧拉四元数(w,x,y,z)
	2. 若为关节空间控制模式
	lArm_desMsg，rArm_desMsg -- 依次为七个关节的角度
*/
struct ctlDesMsg_trans
{
	SingleDot_ctlMsg m_SingleDotCtlMsg;		//单点位控制命令

	int ifInitUseRet;						//初始化过程是否用到返回值

	std::vector<double> lArm_desMsg;
	std::vector<double> rArm_desMsg;


	std::vector<int> lArm_rigidity_mod;			//刚度等级
	std::vector<double> lArm_JntKp;				//关节Kp
	std::vector<double> lArm_JntKd;				//关节Kd

	std::vector<int> rArm_rigidity_mod;
	std::vector<double> rArm_JntKp;
	std::vector<double> rArm_JntKd;
};

//------------------------
/*	辅助消息类型 -- 关节轨迹信息传输包裹
---------------------------------------
协议：
	1. 重力补偿控制量 -- gravCompMod
		0 -- 无重力补偿；1 -- 有重力补偿，目标值作为输入量；2 -- 有重力补偿，反馈值作为输入量
	2. 关节限位控制量 -- JntStabMod
	3. 运动控制量 -- ctlTransMod
		0 -- 归零控制；1 -- 关节模式；2 -- 点位控制； 3 -- 关节轨迹控制； 4 -- 点位轨迹控制； 5 -- 零力拖动模式
	4. 目标力位 -- ctlDesMsg
	5. 力前馈开关量 -- ifUse_TorFeedForward
		默认值：0
		0 -- 不使用；1 -- 使用

	**备注 -- 底层力控尚未开发，所以力前馈控制量恒定为0
*/
struct armTrajMsg_Trans{
	int gravCompMod;      				//重力补偿控制模式
	JntLimMsg_trans JntStabMod_l;   	//左臂关节限位控制模式
	JntLimMsg_trans JntStabMod_r;   	//右臂关节限位控制模式	
	int ctlTransMod;      				//运动控制模式
	ctlDesMsg_trans ctlDesMsg;        	//目标位置控制量
	bool ifReachEnd;      				//是否到达目标位置
	bool ifUseTorFeedForward;  			//是否使用力前馈控制
};

//--------------------
//辅助函数 -- 欧拉角转化为欧拉四元数位姿
struct Quaternion{
	double w, x, y, z;
};
struct Pose{
	double x, y, z;
	Quaternion q;
};

Pose rpyToQuaternion(double x, double y, double z, double roll, double pitch, double yaw);

#endif