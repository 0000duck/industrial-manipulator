#pragma once
#include <math.h>
#include <string.h>
#include <fstream>
#include <stdlib.h>
#include<vector>
#include <iostream>
using namespace std;

#define PI M_PI
#define F64 double

/*====================================================================================*\
* 内容：定义常量

* 作者：徐凡

* 创建日期：2015-7-22

* 修改日期：2015-7-22
\*====================================================================================*/

#define DH_JtoPPU_A 1740.8125 //机器人关节角度到PPU的比例关系
#define DH_JtoPPU_B  1740.8125
#define DH_JtoPPU_C  1831.83125
#define DH_JtoPPU_D  1137.78125
#define DH_JtoPPU_E  1149.16
#define DH_JtoPPU_F  910.0

#define DH_RPPUtoWPPU_A 4.0 //反馈的PPU到指令PPU的比例关系
#define DH_RPPUtoWPPU_B  4.0
#define DH_RPPUtoWPPU_C  4.0
#define DH_RPPUtoWPPU_D  4.0
#define DH_RPPUtoWPPU_E  4.0
#define DH_RPPUtoWPPU_F  4.0
//using namespace base;



//修改时间：2016-2-6 13:32:59
//修改人：党渊渊
#define  JOINT_A   0.0   //定义JOINT_ 为theta值
#define  JOINT_B   PI/2
#define  JOINT_C   0.0
#define  JOINT_D   0.0
#define  JOINT_E   -PI/2
#define  JOINT_F   PI/2

#define  DH_jointrangeU_A   180.0 //机器人关节运动范围上限
#define  DH_jointrangeU_B   80.0
#define  DH_jointrangeU_C   160.0 //修改时间：2016-2-6 13:34:58，修改人：党渊渊
#define  DH_jointrangeU_D   240.0
#define  DH_jointrangeU_E   200.0 //135.0 //修改时间：2016-2-6，修改人：党渊渊
#define  DH_jointrangeU_F   360.0

#define  DH_jointrangeD_A   -180.0 //机器人关节运动范围下限
#define  DH_jointrangeD_B   -130.0
#define  DH_jointrangeD_C   -70.0 //-170.0 //修改时间：2016-2-6，修改人：党渊渊
#define  DH_jointrangeD_D   -240.0
#define  DH_jointrangeD_E   -30.0 //-135.0  //修改时间：2016-2-6，修改人：党渊渊
#define  DH_jointrangeD_F   -360.0

#define  DH_O_A   40.0 //机器人DH的a，连杆长度
#define  DH_O_B   315.0
#define  DH_O_C   70.0
#define  DH_O_D   0.0
#define  DH_O_E   0.0
#define  DH_O_F   0.0

//修改时间：2016-2-6，修改人：党渊渊
#define  DH_P_A   330.0 //机器人DH的d，连杆偏移量
#define  DH_P_B   0.0
#define  DH_P_C   310.0
#define  DH_P_D   0.0
#define  DH_P_E   0.0
#define  DH_P_F   70.0

//修改时间：2016-2-6，修改人：党渊渊
#define  DH_Q_A   PI/2    //机器人DH的alpha，扭转角
#define  DH_Q_B   0.0
#define  DH_Q_C   PI/2
#define  DH_Q_D   -PI/2
#define  DH_Q_E   PI/2
#define  DH_Q_F   0.0

//定义机器人的关节减速比

#define GEAR_RATIO_J1 153
#define GEAR_RATIO_J2 153
#define GEAR_RATIO_J3 161
#define GEAR_RATIO_J4 100
#define GEAR_RATIO_J5 100
#define GEAR_RATIO_J6 80
#define GEAR_RATIO_1 153 //减速比
#define GEAR_RATIO_2 153
#define GEAR_RATIO_3 -161
#define GEAR_RATIO_4 -100
#define GEAR_RATIO_5 -101
#define GEAR_RATIO_6 80
#define SERVE_MOTOR_MAX_1 2000 // 电机最大转速，单位：转/分钟
#define SERVE_MOTOR_MAX_2 3000
#define SERVE_MOTOR_MAX_3 3000
#define SERVE_MOTOR_MAX_4 4000
#define SERVE_MOTOR_MAX_5 3000
#define SERVE_MOTOR_MAX_6 4000
/*
#define JOINT_SPEED_MAX_1	0.5
#define JOINT_SPEED_MAX_2	0.6
#define JOINT_SPEED_MAX_3	0.6
#define JOINT_SPEED_MAX_4	1.2
#define JOINT_SPEED_MAX_5	1.2
#define JOINT_SPEED_MAX_6	1.6
#define JOINT_ACC_MAX_1   0.013   // 关节最大加速度，单位：弧度/秒*秒
#define JOINT_ACC_MAX_2   0.015
#define JOINT_ACC_MAX_3   0.015
#define JOINT_ACC_MAX_4   0.03
#define JOINT_ACC_MAX_5   0.03
#define JOINT_ACC_MAX_6   0.04
*/
#define JOINT_SPEED_MAX_1		1.92   /////////////////max motor speed=3000r/min
#define JOINT_SPEED_MAX_2		3.11
#define JOINT_SPEED_MAX_3		3.11
#define JOINT_SPEED_MAX_4		2.13
#define JOINT_SPEED_MAX_5		5.23
#define JOINT_SPEED_MAX_6		5.23
#define JOINT_ACC_MAX_1   2.94   // 关节最大加速度，单位：弧度/秒*秒//////////////////max motor acc=4500r/min*s
#define JOINT_ACC_MAX_2   4.66
#define JOINT_ACC_MAX_3   4.66
#define JOINT_ACC_MAX_4   3.19
#define JOINT_ACC_MAX_5   7.85
#define JOINT_ACC_MAX_6   7.85

/*
#define JOINT_ACC_RATIO_1   0.5
#define JOINT_ACC_RATIO_2   0.5
#define JOINT_ACC_RATIO_3   0.5
#define JOINT_ACC_RATIO_4   0.5
#define JOINT_ACC_RATIO_5   0.5
#define JOINT_ACC_RATIO_6   0.5
*/
#define JOINT_ACC_RATIO_1   2/////////////1
#define JOINT_ACC_RATIO_2   2
#define JOINT_ACC_RATIO_3   2
#define JOINT_ACC_RATIO_4   2
#define JOINT_ACC_RATIO_5   2
#define JOINT_ACC_RATIO_6   2

#define JOINT_JERK_1      76//7.6 //关节最大加加速度
#define JOINT_JERK_2      120
#define JOINT_JERK_3      120
#define JOINT_JERK_4      82
#define JOINT_JERK_5      202
#define JOINT_JERK_6      202
#define TCP_VELOCITY      1200  //单位mm/s ///1600
#define TCP_ACC           2800
#define TCP_JERK          75000
#define TCP_ACC_RATIO  2//////0.03
#define TCP_ROT_VELOCITY      1.92  //单位 弧度/秒
#define TCP_ROT_ACC           2.94   // 最大加速度，单位：弧度/秒*秒
#define TCP_ROT_JERK          38
#define TCP_ROT_ACC_RATIO  2//////0.03
#define TIME              0.002//插补周期4ms

#define STOP_RATIO		2;


/*====================================================================================*\
* 内容：定义与robot六关节相关参数的结构体

* 作者：彭理仁

* 创建日期：2015-7-21

* 修改日期：2015-7-21
\*====================================================================================*/
struct RobotJointRelate//定义与robot六关节相关参数的结构体,用于高度抽象与关节相关5个参数集
{
	double RJR_a;
	double RJR_b;
	double RJR_c;
	double RJR_d;
	double RJR_e;
	double RJR_f;
};
struct JointPoint //关节值表示的示教路径点,单位°
{
	double joint[6];
};
struct JointTorq //关节值表示的示教路径点,单位°
{
	double torq[6];
};
struct RPYPoint
{
	double rpypoint[6];//[px,py,pz,rx,ry,rz]
};
struct TransMatrix//局部坐标与基座标系之间的转换矩阵
{
	double m[4][4];
};

class BaseRobot
{
public:
	BaseRobot(void);
	~BaseRobot(void);


	/**
	* 函数名： forwardkine

	* 输入：Input

	* 输出：Output

	* 作者：徐凡

	* 创建日期：2015-7-1

	* 修改日期：2015-7-20（移动至此，移动者：彭理仁）
	**/
public://自定义成员函数
	//void forwardkine(double J[], double point[], BaseRobot &newrobot);//正解
	void InitializeStruct(int n,RobotJointRelate *robotjointrelate);//声明结构体初始化函数
	//void inverse_kine(double pos16[],double angle[],BaseRobot &newrobot1); //逆解
	//void ThreePointArc(Pnt3 PointStart,Pnt3 PointMid,Pnt3 PointEnd,Pnt3 *Point);//三点圆弧
	void RotateTransfor(double Pos[4][4],double theta,int Axis);//用户坐标系下绕单轴旋转
	void TranslateOrRotateTransfor(double Pos[4][4],double theta,double a,int Axis);//特定坐标系下单轴相对平移或旋转
	void RPY2RotMatrix(double RX,double RY,double RZ,double PX,double PY,double PZ,double RotateMatrix[16]);//将RPY转换成齐次矩阵
	void RotMatrix2RPY(double RotateMatrix[],double rpy[]);//将齐次矩阵站换成RPY角度
	void RPY2RotMatrix(RPYPoint p,TransMatrix &p_matrix);//将RPY转换成齐次矩阵
	void RotMatrix2RPY(TransMatrix p_matrix,RPYPoint &p);//将齐次矩阵站换成RPY角度
	//void Invers_Kine(double pos[][4],double angle_solution[][6],int &Num);
	int SelectJoint(double pJoint[],double TempJoint[][6],double Opti_Joint[],int InversNum);//对所求的逆解进行选择
	void transl(double dx,double dy,double dz,double p_transl[][4]);//平移矩阵
    void ServeMotor2Position(double serve_motor[],double position[]);//将电机编码值转换成空间坐标点位置
	void Quater(double posstart[16], double posend[16], double quart[4]);
	void Quater2Matrix(double q[4],double ResultMatrix[][4]);//将四元素转换到齐次矩阵
	//void Invers_Matrix(double Matrix1[][4], double Result_Matrix[][4]);//针对4x4的矩阵求逆

	//基于四元素插补姿态直线规划，修改时间2016年5月30日
	int JudgePointPositionPose(double point[4][4]);

/*
	//基于四元素插补姿态，速度规划的圆弧函数,输入P[16]
	void MoveCircleSpeed_Radius(double P1[],double P2[],double P3[],double Result[]);
	void MoveCircleSpeedQuater(double P1[],double P2[],double P3[],double Property[],double *pJoint[],double *InternPoint[]);
	void PreMoveCircleSpeedQuater(double P1[],double P2[],double P3[],double V_Max,double Property[]);
*/

	void Matrix_Mul(double a[][4], double b[][4], double rst[][4]);
	void Matrix_Mul(double a[][4],double b[][1],double c[][1]);
	void Inv_MATRIX(double b[4][4], double c[4][4]);
	void quartion(double posstart[16], double posend[16], double quart[4]);
	void q2transf(double q[4],double tsf[16]);

public://自定义成员变量
	RobotJointRelate DH_joint;//定义六关节参数的结构体变量
	RobotJointRelate jointrangeU,jointrangeD;//定义robot六关节上下范围的结构体变量（U为上限，D为下限）
	RobotJointRelate DH_o,DH_p,DH_q;//定义D-H参数的结构体变量
	RobotJointRelate JtoPPU;//定义关节到PPU比例的结构体变量
	RobotJointRelate RPPUtoWPPU;//定义读取PPU到发送PPU比例的结构体变量

// Filie IO
public:
	//void ClearInfo();
	//void SetJointRotAxis();
	//void InitialJointRotAngle();

   /**********************************

	* 创建日期：2015-7-1

	* 搬迁日期：2016-2-6 14:02:55

	* 搬迁人：党渊渊

	************************************/

	void forwardkine_tong(double J[],double point_16[], double point_44[][4]); //通用的正解
	void inverse_kine_tong(double pos[][4], double angle_solution[][6], int &num);//改版后的求逆解（完全按照新松Kg机器人建立的）
	void nijiequyou(double J[], double J1[][4][6], double J2[][6]); //逆解取优判断

	/***************************************************************************************

	* 函数名：矩阵相乘、求逆、转置、相叠、相加、相减

	* 输入：Input

	* 输出：Output

	* 作者：党渊渊

	* 创建日期：2015-10-8

	* 修改日期：2015-10-8
	* 搬迁日期：2015-12-1 20:16:50
	* 搬迁日期：2016-2-6 14:05:30
	****************************************************************************************/
	void Juzhenxiangcheng (double *Trans1,double *Trans2,double *Trans3_out,int m,int n,int k);
	void Juzhenzhuanzhi(double *Trans1,double *Trans2_out,int m,int n);
	void Juzhenxiangdie(double *Trans1,double *Trans2,double *Trans3,double *Trans4_out,int m,int n,int k);
	void Juzhenxiangdie(double *Trans1,double *Trans2,double *Trans4_out,int m,int n, int k);
	void Juzhenxiangjia(double *Trans1, double *Trans2,double *Trans3_out, int m, int n);
	void Juzhenxiangjian(double *Trans1, double *Trans2,double *Trans3_out, int m, int n);
	void Lgbhjz(double Rot_z[4][4], double Trans[4][4], double Rot_x[4][4], double A[4][4]);  //求取连杆变换矩阵
	void Juzhenqiuni (double *Trans1,double *Trans2_out, int n);

	int SelectJointRPY(double pJoint[],double TempJoint[][6],double Opti_Joint[],int InversNum);

	//////////////////////////////////////2017-4-21
	void Cross(double a[3],double b[3],double rst[3]);
	double VectorLen(double vec[]);
	////三点法建立齐次变换矩阵，已测试通过
	int BuildTransMatrix(TransMatrix p1,TransMatrix p2,TransMatrix p3,TransMatrix &trans_matrix);///p1在x轴，p2 x-y任意一点，p3为局部坐标原点
	int BuildTransMatrix(RPYPoint p1,RPYPoint p2,RPYPoint p3,TransMatrix &trans_matrix);///p1在x轴，p2 x-y任意一点，p3为局部坐标原点
	int BuildTransMatrix(JointPoint j1,JointPoint j2,JointPoint j3,TransMatrix &trans_matrix);///p1在x轴，p2 x-y任意一点，p3为局部坐标原点
	int PointHmgTrans(RPYPoint p0,RPYPoint &p1,TransMatrix trans,TransMatrix tool_trans);///齐次变换
	int PointHmgTrans(TransMatrix p0,TransMatrix &p1,TransMatrix trans,TransMatrix tool_trans);///齐次变换
	/////建立工具坐标变换矩阵，测试未完成
	int CreatTool (vector<JointPoint> Peo,double Tt_b[][4],double Ptcp_e[],TransMatrix tool_trans);

	///////////////////////////////////////////////////2017-5-25 xujinrong
	void ResetJointLimit(JointPoint joint_min,JointPoint joint_max);

	///////////////////////////2017-6-8
	void GravityCompst(JointPoint joint_pos,JointTorq &joint_tq);
};

