/*
 * motionstacktest.cpp
 *
 *  Created on: Oct 18, 2017
 *      Author: a1994846931931
 */

# include "../../simulation/MotionStack.h"
# include "../../simulation/TaskStack.h"
# include "../../parse/RobotXMLParser.h"
# include "../../ik/SiasunSR4CSolver.h"
# include "../../common/common.h"
# include "../../common/fileAdvance.h"
# include "../../pathplanner/JoggingPlanner.h"
# include <functional>
# include <sys/time.h>
# include <stdlib.h>
# include <stdio.h>
# include <unistd.h>
# include <thread>

using std::cout;
using std::vector;
using robot::simulation::MotionStack;
using robot::simulation::TaskStack;
using robot::ik::SiasunSR4CSolver;
using namespace robot::pathplanner;
using namespace robot::common;

typedef enum{
	StatusNormal,
	StatusPause,
	StatusStop
} Status;

/**> 读取模型文件 */
robot::parse::RobotXMLParser modelParser;
SerialLink::ptr robotModel = modelParser.parse("src/example/modelData/siasun6.xml");

/**> 设置工具(可选) */
HTransform3D<double> tran = HTransform3D<double>(Vector3D<double>(0, 0, 0.2), Rotation3D<double>());
Frame tool = Frame(tran);
//	robot.setTool(&tool);
//	solver->init();

/**> 逆解器 */
std::shared_ptr<SiasunSR4CSolver> solver(new SiasunSR4CSolver(robotModel));

/**> 参数设定 */
double vMax = 1.0;
double aMax = 20.0;
double h = 50;

Q dqLim = Q(3, 3, 3, 3, 5, 5);
Q ddqLim = Q(20, 20, 20, 20, 20, 20);

/** Jogging 规划器 */
JoggingPlanner jogPlanner = JoggingPlanner(solver, vector<double>{0.5, 10, 50, 1, 10, 100}, dqLim, ddqLim);

/** 堆栈锁 */
std::mutex motionStackMutex;

vector<double> vt;
vector<Q> vxpath;
vector<Q> vdxpath;
vector<Q> vddxpath;
const unsigned long long t0 = getUTime();
void record(State &state) {Q x = state.getAngle();Q dx = state.getVelocity(); Q ddx = state.getAcceleration();vxpath.push_back(x);vdxpath.push_back(dx);vddxpath.push_back(ddx);};


bool addJog(MotionStack *motionStack, const Q& start, Q& farEnd, const std::string mode)
{
	Planner::ptr planner;
	if (mode.compare("x") == 0)
		planner = jogPlanner.jogX(start, farEnd, true); //默认为底座坐标系
	else if (mode.compare("-x") == 0)
		planner = jogPlanner.jogX(start, farEnd, false); //默认为底座坐标系
	else if (mode.compare("y") == 0)
		planner = jogPlanner.jogY(start, farEnd, true); //默认为底座坐标系
	else if (mode.compare("-y") == 0)
		planner = jogPlanner.jogY(start, farEnd, false); //默认为底座坐标系
	else if (mode.compare("z") == 0)
		planner = jogPlanner.jogZ(start, farEnd, true); //默认为底座坐标系
	else if (mode.compare("-z") == 0)
		planner = jogPlanner.jogZ(start, farEnd, false); //默认为底座坐标系
	else
	{
		cout << "尚未识别的jog模式!";
		return false;
	}
	if (start == farEnd)
	{
		cout << "无法再运动, 到达边界\n";
		return false;
	}
	int result = motionStack->addPlanner(planner);
	if (result == 0)
	{
		return true;
	}
	else
	{
		cout << "添加Jog失败, 错误代码: " << result << endl;
		return false;
	}

}

void move(MotionStack *motionStack, int *status, State *state)
{
	cout << "- 运动开始\n";
	unsigned long long next = getUTime();
	const unsigned int dt = 4000;
	while(*status == StatusNormal || *status == StatusPause)
	{
		double timeToNext = (double(next - getUTime()))/1000000;
		if (timeToNext < 0)
			continue;
		usleep(timeToNext*1000000);
		unsigned long long t = getUTime();
		int result = motionStack->state(t, *state);
		if (result == 0 || result == 1)
		{
//			cout << "下发命令\n";
			record(*state); //下发指令
			vt.push_back((double(t - t0))/1000000);
			if (result == 1) //任务完成
			{
				*status = StatusStop;
				cout << "- 运动结束\n";
				break;
			}
		}
		else
		{
			cout << "错误\n";
		}
		next += dt;
	}
}

void motionstacktest()
{

	Q start = Q::zero(6);
	Q intermediate = Q(0.7, 0, 0, 0, -0.7, 0);
	Q end =  Q(1.5, 0, 0, 0, -1.5, 0);
	vector<Q> qPath = {intermediate, end};
	vector<double> arcRatio = {0.2};
	vector<double> velocity = {vMax, vMax};
	vector<double> acceleration = {aMax, aMax};
	vector<double> jerk = {h, h};

	auto motionStack = std::make_shared<MotionStack>(start, &motionStackMutex);
	State state;
	int status = StatusStop;

	Q temp;
	addJog(motionStack.get(), start, temp, "-x"); //逆x轴直线示教到最远端
	TaskStack taskStack(motionStack, temp, dqLim, ddqLim, vMax, aMax, h, solver); //以最远端为初始点建立一个任务栈
	taskStack.addLine(start, 1.0, 1.0); //添加直线, 运动回初始位置
	taskStack.addMLAB(qPath, arcRatio, velocity, acceleration, jerk); //添加连续轨迹

	/**
	 * 以下为运动堆栈发送测试代码;
	 * 对于堆栈中的所有轨迹, 都尝试运行一段时间后暂停一秒, 若此时无法暂停(说明路径要么已经走完, 要么离结束已近很近了), 则继续运行直到结束后再暂停一秒.
	 */
	while (motionStack->getStatus() != 0) //不为空
	{
		if (motionStack->start() == 0)
			status = StatusNormal;
		else
		{
			cout << "启动不成功\n";
			return;
		}
		std::thread normal_t(move, motionStack.get(), &status, &state);
		normal_t.detach();
		usleep(900000); //几秒后暂停
		cout << "尝试暂停\n";
		int result = motionStack->pause();
		if (result != 0)
			cout << "暂停不成功, 结果: " << result << endl;
		else
		{
			cout << "暂停成功\n";
			status = StatusPause;
		}
		try{
			while(status != StatusStop)
			{
				usleep(4000);
			}
			cout << "暂停结束, 等待一秒\n";
			sleep(1);
			cout << "恢复路径\n";
			int result = motionStack->resume(*(vxpath.end() - 1));
			if (result != 0)
				cout << "恢复不成功, 结果: " << result << "\n";
			if (result == 0)
			{
				int result = motionStack->start();
				if (result == 0)
				{
					cout << "再启动成功\n";
					status = StatusNormal;
					std::thread normal_t(move, motionStack.get(), &status, &state);
					normal_t.detach();
				}
				else
				{
					cout << "再启动失败, 清空堆栈\n";
					motionStack->clear();
					status = StatusStop;
				}
			}
			while(status != StatusStop)
			{
				usleep(4000);
			}
			cout << "完成一次运动\n\n";
		}
		catch(char const* msg)
		{
			cout << msg << endl;
		}
	}
	saveQPath("src/example/motionstack/tempx.csv", vxpath, vt);
	saveQPath("src/example/motionstack/tempdx.csv", vdxpath, vt);
	saveQPath("src/example/motionstack/tempddx.csv", vddxpath, vt);
}


