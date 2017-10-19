/*
 * motionstacktest.cpp
 *
 *  Created on: Oct 18, 2017
 *      Author: a1994846931931
 */

# include "../../simulation/MotionStack.h"
# include "../../parse/RobotXMLParser.h"
# include "../../ik/SiasunSR4CSolver.h"
# include "../../pathplanner/LinePlanner.h"
# include "../../pathplanner/CircularPlanner.h"
# include "../../pathplanner/MultiLineArcBlendPlanner.h"
# include "../../common/common.h"
# include "../../common/fileAdvance.h"
# include <functional>
# include <sys/time.h>
# include <stdlib.h>
# include <stdio.h>
# include <unistd.h>
# include <thread>

using std::cout;
using std::vector;
using robot::simulation::MotionStack;
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

double vMax = 1.0;
double aMax = 20.0;
double h = 50;

Q dqLim = Q(3, 3, 3, 3, 5, 5);
Q ddqLim = Q(20, 20, 20, 20, 20, 20);

vector<double> vt;
vector<Q> vxpath;
vector<Q> vdxpath;
vector<Q> vddxpath;
const unsigned long long t0 = getUTime();
void record(State &state) {Q x = state.getAngle();Q dx = state.getVelocity(); Q ddx = state.getAcceleration();vxpath.push_back(x);vdxpath.push_back(dx);vddxpath.push_back(ddx);};

bool addLine(MotionStack* motionStack, Q start, Q end, double vRatio, double aRatio)
{
	try{
		LinePlanner::ptr planner( new LinePlanner(dqLim, ddqLim, vMax*vRatio, aMax*aRatio, h, solver, robotModel, end));
		planner->query(start);
		int result = motionStack->addPlanner(planner);
		if (result == 0)
		{
			return true;
		}
		else
		{
			cout << "添加直线planner失败, 错误代码: " << result << endl;
			return false;
		}
	}
	catch(char const* msg)
	{
		cout << msg << endl;
	}
	catch(std::string &msg)
	{
		cout << msg << endl;
	}
	return false;
}

bool addCircle(MotionStack* motionStack, Q start, Q intermediate, Q end, double vRatio, double aRatio)
{
	try{
		CircularPlanner::ptr planner( new CircularPlanner(dqLim, ddqLim, vMax*vRatio, aMax*aRatio, h, solver, robotModel, intermediate, end));
		planner->query(start);
		int result = motionStack->addPlanner(planner);
		if (result == 0)
		{
			return true;
		}
		else
		{
			cout << "添加圆弧planner失败, 错误代码: " << result << endl;
			return false;
		}
	}
	catch(char const* msg)
	{
		cout << msg << endl;
	}
	catch(std::string &msg)
	{
		cout << msg << endl;
	}
	return false;
}

bool addMLAB(MotionStack* motionStack, vector<Q> qPath, vector<double> arcRatio, vector<double> velocity, vector<double> acceleration, vector<double> jerk)
{
	try{
		MultiLineArcBlendPlanner::ptr planner( new MultiLineArcBlendPlanner(dqLim, ddqLim, solver, robotModel, qPath, arcRatio, velocity, acceleration, jerk));
		planner->query();
		int result = motionStack->addPlanner(planner);
		if (result == 0)
		{
			return true;
		}
		else
		{
			cout << "添加直线圆弧planner失败, 错误代码: " << result << endl;
			return false;
		}
	}
	catch(char const* msg)
	{
		cout << msg << endl;
	}
	catch(std::string &msg)
	{
		cout << msg << endl;
	}
	return false;
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
	vector<Q> qPath = {start, intermediate, end};
	vector<double> arcRatio = {0.5};
	vector<double> velocity = {vMax, vMax};
	vector<double> acceleration = {aMax, aMax};
	vector<double> jerk = {h, h};

	MotionStack motionStack(start);
	State state;
	int status = StatusStop;

	addLine(&motionStack, start, end, 1.0, 1.0);
	addCircle(&motionStack, end, intermediate, start, 1.0, 1.0);
	addMLAB(&motionStack, qPath, arcRatio, velocity, acceleration, jerk);

	while (motionStack.getStatus() != 0) //不为空
	{
		if (motionStack.start() == 0)
			status = StatusNormal;
		else
		{
			cout << "启动不成功\n";
			return;
		}
		std::thread normal_t(move, &motionStack, &status, &state);
		normal_t.detach();
		usleep(900000); //几秒后暂停
		cout << "尝试暂停\n";
		int result = motionStack.pause();
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
			int result = motionStack.resume(*(vxpath.end() - 1));
			if (result != 0)
				cout << "恢复不成功, 结果: " << result << "\n";
			if (result == 0)
			{
				int result = motionStack.start();
				if (result == 0)
				{
					cout << "再启动成功\n";
					status = StatusNormal;
					std::thread normal_t(move, &motionStack, &status, &state);
					normal_t.detach();
				}
				else
				{
					cout << "再启动失败, 清空堆栈\n";
					motionStack.clear();
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


