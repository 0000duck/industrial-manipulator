/*
 * TaskStack.cpp
 *
 *  Created on: Oct 20, 2017
 *      Author: a1994846931931
 */

#include "TaskStack.h"
# include "../pathplanner/LinePlanner.h"
# include "../pathplanner/CircularPlanner.h"
# include "../pathplanner/MultiLineArcBlendPlanner.h"
# include <thread>

using std::vector;
using namespace robot::pathplanner;


namespace robot {
namespace simulation {

TaskStack::TaskStack(MotionStack::ptr motionStack, Q start, Q dqLim, Q ddqLim, double vMax, double aMax, double h, std::shared_ptr<IKSolver> solver)
: _motionStack(motionStack),
  _msmtx(motionStack->getMutex()),
  _start(start),
  _dqLim(dqLim),
  _ddqLim(ddqLim),
  _velocity(vMax),
  _acceleration(aMax),
  _jerk (h)
{
	_mode = 0;
	_solver = solver;
	_error = false;
}

void TaskStack::reset(Q start)
{
	_start = start;
	_taskMutex.unlock();
	_queryMutex.unlock();
	_task = std::queue<Planner::ptr>();
	_error = false;
}

bool TaskStack::addLine(Q end, double vRatio, double aRatio)
{
	if (_error)
		return false;
	if(_mode == 0) //带有检查的阻塞模式
	{
		try{
			LinePlanner::ptr planner( new LinePlanner(_dqLim, _ddqLim, _velocity*vRatio, _acceleration*aRatio, _jerk, _solver,_start, end));
			planner->query();
			_msmtx->lock();
			int result = _motionStack->addPlanner(planner);
			_msmtx->unlock();
			if (result == 0)
			{
				_start = end;
				return true;
			}
			else
			{
				cout << "添加直线planner失败, 错误代码: " << result << endl;
				_error = true;
				return false;
			}
		}
		catch(char const* msg) { cout << msg << endl;}
		catch(std::string &msg) {cout << msg << endl;}
		_error = true;
		return false;
	}
	else //速度优先的非阻塞模式
	{
		LinePlanner::ptr planner;
		try{
			planner = LinePlanner::ptr( new LinePlanner(_dqLim, _ddqLim, _velocity*vRatio, _acceleration*aRatio, _jerk, _solver, _start, end));
			_start = end;
		}
		catch(char const* msg) { cout << msg << endl; _error = true; return false;}
		catch(std::string &msg) {cout << msg << endl; _error = true; return false;}
		_taskMutex.lock();
		_task.push(planner);
		_taskMutex.unlock();
		if (_queryMutex.try_lock())
		{
			std::thread t(std::mem_fn(&TaskStack::doQuery), this);
			t.detach();
		}
		return true;
	}
}

bool TaskStack::addCircle(Q intermediate, Q end, double vRatio, double aRatio)
{
	if (_error)
		return false;
	if(_mode == 0) //带有检查的阻塞模式
	{
		try{
			CircularPlanner::ptr planner( new CircularPlanner(_dqLim, _ddqLim, _velocity*vRatio, _acceleration*aRatio, _jerk, _solver, intermediate, _start, end));
			planner->query();
			_msmtx->lock();
			int result = _motionStack->addPlanner(planner);
			_msmtx->unlock();
			if (result == 0)
			{
				_start = end;
				return true;
			}
			else
			{
				cout << "添加圆弧planner失败, 错误代码: " << result << endl;
				_error = true;
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
	else //速度优先的非阻塞模式
	{
		CircularPlanner::ptr planner;
		try{
			planner = CircularPlanner::ptr( new CircularPlanner(_dqLim, _ddqLim, _velocity*vRatio, _acceleration*aRatio, _jerk, _solver, intermediate, _start, end));
			_start = end;
		}
		catch(char const* msg) { cout << msg << endl; _error = true; return false;}
		catch(std::string &msg) {cout << msg << endl; _error = true; return false;}
		_taskMutex.lock();
		_task.push(planner);
		_taskMutex.unlock();
		if (_queryMutex.try_lock())
		{
			std::thread t(std::mem_fn(&TaskStack::doQuery), this);
			t.detach();
		}
		return true;
	}
}

bool TaskStack::addMLAB(vector<Q> qPath, vector<double> arcRatio, vector<double> velocity, vector<double> acceleration, vector<double> jerk)
{
	if (_error)
		return false;
	qPath.insert(qPath.begin(), _start); //添加起始点到路径开始
	if(_mode == 0) //带有检查的阻塞模式
	{
		try{
			MultiLineArcBlendPlanner::ptr planner( new MultiLineArcBlendPlanner(_dqLim, _ddqLim, _solver, qPath, arcRatio, velocity, acceleration, jerk));
			planner->query();
			_msmtx->lock();
			int result = _motionStack->addPlanner(planner);
			_msmtx->unlock();
			if (result == 0)
			{
				_start = *(qPath.end() - 1);
				return true;
			}
			else
			{
				cout << "添加直线圆弧planner失败, 错误代码: " << result << endl;
				_error = true;
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
		_error = true;
		return false;
	}
	else //速度优先的非阻塞模式
	{
		MultiLineArcBlendPlanner::ptr planner;
		try{
			planner = MultiLineArcBlendPlanner::ptr( new MultiLineArcBlendPlanner(_dqLim, _ddqLim, _solver, qPath, arcRatio, velocity, acceleration, jerk));
			_start = *(qPath.end() - 1);
		}
		catch(char const* msg) { cout << msg << endl; _error = true; return false;}
		catch(std::string &msg) {cout << msg << endl; _error = true; return false;}
		_taskMutex.lock();
		_task.push(planner);
		_taskMutex.unlock();
		if (_queryMutex.try_lock())
		{
			std::thread t(std::mem_fn(&TaskStack::doQuery), this);
			t.detach();
		}
		return true;
	}
}


TaskStack::~TaskStack() {
	// TODO Auto-generated destructor stub
}

void TaskStack::doQuery()
{
	do{
		if (_task.empty())
		{
			_queryMutex.unlock();
			break;
		}
		Planner::ptr planner = _task.front();
		_task.pop();
		_taskMutex.unlock();
		try{
			planner->doQuery();
			_msmtx->lock();
			int result = _motionStack->addPlanner(planner);
			_msmtx->unlock();
			if (result != 0)
			{
				cout << "堆栈添加路径到堆栈失败, 错误代码: " << result << endl;
				_error = true;
			}
		}
		catch(char const* msg)
		{
			cout << "堆栈添加路径到堆栈失败" << endl;
			cout << msg << endl;
		}
		catch(std::string &msg)
		{
			cout << "堆栈添加路径到堆栈失败" << endl;
			cout << msg << endl;
		}
	}while(true);

}

} /* namespace simulation */
} /* namespace robot */
