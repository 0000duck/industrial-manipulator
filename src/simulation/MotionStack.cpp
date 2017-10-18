/*
 * MotionStack.cpp
 *
 *  Created on: Oct 17, 2017
 *      Author: a1994846931931
 */

#include "MotionStack.h"

namespace robot {
namespace simulation {

using robot::common::getUTime;

MotionStack::MotionStack() {
	_id = 0;
	_status = stackEmpty;
	_recordTime = 0;
}

int MotionStack::addPlanner(Planner::ptr planner)
{
	if ( ! planner->isTrajectoryExist())
		return 3;
	if ((int)_motionQueue.size() >= _maxDataCount)
		return 1;
	if ((int)_motionQueue.size() > 0)
	{
		Q end = _motionQueue.back().planner->getQTrajectory()->end();
		Q start = planner->getQTrajectory()->start();
		Q delta = end - start;
		delta.abs();
		if (delta.getMax() > 0.0001)
			return 2;
	}
	{
		double duration = planner->getQTrajectory()->duration();
		double precision = 0.0001;
		if (
				(! planner->getQTrajectory()->dx(0).isZero(precision)) ||
//				(! planner->getQTrajectory()->ddx(0).isZero(precision)) ||
				(! planner->getQTrajectory()->dx(duration).isZero(precision))
//				(! planner->getQTrajectory()->ddx(duration).isZero(precision))
				)
			return 4;
	}
	_motionQueue.push(motionData{_id++, planner, planner->getQTrajectory()->duration()});
	if (_status == stackEmpty)
		_status = stackWait;
	return 0;
}

int MotionStack::start()
{
	switch (_status)
	{
	case stackEmpty://栈空
		return 1;
	case stackWait://栈不空, 等待状态
		break;//可启动状态
	case stackNormal://栈不空, 正常发送状态
		return 2;//非等待状态
	case stackPause://栈不空, 运行暂停路径状态
		return 2;//非等待状态
	case stackStop:
		return 2;//非正常等待状态
	default://未识别的标志号
		throw("内部错误, 运动堆栈启动时找到异常状态号\n");
	}
	_status = stackNormal;
	_recordTime = getUTime();
	return 0;
}

int MotionStack::state(unsigned long long t, State &state)
{
	switch (_status)
	{
	case stackEmpty://栈空
		return 2;//处于非发送状态
	case stackWait://栈不空, 等待状态
		return 2;//处于非发送状态
	case stackNormal://栈不空, 正常发送状态
		break;//正常发送状态
	case stackPause://栈不空, 运行暂停路径状态
	{
		double time = (double(t - _recordTime))/1000000.0; //秒
		if ((_stopIpr->duration()) <= time) //时间超出
		{
			Q end = _stopIpr->end();
			Q zero = Q::zero(end.size());
			state = State(end, zero, zero);
			_status = stackStop;
			return 1;
		}
		state = _stopIpr->getState(time);
		return 0;
	}
	case stackStop://栈不空, 暂停状态
		return 2;//处于非发送状态
	default://未识别的标志号
		throw("内部错误, 运动堆栈启动时找到异常状态号\n");
	}
	double time = (double(t - _recordTime))/1000000.0; //秒
	Interpolator<Q>::ptr qIpr = _motionQueue.front().planner->getQTrajectory();
	if ((qIpr->duration()) <= time) //时间超出
	{
		Q end = qIpr->end();
		Q zero = Q::zero(end.size());
		state = State(end, zero, zero);
		_motionQueue.pop();
		if (_motionQueue.empty())
			_status = stackEmpty;
		else
			_status = stackWait;

		return 1;
	}
	state = qIpr->getState(time);
	return 0;
}

int MotionStack::pause()
{
	switch (_status)
	{
	case stackEmpty://栈空
		return 2; //非正常发送状态
	case stackWait://栈不空, 等待状态
		return 2; //非正常发送状态
	case stackNormal://栈不空, 正常发送状态
		break;
	case stackPause://栈不空, 运行暂停路径状态
		return 2; //非正常发送状态
	case stackStop://栈不空, 暂停状态
		return 2; //非正常发送状态
	default://未识别的标志号
		throw("内部错误, 运动堆栈启动时找到异常状态号\n");
	}
	unsigned long long t = getUTime();
	double time = (double(t - _recordTime))/1000000.0; //秒
	Planner::ptr planner = _motionQueue.front().planner;
	if (planner->stop(time, _stopIpr)) //成功规划暂停路径
	{
		_status = stackPause;
		_recordTime = getUTime();
		return 0;
	}
	else //无法规划暂停路径
	{
		return 1;
	}
}

int MotionStack::resume(Q &current)
{
	switch (_status)
	{
	case stackEmpty://栈空
		return 1; //不是处于暂停停止阶段
	case stackWait://栈不空, 等待状态
		return 1; //不是处于暂停停止阶段
	case stackNormal://栈不空, 正常发送状态
		return 1; //不是处于暂停停止阶段
	case stackPause://栈不空, 运行暂停路径状态
		return 1; //不是处于暂停停止阶段
	case stackStop://栈不空, 暂停状态
		break;
	default://未识别的标志号
		throw("内部错误, 运动堆栈启动时找到异常状态号\n");
	}
	Planner::ptr planner = _motionQueue.front().planner;
	planner->resume(current);
	_status = stackWait;
	return 0;
}

bool MotionStack::clear()
{
	switch (_status)
	{
	case stackEmpty://栈空
		break;
	case stackWait://栈不空, 等待状态
		break;
	case stackNormal://栈不空, 正常发送状态
		return false; //运行过程中无法清空栈
	case stackPause://栈不空, 运行暂停路径状态
		return false; //运行过程中无法清空栈
	case stackStop://栈不空, 暂停状态
		break;
	default://未识别的标志号
		throw("内部错误, 运动堆栈启动时找到异常状态号\n");
	}
	_motionQueue = std::queue<motionData>();
	_status = stackEmpty;
	return true;
}

MotionStack::~MotionStack() {
	// TODO Auto-generated destructor stub
}

} /* namespace simulation */
} /* namespace robot */
