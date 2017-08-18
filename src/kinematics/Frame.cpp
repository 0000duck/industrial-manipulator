/*
 * Frame.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: a1994846931931
 */

# include "Frame.h"
# include "stddef.h"
# include <iostream>
# include "../common/printAdvance.h"

using namespace robot::common;

namespace robot {
namespace kinematic {

int Frame::_frameIDCounter = 0;

Frame::Frame()
{
	_parent = NULL;
	_frameID = Frame::_frameIDCounter++;
	static HTransform3D<double> worldFrame;
	_tran = worldFrame;
}

Frame::Frame(HTransform3D<double>& transform)
{
	_parent = NULL;
	_frameID = Frame::_frameIDCounter++;
	_tran = transform;
}

Frame::Frame(Frame* parent, HTransform3D<double>& transform)
{
	_parent = parent;
	_frameID = Frame::_frameIDCounter++;
	_tran = transform;
	if (parent != NULL)
		parent->addChild(this);
}

Frame::Frame(Frame* parent)
{
	_parent = parent;
	_frameID = Frame::_frameIDCounter++;
	_tran = HTransform3D<>::identity();
	if (parent != NULL)
	parent->addChild(this);
}

void Frame::setParent(Frame* parent, bool doAddChild)
{
	/*
	 * 设置parent；
	 * 外部调用模式下，doAddChild=true，此时除了设置parent之外；
	 * 还要给parent添加自身作为child；
	 * 如此保证数据的统一性
	 */
	_parent = parent;
	if (doAddChild)
		_parent->addChild(this, false);

}

void Frame::addChild(Frame* child, bool doSetParent)
{
	/*
	 * 添加一个child；
	 * 外部调用模式下，doSetParent=true，此时不仅要给这个Frame添加child；
	 * 还要给它的child设置parent；
	 * 如此保证数据的统一性；
	 */
	if (not this->haveChild(child))
		_children.push_back(child);
	if (doSetParent)
		child->setParent(this, false);
}

bool Frame::haveChild(Frame* child)
{
	/*
	 * 判断一个Frame的_children中有没有一个child；
	 */
	for (int i=0; i<(int)_children.size(); i++)
		if (_children[i] == child)
			return true;
	return false;
}

int Frame::getChildIndex(Frame* child)
{
	/*
	 * 获取一个child在_children中的位置；
	 * 如果没有这个child，那么返回-1；
	 */
	for (int i=0; i<(int)_children.size(); i++)
			if (_children[i] == child)
				return i;
	return -1;
}

const Frame* Frame::getParent()
{
	return _parent;
}

const std::vector<Frame*>& Frame::getChildren()
{
	return _children;
}

void Frame::removeParent(bool doRemoveChild)
{
	Frame* parent = _parent;
	_parent = NULL;
	if (doRemoveChild)
		parent->removeChild(this, false);
}

void Frame::removeChild(Frame* child, bool doRemoveParent)
{
	int index = this->getChildIndex(child);
	if (index == -1)
		return;
	_children.erase(_children.begin() + index);
}

void Frame::setTransform(HTransform3D<double>& transform)
{
	_tran = transform;
}

void Frame::updateTransform(
		const double& r11, const double& r12, const double& r13, const double& r14,
		const double& r21, const double& r22, const double& r23, const double& r24,
		const double& r31, const double& r32, const double& r33, const double& r34)
{
	_tran.update(r11, r12, r13, r14, r21, r22, r23, r24, r31, r32, r33, r34);
}

const HTransform3D<double>& Frame::getTransform() const
{
	return _tran;
}

void Frame::print()
{
	std::cout << "Frame ID: " << _frameID << '\n';
	std::cout << "Have parent? -> " << (_parent == NULL ? "no":"yes") << '\n';
	std::cout << "Number of children -> " << common::to_string(_children.size()) << '\n';
	std::cout << "transform: " << '\n';
	_tran.print();
}
Frame::~Frame()
{
}

} /* namespace kinematic */
} /* namespace robot */
