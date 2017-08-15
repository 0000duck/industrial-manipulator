/*
 * Frame.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: a1994846931931
 */

# include "Frame.h"
# include "stddef.h"

namespace robot {
namespace kinematic {

int Frame::_frameIDCounter = 0;

Frame::Frame(HTransform3D<double>* transform=NULL)
{
	// TODO Auto-generated constructor stub
	_parent = NULL;
	_frameID = Frame::_frameIDCounter++;
	_tran = transform;
}

Frame::Frame(Frame* parent, HTransform3D<double>* transform=NULL)
{
	_parent = parent;
	_frameID = Frame::_frameIDCounter++;
	_tran = transform;
	if (parent != NULL)
		parent->addChild(this);
}

void Frame::setParent(Frame* parent)
{
	_parent = parent;
}

void Frame::addChild(Frame* child)
{
	_children.push_back(child);
}

void Frame::setTransform(HTransform3D<double>* transform)
{
	_tran = transform;
}

void Frame::updateTransform(
		const double& r11, const double& r12, const double& r13, const double& r14,
		const double& r21, const double& r22, const double& r23, const double& r24,
		const double& r31, const double& r32, const double& r33, const double& r34)
{
	_tran->update(r11, r12, r13, r14, r21, r22, r23, r24, r31, r32, r33, r34);
}

const HTransform3D<double>* Frame::getTransform() const
{
	return _tran;
}

Frame::~Frame()
{
	// TODO Auto-generated destructor stub
}

} /* namespace kinematic */
} /* namespace robot */
