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

Frame::~Frame()
{
	// TODO Auto-generated destructor stub
}

} /* namespace kinematic */
} /* namespace robot */
