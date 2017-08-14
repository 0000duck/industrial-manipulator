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

Frame::Frame() {
	// TODO Auto-generated constructor stub
	_parent = NULL;
}

Frame::Frame(Frame* parent){
	_parent = parent;
	if (parent != NULL)
		parent->addChild(this);
}

void Frame::addChild(Frame* child){
	_children.push_back(child);
}

Frame::~Frame() {
	// TODO Auto-generated destructor stub
}

} /* namespace kinematic */
} /* namespace robot */
