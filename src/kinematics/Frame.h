/*
 * Frame.h
 *
 *  Created on: Aug 11, 2017
 *      Author: a1994846931931
 */

#ifndef FRAME_H_
#define FRAME_H_

# include <vector>

namespace robot {
namespace kinematic {

class Frame {
	typedef std::vector<Frame*> FrameList;
public:
	Frame();
	Frame(Frame* parent);
	void addChild(Frame* child);
	Frame* getParent();
	virtual ~Frame();
private:
	static int _frameID;
	Frame* _parent;
	FrameList _children;
};

} /* namespace kinematic */
} /* namespace robot */

#endif /* FRAME_H_ */
