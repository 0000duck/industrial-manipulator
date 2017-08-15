/*
 * Frame.h
 *
 *  Created on: Aug 11, 2017
 *      Author: a1994846931931
 */

#ifndef FRAME_H_
#define FRAME_H_

# include <vector>
# include "stddef.h"
# include "../math/HTransform3D.h"

using namespace robot::math;

namespace robot {
namespace kinematic {

class Frame {
	typedef std::vector<Frame*> FrameList;
public:
	Frame(HTransform3D<double>*);
	Frame(Frame* parent, HTransform3D<double>*);
	void addChild(Frame* child);
	void setParent(Frame* parent);
	void setTransform(HTransform3D<double>*);
	void updateTransform(const double&, const double&, const double&, const double&,
			const double&, const double&, const double&, const double&,
			const double&, const double&, const double&, const double&);
	Frame* getParent();
	const HTransform3D<double>* getTransform() const;
	void print();
	virtual ~Frame();
private:
	static int _frameIDCounter;
	int _frameID;
	HTransform3D<double>* _tran;
	Frame* _parent;
	FrameList _children;
};

} /* namespace kinematic */
} /* namespace robot */

#endif /* FRAME_H_ */
