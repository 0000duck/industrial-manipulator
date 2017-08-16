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
	Frame();
	Frame(HTransform3D<double>*);
	Frame(Frame* parent, HTransform3D<double>*);
	void addChild(Frame* child, bool doSetParent=true);
	void setParent(Frame* parent, bool doCheckParent=true);
	void removeParent(bool doRemoveChild=true);
	void removeChild(Frame* child, bool doRemoveParent=true);
	const Frame* getParent();
	const FrameList& getChildren();
	bool haveChild(Frame* child);
	int getChildIndex(Frame* child);
	void setTransform(HTransform3D<double>*);
	void updateTransform(const double&, const double&, const double&, const double&,
			const double&, const double&, const double&, const double&,
			const double&, const double&, const double&, const double&);
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
