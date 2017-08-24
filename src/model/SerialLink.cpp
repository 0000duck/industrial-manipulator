/*
 * SerialLink.cpp
 *
 *  Created on: Aug 15, 2017
 *      Author: a1994846931931
 */

# include "SerialLink.h"
# include "../kinematics/Frame.h"

using robot::kinematic::Frame;

namespace robot {
namespace model {

SerialLink::SerialLink(Frame* tool)
{
	Frame worldFrame;
	_worldFrame = &worldFrame;
	if (tool == NULL)
	{
		static Frame endTool(_worldFrame);
		_endToTool = &endTool;
	}
	else
		_endToTool = tool;
}

SerialLink::SerialLink(std::vector<Link*> linkList,Frame* tool)
{
	static Frame worldFrame;
	if (tool == NULL)
		{
			static Frame endTool(_worldFrame);
			_endToTool = &endTool;
		}
		else
			_endToTool = tool;
	_worldFrame = &worldFrame;
	Frame* parent = _worldFrame;
	for (int i=0; i<(int)linkList.size(); i++)
	{
		_linkList.push_back(linkList[i]);
		parent->addChild(linkList[i]->getFrame());
		parent = linkList[i]->getFrame();
	}
}

void SerialLink::append(Link* link)
{
	Frame* parent = NULL;
	if (_linkList.size() < 1)
		parent = _worldFrame;
	else
		parent = _linkList[_linkList.size() - 1]->getFrame();
	_linkList.push_back(link);
	parent->addChild(link->getFrame());
}

Link* SerialLink::pop()
{
	Link* link = *_linkList.end();
	_linkList.pop_back();
	link->getFrame()->removeParent();
	return link;
}

int SerialLink::getDOF()
{
	return (int)_linkList.size();
}


DHTable SerialLink::getDHTable()
{
	DHTable dHTable;
	for (int i=0;i<(int)_linkList.size();i++)
	{
		dHTable.append(_linkList[i]->getDHParams());
	}
	return dHTable;
}

HTransform3D<double> SerialLink::getTransform(
		unsigned int startLink, unsigned int endLink, const robot::math::Q& q) const
{
	/*
	 * 获取连个关节之间的变换矩阵；
	 * 例如startLink = 0, endLink = 1，获得第一个关节到世界坐标系的转变；
	 */
	HTransform3D<double> tran = HTransform3D<double>::identity();
	for (unsigned int i=startLink; i<endLink; i++)
	{
		tran *= HTransform3D<double>::DH(
				_linkList[i]->alpha(),
				_linkList[i]->a(),
				_linkList[i]->d(),
				_linkList[i]->theta() + q[i]);
	}
	return tran;
}

HTransform3D<double> SerialLink::getEndTransform() const
{
	HTransform3D<> tran = _endToTool->getTransform();
	for (int i=_linkList.size()-1;i>-1;i--)
	{
		tran = _linkList[i]->getFrame()->getTransform()*tran;
	}
	return tran;
}


HTransform3D<double> SerialLink::getEndTransform(const robot::math::Q& q) const
{
	return this->getTransform(0, _linkList.size(), q);
}

Jacobian SerialLink::getJacobian(const robot::math::Q& q) const
{
	HTransform3D<double> T01 = this->getTransform(0, 1, q);
	HTransform3D<double> T12 = this->getTransform(1, 2, q);
	HTransform3D<double> T23 = this->getTransform(2, 3, q);
	HTransform3D<double> T34 = this->getTransform(3, 4, q);
	HTransform3D<double> T45 = this->getTransform(4, 5, q);
	HTransform3D<double> T56 = this->getTransform(5, 6, q);


}

SerialLink::~SerialLink()
{
}

} /* namespace model */
} /* namespace robot */
