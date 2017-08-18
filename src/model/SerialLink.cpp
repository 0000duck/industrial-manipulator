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

SerialLink::SerialLink()
{
	Frame worldFrame;
	_worldFrame = &worldFrame;
}

SerialLink::SerialLink(std::vector<Link*> linkList)
{
	Frame worldFrame;
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
	Frame* parent = _linkList[_linkList.size() - 1]->getFrame();
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


DHTable& SerialLink::getDHTable()
{
	static DHTable dHTable;
	for (int i=0;i<(int)_linkList.size();i++)
	{
		dHTable.append(_linkList[i]->getDHParams());
	}
	return dHTable;
}

SerialLink::~SerialLink()
{
}

} /* namespace model */
} /* namespace robot */
