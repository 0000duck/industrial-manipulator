/*
 * SerialLink.cpp
 *
 *  Created on: Aug 15, 2017
 *      Author: a1994846931931
 */

# include "SerialLink.h"
# include "../kinematics/Frame.h"
# include "../common/printAdvance.h"
# include "Jacobian.h"

using robot::kinematic::Frame;
using std::vector;

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

int SerialLink::getDOF() const
{
	return (int)_linkList.size();
}


DHTable SerialLink::getDHTable() const
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

/*
 * 根据关节角度获取雅克比矩阵；
 */
Jacobian SerialLink::getJacobian() const
{
	return getJacobian(getQ());
}

Jacobian SerialLink::getJacobian(const robot::math::Q& q) const
{
	int dof = getDOF();
	dof = 6; // 目前只处理6关节的雅克比矩阵
	vector< vector<double> > j(6, vector<double>(dof)); // 只处理6X6的雅克比矩阵 // TODO static?

	// 计算每个关节相对上一坐标系的变换矩阵
	std::vector< HTransform3D<double> > Ti_1i; // i=1~n
	for (int i=0; i<dof; i++)
	{
		Ti_1i.push_back(this->getTransform(i, i + 1, q));
	}

	// 计算每个关节变换矩阵的导
	std::vector< Rotation3D<double> > dTi_1i; // i=1~n
	for (int i=0; i<dof; i++)
	{
		Link* link = _linkList[i];
		dTi_1i.push_back(Rotation3D<double>::dDH(link->alpha(), link->a(), link->d(), link->theta() + q[i]));
	}

	// 计算每个关节相对于0坐标系的矩阵变换
	std::vector< Rotation3D<double> > R0i; // i=1~n
	R0i.push_back(Ti_1i[0].getRotation());
	for (int i=1; i<dof; i++)
	{
		R0i.push_back(R0i[i-1]*(Ti_1i[i].getRotation()));
	}

	// 计算工具末端相对于哥哥关节坐标的坐标值
	Vector3D<double> nPend = _endToTool->getTransform().getPosition();
	std::vector< Vector3D<double> > iPend; // i=n~1 注意为逆向储存
	iPend.push_back(nPend);
	for (int i=dof; i>1; i--)
	{
		iPend.push_back( Ti_1i[i-1]*iPend[dof-i] );
	}

	// Jv速度雅克比
	Vector3D<double> temp = dTi_1i[0]*iPend[dof-1];
	j[0][0] = temp(0);
	j[1][0] = temp(1);
	j[2][0] = temp(2);

	for (int i=1; i<dof; i++)
	{
		temp = R0i[i-1]*dTi_1i[i]*iPend[dof-i-1];
		j[0][i] = temp(0);
		j[1][i] = temp(1);
		j[2][i] = temp(2);
	}

	// Jw角速度雅克比
	for (int i=0; i<dof; i++)
	{
		j[3][i] = R0i[i](0, 2);
		j[4][i] = R0i[i](1, 2);
		j[5][i] = R0i[i](2, 2);
	}

	return Jacobian(j); // 返回6X6的雅克比矩阵
}


const robot::math::Q SerialLink::getQ() const
{
	robot::math::Q q = robot::math::Q::zero(getDOF());
	for (int i=0; i<getDOF(); i++)
	{
		q(i) = _linkList[i]->getQ();
	}
	return q;
}

void SerialLink::setQ(robot::math::Q q)
{
	int i=0;
	for (std::vector<Link*>::iterator it=_linkList.begin(); it<_linkList.end(); it++)
		(*it)->change(q[i++]);
}

Config SerialLink::getConfig(const robot::math::Q& q) const
{
	DHTable dHTable = getDHTable();
	double j2 = q[1] + dHTable[1].theta();
	double j3 = q[2] + dHTable[2].theta();
	double j5 = q[4] + dHTable[4].theta();
	double a2 = dHTable[1].a();
	double a3 = dHTable[2].a();
	double a4 = dHTable[3].a();
	double d4 = dHTable[3].d();
	double config_r = a2 + a3*cos(j2) + a4*cos(j2 + j3) - d4*sin(j2 + j3);
	double wrist = (j5 >= 0)? Config::wpositive:Config::wnegative;
	double elbow = (j3 >= 0)? Config::epositive:Config::enegative;
	double shoulder =(config_r < 0)? Config::righty:Config::lefty;
	return Config(shoulder, elbow, wrist);
}

const robot::math::Q SerialLink::getEndVelocity(const robot::kinematic::State& state) const
{
	Jacobian jacob = getJacobian(state.getAngle());
	return jacob*(state.getVelocity());
}

SerialLink::~SerialLink()
{
}

} /* namespace model */
} /* namespace robot */
