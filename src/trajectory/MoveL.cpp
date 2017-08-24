/*
 * MoveL.cpp
 *
 *  Created on: Aug 21, 2017
 *      Author: zrf
 *
 *      朱瑞锋
 *      输入量x1,y1,z1,x2,y2,z2,v,a
 */

#include "MoveL.h"
#include <math.h>
#include <iostream>
#include <vector>
using std::vector;
namespace robot {

int n;//插补点数
double L,l,a,diffx,diffy,diffz,diffrx,diffry,diffrz;//直线段长度L，插补线段长度l
vector<double> kp;
vector<double> x;
vector<double> y;
vector<double> z;
vector<double> rx;
vector<double> ry;
vector<double> rz;
MoveL::MoveL() {

_x1=_y1=_z1=_rx1=_ry1=_rz1=_v=0;

}

MoveL::MoveL(double x1,double y1,double z1,double rx1,double ry1,double rz1,
		double x2,double y2,double z2,double rx2,double ry2,double rz2,int v):
				_x1(x1),_y1(y1),_z1(z1),_rx1(rx1),_ry1(ry1),_rz1(rz1),
				_x2(x2),_y2(y2),_z2(z2),_rx2(rx2),_ry2(ry2),_rz2(rz2),_v(v)
{
	diffx=_x2-_x1;diffy=_y2-_y1;diffz=_z2-_z1;
	diffrx=_rx2-_rx1;diffry=_ry2-_ry1;diffrz=_rz2-_rz1;
	L=sqrt(diffx*diffx+diffy*diffy+diffz*diffz);
	n=(int)(L/l);//×××××最后插补丢失一段××××××××××

}

void MoveL::coe(double t)//参数k为归一化因子
{
	double k;
	int i;
	double r1=(_v*_v+a*L)*(_v*_v+a*L)/2/(a*_v*_v*L);
	double Tb=_v*_v/(_v*_v+a*L);
	double Lb=_v*_v/a/L/2;
	for(i=0;i<=n;i++)
	{
		t=i/n;
		if (0<=t<=Tb)
		{
			k=r1*t*t/(4*a*_v*_v*L);
		}
		else if (Tb<t<=1-Tb)
		{
			k=1/2/Tb*t-Lb/2;
		}
		else if(Tb<t<1-Tb)
		{
			k=-r1*t*t/2+r1*t-Lb/2-1/8/Lb;
		}
		kp.push_back(k);
	}

}

void MoveL::interpolation()
{
	int i;
	double xp,yp,zp,rxp,ryp,rzp;
	for(i=0;i<=kp.size();i++)
	{
		xp=_x1+kp[i]*diffx;
		yp=_y1+kp[i]*diffy;
		zp=_z1+kp[i]*diffz;
		rxp=_rx1+kp[i]*diffrx;
		ryp=_ry1+kp[i]*diffry;
		rzp=_rz1+kp[i]*diffrz;

		x.push_back(xp);
		y.push_back(yp);
		z.push_back(zp);
		rx.push_back(rxp);
		ry.push_back(ryp);
		rz.push_back(rzp);
	}
}

MoveL::~MoveL()
	{

	}
}
