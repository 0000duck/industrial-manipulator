/*
 * MoveL.h
 *
 *  Created on: Aug 21, 2017
 *      Author: zrf
 */

#ifndef MOVEL_H_
#define MOVEL_H_
namespace robot{
class MoveL {
public:
	MoveL();
	MoveL(double x1,double y1,double z1,double rx1,double ry1,double rz1,
			double x2,double y2,double z2,double rx2,double ry2,double rz2,int v);
	void coe(double t);
	void interpolation();
	virtual ~MoveL();
private:
	double _x1;double _y1;double _z1;
	double _rx1;double _ry1;double _rz1;
	double _x2;double _y2;double _z2;
	double _rx2;double _ry2;double _rz2;

	int _v;
};
}

#endif /* MOVEL_H_ */
