/*
 * test.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: a1994846931931
 */

# include "../math/Rotation3D.h"
# include "../math/HTransform3D.h"
# include "../kinematics/Trsf.h"
# include "../kinematics/Frame.h"
# include "../kinematics/State.h"
# include "../common/printAdvance.h"
# include "../model/Link.h"
# include "../ik/SiasunSR4CSolver.h"
# include "../model/Config.h"
# include "../math/Quaternion.h"
# include "../math/Integrator.h"
# include "../trajectory/LinearInterpolator.h"
# include "../trajectory/ConvertedInterpolator.h"
# include "../trajectory/MLABTrajectory.h"
# include "../pathplanner/PointToPointPlanner.h"
# include "../pathplanner/LinePlanner.h"
# include "../pathplanner/QBlend.h"
# include "../pathplanner/CircularPlanner.h"
# include "../pathplanner/MultiLineArcBlendPlanner.h"
# include "../simulation/IterativeSimulator.h"
# include <math.h>
# include <string>
# include <vector>
# include <memory>
# include <fstream>
//# include "test.h"
//# include "testIK.h"
//# include "testIK2.h"
//# include "smoothplanner/smoothPlannerSampler.h"
//# include "p2pplanner/p2pPlannerSampler.h"
//# include "lineplanner/lineplannertest.h"
//# include "qblendtest/qblendtest.h"
//# include "circularplanner/circularplannertest.h"
//# include "simulation/simulationtest.h"
//# include "smplannerex/smplannertest.h"
# include "mlabplanner/mlabplannertest.h"
# include <functional>

using namespace robot::math;
using namespace robot::kinematic;
using std::cout;
using std::vector;
using std::string;
using std::shared_ptr;
using namespace robot::common;
using namespace robot::model;
using namespace robot::ik;
using namespace robot::trajectory;
using namespace robot::pathplanner;
using namespace robot::simulation;
using Eigen::MatrixXd;

class base{
public:
	base(){}
	virtual void print(){}
	virtual ~base(){}
	const string& getType() const{return _type;}
protected:
	std::string _type;
};

Q getQ(){
	Q q = Q::zero(6);
//	println(&q);
	return q;
}
class class1:public base{
public:
	class1(){_type = std::string("class1");}
	virtual void print1()
	{
		cout << "class1 method" << endl;
	}
	void setQ()
	{
		Q q = getQ();
		_q = q;
	}
	void printQ()
	{
		_q.print();
	}
	virtual ~class1(){}
private:
	Q _q;
};
class class2:public base{
public:
	class2(){_type = std::string("class2");}
	virtual void print2()
	{
		cout << "class2 method" << endl;
	}
	void setQ()
	{
		Q q = getQ();
		_q = q;
	}
	void printQ()
	{
		_q.print();
	}
	virtual ~class2(){}
private:
	Q _q;
};

/** 以函数作为传参 */
typedef Vector3D<double>(*positionFunction)(double);
void printPosition(positionFunction posFun, double t)
{
	posFun(t).print();
}

int main(){
	println("*** test ***");
	SerialLink robot;

//	double alpha1 = M_PI/2;
//	double alpha2 = 0;
//	double alpha3 = M_PI/2;
//	double alpha4 = -M_PI/2;
//	double alpha5 = M_PI/2;
//	double alpha6 = 0;
//	double a1 = 40;
//	double a2 = 315;
//	double a3 = 70;
//	double a4 = 0;
//	double a5 = 0;
//	double a6 = 0;
//	double d1 = 330;
//	double d2 = 0;
//	double d3 = 310;
//	double d4 = 0;
//	double d5 = 0;
////	double d6 = 70;
//	double d6 = 0;
//	double theta1 = 0;
//	double theta2 = M_PI/2;
//	double theta3 = 0;
//	double theta4 = 0;
//	double theta5 = -M_PI;
//	double theta6 = M_PI/2;

//	// *** My virtual robot
//	double alpha1 = 0;
//	double alpha2 = -M_PI/2;
//	double alpha3 = 0;
//	double alpha4 = M_PI/2;
//	double alpha5 = M_PI/2;
//	double alpha6 = -M_PI/2;
//	double a1 = 0;
//	double a2 = 0;
//	double a3 = 0.3;
//	double a4 = 0;
//	double a5 = 0;
//	double a6 = 0;
//	double d1 = 0.1;
//	double d2 = 0;
//	double d3 = 0;
//	double d4 = 0.2;
//	double d5 = 0;
//	double d6 = 0;
//	double theta1 = M_PI/2;
//	double theta2 = -M_PI/2;
//	double theta3 = M_PI/2;
//	double theta4 = M_PI/2;
//	double theta5 = 0;
//	double theta6 = 0;
//	// ***
//	double lmin1 = M_PI/180.0*-180;
//	double lmin2 = M_PI/180.0*-130;
//	double lmin3 = M_PI/180.0*-70;
//	double lmin4 = M_PI/180.0*-240;
//	double lmin5 = M_PI/180.0*-30;
//	double lmin6 = M_PI/180.0*-360;
//	double lmax1 = M_PI/180.0*180;
//	double lmax2 = M_PI/180.0*80;
//	double lmax3 = M_PI/180.0*160;
//	double lmax4 = M_PI/180.0*240;
//	double lmax5 = M_PI/180.0*200;
//	double lmax6 = M_PI/180.0*360;

	// *** siasun 6kg
	double alpha1 = 0;
	double alpha2 = -M_PI/2;
	double alpha3 = 0;
	double alpha4 = -M_PI/2;
	double alpha5 = M_PI/2;
	double alpha6 = -M_PI/2;
	double a1 = 0;
	double a2 = 0.16;
	double a3 = 0.575;
	double a4 = 0.13;
	double a5 = 0;
	double a6 = 0;
	double d1 = 0.439;
	double d2 = 0;
	double d3 = 0;
	double d4 = 0.644;
	double d5 = 0;
	double d6 = 0.1095;
	double theta1 = 0;
	double theta2 = -M_PI/2;
	double theta3 = 0;
	double theta4 = 0;
	double theta5 = M_PI/2;
	double theta6 = 0;
	// ***
	double lmin1 = M_PI/180.0*-180; //-180
	double lmin2 = M_PI/180.0*-130; //-210
	double lmin3 = M_PI/180.0*-70; //-70
	double lmin4 = M_PI/180.0*-240; //-240
	double lmin5 = M_PI/180.0*-200; //-110
	double lmin6 = M_PI/180.0*-360; //-360
	double lmax1 = M_PI/180.0*180; //180
	double lmax2 = M_PI/180.0*80; //-10
	double lmax3 = M_PI/180.0*160; //160
	double lmax4 = M_PI/180.0*240; //240
	double lmax5 = M_PI/180.0*30; //120
	double lmax6 = M_PI/180.0*360; //360

//	Link(alpha, a, d, theta, min, max, sigma=0)
	Link l1(alpha1, a1, d1, theta1, lmin1, lmax1);
	Link l2(alpha2, a2, d2, theta2, lmin2, lmax2);
	Link l3(alpha3, a3, d3, theta3, lmin3, lmax3);
	Link l4(alpha4, a4, d4, theta4, lmin4, lmax4);
	Link l5(alpha5, a5, d5, theta5, lmin5, lmax5);
	Link l6(alpha6, a6, d6, theta6, lmin6, lmax6);
	robot.append(&l1);
	robot.append(&l2);
	robot.append(&l3);
	robot.append(&l4);
	robot.append(&l5);
	robot.append(&l6);

	HTransform3D<double> tran = HTransform3D<double>(Vector3D<double>(0, 0, 0.2), Rotation3D<double>());
	Frame tool = Frame(tran);
//	robot.setTool(&tool);
//	solver->init();

	std::shared_ptr<SiasunSR4CSolver> solver(new SiasunSR4CSolver(robot));
	Q qMin = Q(lmin1, lmin2, lmin3, lmin4, lmin5, lmin6);
	Q qMax = Q(lmax1, lmax2, lmax3, lmax4, lmax5, lmax6);
	Q dqLim = Q(3, 3, 3, 3, 5, 5);
	Q ddqLim = Q(20, 20, 20, 20, 20, 20);

//	ikTest();

//	ik2Test();

//	smoothPlannerSampler();

//	p2pPlannerSampler();

//	lineplannerTest();

//	circularplannerTest();

//	simulationtest();

//	smplannertest();

	mlabplannertest();


//	Q pos(0, 0, 0, 0, 0, 0);
//	Q velocity = Q(2./sqrt(3), 2./sqrt(3), 2./sqrt(3), 0, 0, 0);
//	HTransform3D<> end = robot.getEndTransform(pos);
//	double dt = 0.00000001;
//	HTransform3D<> end2 = HTransform3D<>(Vector3D<>(dt*velocity(0), dt*velocity(1), dt*velocity(2)))*end;
//	Config config = robot.getConfig(pos);
//	Q shuzhi;
//	Q jacobian;
//	clock_t start_t = clock();
//	shuzhi = (solver.solve(end2, config)[0] - solver.solve(end, config)[0])/dt; //数值求法
//	clock_t end_t = clock();
//	cout << "数值求法用时: " << end_t - start_t << "us" << endl;
//	start_t = clock();
//	jacobian = robot.getEndVelocity(velocity, pos); //雅克比算法
//	end_t = clock();
//	cout << "雅克比求法用时: " << end_t - start_t << "us" << endl;
//	(shuzhi -jacobian).print();


//	Vector3D<double> p1(0, 0, 0);
//	Vector3D<double> p2(1, 1, 0);
//	Vector3D<double> p3(0.5, 0.6, 0);
//	Interpolator<Vector3D<double> >::ptr circle(new CircularInterpolator<Vector3D<double> >(p1, p2, p3));
//
//	vector<Vector3D<double> > position;
//	double L = circle->duration();
//	int step = 100;
//	double dl = L/(step - 1);
//	for (double l=0; l<=L; l+=dl)
//	{
//		Vector3D<double> tempPosition = circle->x(l);
//		position.push_back(tempPosition);
//	}
//
//	/**> 保存文件 */
//	const char* filename1 = "src/example/tempx.csv";
//	std::ofstream out1(filename1);
//	for (int i=0; i<(int)position.size(); i++)
//	{
//		out1 << position[i](0) << "," << position[i](1) << "," << position[i](2) << endl;
//	}
//	out1.close();


//	std::vector<std::shared_ptr<base> > a;
//	a.push_back(std::shared_ptr<class1>(new class1) );
//	a[0]->print();
	return 0;
}



