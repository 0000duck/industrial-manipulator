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
# include "../common/fileAdvance.h"
# include "../common/common.h"
# include "../model/Link.h"
# include "../ik/SiasunSR4CSolver.h"
# include "../model/Config.h"
# include "../math/Quaternion.h"
# include "../math/Integrator.h"
# include "../trajectory/LinearInterpolator.h"
# include "../trajectory/ConvertedInterpolator.h"
# include "../trajectory/MLABTrajectory.h"
# include "../trajectory/BezierPath.h"
# include "../trajectory/Sampler.h"
# include "../pathplanner/PointToPointPlanner.h"
# include "../pathplanner/LinePlanner.h"
# include "../pathplanner/QBlend.h"
# include "../pathplanner/CircularPlanner.h"
# include "../pathplanner/MultiLineArcBlendPlanner.h"
# include "../pathplanner/SMPlannerEx.h"
# include "../simulation/IterativeSimulator.h"
# include "../simulation/MotionStack.h"
# include "../parse/RobotXMLParser.h"
# include <math.h>
# include <string>
# include <vector>
# include <memory>
# include <fstream>
# include <algorithm>
# include <functional>
# include <sys/time.h>
# include <stdlib.h>
# include <stdio.h>
# include <unistd.h>
# include <thread>
//# include "test.h"
//# include "testIK.h"
//# include "testIK2.h"
//# include "smoothplanner/smoothPlannerSampler.h"
//# include "p2pplanner/p2pPlannerSampler.h"
# include "lineplanner/lineplannertest.h"
//# include "qblendtest/qblendtest.h"
# include "circularplanner/circularplannertest.h"
//# include "simulation/simulationtest.h"
//# include "smplannerex/smplannertest.h"
# include "mlabplanner/mlabplannertest.h"
# include "motionstack/motionstacktest.h"
# include <functional>
# include <map>

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


int main(){
	println("*** test ***");
//	SerialLink robot;

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

	/**> 读取模型文件 */
	robot::parse::RobotXMLParser modelParser;
	SerialLink::ptr robot = modelParser.parse("src/example/modelData/siasun6.xml");

	/**> 设置工具(可选) */
	HTransform3D<double> tran = HTransform3D<double>(Vector3D<double>(0, 0, 0.2), Rotation3D<double>());
	Frame tool = Frame(tran);
//	robot.setTool(&tool);
//	solver->init();

	/**> 逆解器 */
	std::shared_ptr<SiasunSR4CSolver> solver(new SiasunSR4CSolver(robot));

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

//	mlabplannertest();

//	motionstacktest();

	string test = "abcd";
	cout << test.substr(1, 3);

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



