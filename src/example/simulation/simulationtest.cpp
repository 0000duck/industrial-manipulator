/*
 * simulationtest.cpp
 *
 *  Created on: Sep 19, 2017
 *      Author: a1994846931931
 */

# include "simulationtest.h"
# include <string>
# include "../../common/printAdvance.h"
# include "../../model/Link.h"
# include "../../ik/SiasunSR4CSolver.h"
# include "../../math/Quaternion.h"
# include <vector>
# include "../../trajectory/LinearInterpolator.h"
# include "../../trajectory/ConvertedInterpolator.h"
# include "../../pathplanner/PointToPointPlanner.h"
# include "../../pathplanner/LinePlanner.h"
# include "../../simulation/IterativeSimulator.h"
# include <memory>
# include <fstream>

using namespace robot::math;
using namespace robot::kinematic;
using namespace robot::common;
using namespace robot::model;
using namespace robot::ik;
using namespace robot::trajectory;
using namespace robot::pathplanner;
using namespace robot::simulation;

void simulationtest()
{
	SerialLink robot;
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
	double lmin1 = M_PI/180.0*-180;
	double lmin2 = M_PI/180.0*-130;
	double lmin3 = M_PI/180.0*-70;
	double lmin4 = M_PI/180.0*-240;
	double lmin5 = M_PI/180.0*-200;
	double lmin6 = M_PI/180.0*-360;
	double lmax1 = M_PI/180.0*180;
	double lmax2 = M_PI/180.0*80;
	double lmax3 = M_PI/180.0*160;
	double lmax4 = M_PI/180.0*240;
	double lmax5 = M_PI/180.0*30;
	double lmax6 = M_PI/180.0*360;

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
	robot.setTool(&tool);
//	solver->init();

	SiasunSR4CSolver solver(robot);
	std::shared_ptr<SiasunSR4CSolver> solverPtr(new SiasunSR4CSolver(robot));
	Q qMin = Q(lmin1, lmin2, lmin3, lmin4, lmin5, lmin6);
	Q qMax = Q(lmax1, lmax2, lmax3, lmax4, lmax5, lmax6);
	Q dqLim = Q(3, 3, 3, 3, 5, 5);
	Q ddqLim = Q(20, 20, 20, 20, 20, 20);
	double vMaxLine = 1.0;
	double aMaxLine = 20.0;
	double hLine = 50;
	double vMaxAngle = 1.0;
	double aMaxAngle = 10.0;
	double hAngle = 30;
	IterativeSimulator simulator(&robot);
	LinePlanner planner = LinePlanner(qMin, qMax, dqLim, ddqLim, vMaxLine, aMaxLine, hLine, vMaxAngle, aMaxAngle, hAngle,
			solverPtr, &robot);
//	robot.setTool(&tool);
//	solver->init();
	clock_t clockStart = clock();
	Q start = simulator.getState().getAngle();
	Q end =  Q(1.5, 0, 0, 0, -1.5, 0);
	Interpolator<Q>::ptr qInterpolator = planner.query(start, end);
	clock_t clockEnd = clock();
	cout << "插补器构造用时: " << clockEnd - clockStart << "us" << endl;
	int step = 1000;
	double T = qInterpolator->duration();
	double dt = T/(step - 1);
	vector<Q> linePosition;
	vector<Q> realPosition;
	cout << "总时长: " << T << "s" << endl;
	for (double t=0; t<=T; t+=dt)
	{
		simulator.setSpeed(qInterpolator->dx(t), dt);
		realPosition.push_back(simulator.getState().getAngle());
		linePosition.push_back(qInterpolator->x(t));
	}
	const char* filename = "src/example/simulation/temprealx.txt";
	std::ofstream out(filename);
	for (int i=0; i<(int)realPosition.size(); i++)
	{
		out << realPosition[i][0] << ", " << realPosition[i][1] << ", " << realPosition[i][2] << ", "
				<< realPosition[i][3] << ", " << realPosition[i][4] << ", " << realPosition[i][5] << ";" << endl;
	}
	out.close();
	filename = "src/example/simulation/templinex.txt";
	out.open(filename);
	for (int i=0; i<(int)realPosition.size(); i++)
	{
		out << linePosition[i][0] << ", " << linePosition[i][1] << ", " << linePosition[i][2] << ", "
				<< linePosition[i][3] << ", " << linePosition[i][4] << ", " << linePosition[i][5] << ";" << endl;
	}
	out.close();
}


