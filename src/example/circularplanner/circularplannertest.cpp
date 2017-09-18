/*
 * circularplannertest.cpp
 *
 *  Created on: Sep 18, 2017
 *      Author: a1994846931931
 */

# include "circularplannertest.h"
# include <string>
# include "../../common/printAdvance.h"
# include "../../model/Link.h"
# include "../../ik/SiasunSR4CSolver.h"
# include "../../math/Quaternion.h"
# include <vector>
# include "../../trajectory/LinearInterpolator.h"
# include "../../trajectory/ConvertedInterpolator.h"
# include "../../pathplanner/CircularPlanner.h"
# include <memory>
# include <fstream>

using namespace robot::math;
using namespace robot::kinematic;
using namespace robot::common;
using namespace robot::model;
using namespace robot::ik;
using namespace robot::trajectory;
using namespace robot::pathplanner;

void circularplannerTest()
{
	SerialLink robot;
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

	std::shared_ptr<SiasunSR4CSolver> solver(new SiasunSR4CSolver(robot));
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
	try{
		CircularPlanner planner = CircularPlanner(qMin, qMax, dqLim, ddqLim, vMaxLine, aMaxLine, hLine, vMaxAngle, aMaxAngle, hAngle,
				solver, &robot);

	//	robot.setTool(&tool);
	//	solver->init();

		clock_t clockStart = clock();
		Q start = Q::zero(6);
		Q intermediate = Q(0.7, 0, 0, 0, -1, 0);
		Q end =  Q(1.5, 0, 0, 0, -1.5, 0);
		Interpolator<Q>::ptr qInterpoaltor = planner.query(start, intermediate, end);
		clock_t clockEnd = clock();
		cout << "插补器构造用时: " << clockEnd - clockStart << "us" << endl;
		int step = 1000;
		double T = qInterpoaltor->duration();
		double dt = T/(step - 1);
		cout << "总时长: " << T << "s" << endl;
		std::vector<Q> x;
		std::vector<Q> dx;
		std::vector<Q> ddx;
		clockStart = clock();
		for (double t=0; t<=T; t+=dt)
		{
			x.push_back(qInterpoaltor->x(t));
			dx.push_back(qInterpoaltor->dx(t));
			ddx.push_back(qInterpoaltor->ddx(t));
		}
		clockEnd = clock();
		cout << "每次插补用时: " << (clockEnd - clockStart)/(double)step << "us" << endl;
		/**> 保存文件 */
		const char* filename1 = "src/example/circularplanner/tempx.txt";
		std::ofstream out1(filename1);
		for (int i=0; i<(int)x.size(); i++)
		{
			out1 << x[i][0] << ", " << x[i][1] << ", " << x[i][2] << ", " << x[i][3] << ", " << x[i][4] << ", " << x[i][5] << ";" << endl;
		}
		out1.close();
		const char* filename2 = "src/example/circularplanner/tempdx.txt";
		std::ofstream out2(filename2);
		for (int i=0; i<(int)x.size(); i++)
		{
			out2 << dx[i][0] << ", " << dx[i][1] << ", " << dx[i][2] << ", " << dx[i][3] << ", " << dx[i][4] << ", " << dx[i][5] << ";" << endl;
		}
		out2.close();
		const char* filename3 = "src/example/circularplanner/tempddx.txt";
		std::ofstream out3(filename3);
		for (int i=0; i<(int)x.size(); i++)
		{
			out3 << ddx[i][0] << ", " << ddx[i][1] << ", " << ddx[i][2] << ", " << ddx[i][3] << ", " << ddx[i][4] << ", " << ddx[i][5] << ";" << endl;
		}
		out3.close();

		clockStart = clock();
		qInterpoaltor->doLengthAnalysis(&robot);
		clockEnd = clock();
		cout << "分析路径长度耗时: " << clockEnd - clockStart << "us" << endl;

		std::vector<std::pair<double, double> > _trajectoryLength;
		Vector3D<double> position1;
		Vector3D<double> position2 = (&robot)->getEndPosition(qInterpoaltor->x(0));
		_trajectoryLength.push_back(std::pair<double, double>(0, 0));
		int i = 0;
		for (double t = dt; t<= T; t+=dt)
		{
			position1 = position2;
			position2 = (&robot)->getEndPosition(qInterpoaltor->x(t));
			_trajectoryLength.push_back(std::pair<double, double>(t, _trajectoryLength[i++].second + (position2 - position1).getLength()));
		}
	}
	catch (char const* msg)
	{
		std::cerr << msg;
	}
}



