/*
 * qblendtest.cpp
 *
 *  Created on: Sep 14, 2017
 *      Author: a1994846931931
 */

#ifdef COMPILEALLEXAMPLES

# include "qblendtest.h"
# include <string>
# include "../../common/printAdvance.h"
# include "../../model/Link.h"
# include "../../ik/SiasunSR4CSolver.h"
# include "../../math/Quaternion.h"
# include <vector>
# include "../../trajectory/Interpolator.h"
# include "../../trajectory/LinearInterpolator.h"
# include "../../trajectory/ConvertedInterpolator.h"
# include "../../pathplanner/PointToPointPlanner.h"
# include "../../pathplanner/LinePlanner.h"
# include "../../pathplanner/QBlend.h"
# include <memory>
# include <fstream>

using namespace robot::math;
using namespace robot::kinematic;
using namespace robot::common;
using namespace robot::model;
using namespace robot::ik;
using namespace robot::trajectory;
using namespace robot::pathplanner;

void qblendtest()
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
		LinePlanner lineplanner = LinePlanner(qMin, qMax, dqLim, ddqLim, vMaxLine, aMaxLine, hLine, vMaxAngle, aMaxAngle, hAngle,
				solver, &robot);
		QBlend blendder = QBlend(ddqLim, dqLim, qMin, qMax);

//		robot.setTool(&tool);
//		solver->init();
		Q p1 = Q::zero(6);
		Q p2 = Q(1.5, 0, 0, 0, -1.5, 0);
		Q p3 = Q(3, 0, 0, 0, 0, 0);
		println("line1");
		Interpolator<Q>::ptr line1 = lineplanner.query(p1, p2);
		println("line2");
		Interpolator<Q>::ptr line2 = lineplanner.query(p2, p3);
		double T1 = line1->duration();
		double T2 = line2->duration();
		double k = 0.2;
		double t1 = T1*(1 - k);
		double t2 = T2*k;
		clock_t clockStart = clock();
		Interpolator<Q>::ptr qInterpolator = blendder.query(line1->getState(t1), line2->getState(t2), T1 - t1 + t2);
		clock_t clockEnd = clock();
		cout << "插补器构造用时: " << clockEnd - clockStart << "us" << endl;
		/**> 采样 */
		int step = 300;
		cout << "线段1时长: " << T1 << "s" << endl;
		cout << "线段2时长: " << T2 << "s" << endl;
		cout << "原时长: " << k*(T1 + T2) << "s" << endl;
		cout << "总时长: " << qInterpolator->duration() << "s" << endl;
		std::vector<Q> line1x;
		std::vector<Q> line2x;
		std::vector<Q> blendx;
		std::vector<Q> blenddx;
		std::vector<Q> blendddx;
		clockStart = clock();
		blendx = qInterpolator->xSample(step);
		blenddx = qInterpolator->dxSample(step);
		blendddx = qInterpolator->ddxSample(step);
		clockEnd = clock();
		cout << "每次插补用时: " << (clockEnd - clockStart)/(double)step << "us" << endl;
		/**> 两条直线采样 */
		line1x = line1->xSample(step);
		line2x = line2->xSample(step);
		/**> 保存文件 */
		const char* filename1 = "src/example/qblendtest/templine1x.txt";
		const char* filename2 = "src/example/qblendtest/templine2x.txt";
		const char* filename3 = "src/example/qblendtest/tempblendx.txt";
		const char* filename4 = "src/example/qblendtest/tempblenddx.txt";
		const char* filename5 = "src/example/qblendtest/tempblendddx.txt";
		std::ofstream out1(filename1);
		for (int i=0; i<(int)line1x.size(); i++)
		{
			out1 << line1x[i][0] << ", " << line1x[i][1] << ", " << line1x[i][2] << ", " << line1x[i][3] << ", " << line1x[i][4] << ", " << line1x[i][5] << ";" << endl;
		}
		out1.close();
		std::ofstream out2(filename2);
		for (int i=0; i<(int)line1x.size(); i++)
		{
			out2 << line2x[i][0] << ", " << line2x[i][1] << ", " << line2x[i][2] << ", " << line2x[i][3] << ", " << line2x[i][4] << ", " << line2x[i][5] << ";" << endl;
		}
		out2.close();
		std::ofstream out3(filename3);
		for (int i=0; i<(int)line1x.size(); i++)
		{
			out3 << blendx[i][0] << ", " << blendx[i][1] << ", " << blendx[i][2] << ", " << blendx[i][3] << ", " << blendx[i][4] << ", " << blendx[i][5] << ";" << endl;
		}
		out3.close();
		std::ofstream out4(filename4);
		for (int i=0; i<(int)line1x.size(); i++)
		{
			out4 << blenddx[i][0] << ", " << blenddx[i][1] << ", " << blenddx[i][2] << ", " << blenddx[i][3] << ", " << blenddx[i][4] << ", " << blenddx[i][5] << ";" << endl;
		}
		out4.close();
		std::ofstream out5(filename5);
		for (int i=0; i<(int)line1x.size(); i++)
		{
			out5 << blendddx[i][0] << ", " << blendddx[i][1] << ", " << blendddx[i][2] << ", " << blendddx[i][3] << ", " << blendddx[i][4] << ", " << blendddx[i][5] << ";" << endl;
		}
		out5.close();
	}
	catch (std::string& msg)
	{
		std::cerr << msg;
	}
}

#endif
