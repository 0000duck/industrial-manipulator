/*
 * lineplanner.cpp
 *
 *  Created on: Sep 13, 2017
 *      Author: a1994846931931
 */

#define COMPILEALLEXAMPLES

#ifdef COMPILEALLEXAMPLES

# include "lineplannertest.h"
# include <string>
# include "../../common/printAdvance.h"
# include "../../model/Link.h"
# include "../../ik/SiasunSR4CSolver.h"
# include "../../math/Quaternion.h"
# include "../../math/Integrator.h"
# include <vector>
# include "../../trajectory/LinearInterpolator.h"
# include "../../trajectory/LineTrajectory.h"
# include "../../pathplanner/PointToPointPlanner.h"
# include "../../pathplanner/LinePlanner.h"
# include <memory>
# include "../../parse/RobotXMLParser.h"
# include <fstream>

using namespace robot::math;
using namespace robot::kinematic;
using namespace robot::common;
using namespace robot::model;
using namespace robot::ik;
using namespace robot::trajectory;
using namespace robot::pathplanner;

LineTrajectory::ptr lineplannerTest()
{
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
	double vMaxLine = 1.0;
	double aMaxLine = 20.0;
	double hLine = 50;
//	double vMaxAngle = 1.0;
//	double aMaxAngle = 10.0;
//	double hAngle = 30;
	LineTrajectory::ptr lineIpr;
	try{

		clock_t clockStart = clock();
		Q start = Q::zero(6);
		Q end =  Q(1.5, 0, 0, 0, -1.5, 0);
		LinePlanner planner = LinePlanner(dqLim, ddqLim, vMaxLine, aMaxLine, hLine, solver, start, end);
		lineIpr = planner.query();
		clock_t clockEnd = clock();
		cout << "插补器构造用时: " << clockEnd - clockStart << "us" << endl;
		int step = 300;
		const double T = lineIpr->duration();
		double L = lineIpr->getLIpr()->x(T);
		double dt = T/(step - 1);
		cout << "总时长: " << T << "s" << endl;
		cout << "总长度: " << L << "m" << endl;
		std::vector<double> time;
		std::vector<Q> x;
		std::vector<Q> dx;
		std::vector<Q> ddx;
		std::vector<double> l;
		std::vector<double> dl;
		std::vector<double> ddl;
		clockStart = clock();
		try{
			for (double t=0; t<=T; t+=dt)
			{
				time.push_back(t);
				x.push_back(lineIpr->x(t));
				dx.push_back(lineIpr->dx(t));
				ddx.push_back(lineIpr->ddx(t));
				l.push_back(lineIpr->l(t));
				dl.push_back(lineIpr->dl(t));
				ddl.push_back(lineIpr->ddl(t));
			}
		}
		catch(std::string& msg)
		{
			cout << msg << endl;;
		}
		clockEnd = clock();
		cout << "每次插补用时: " << (clockEnd - clockStart)/(double)step << "us" << endl;
		/**> 保存文件 */
		const char* filename1 = "src/example/lineplanner/tempx.txt";
		std::ofstream out1(filename1);
		for (int i=0; i<(int)x.size(); i++)
		{
			out1 << x[i][0] << ", " << x[i][1] << ", " << x[i][2] << ", " << x[i][3] << ", " << x[i][4] << ", " << x[i][5] << ";" << endl;
		}
		out1.close();
		const char* filename2 = "src/example/lineplanner/tempdx.txt";
		std::ofstream out2(filename2);
		for (int i=0; i<(int)x.size(); i++)
		{
			out2 << dx[i][0] << ", " << dx[i][1] << ", " << dx[i][2] << ", " << dx[i][3] << ", " << dx[i][4] << ", " << dx[i][5] << ";" << endl;
		}
		out2.close();
		const char* filename3 = "src/example/lineplanner/tempddx.txt";
		std::ofstream out3(filename3);
		for (int i=0; i<(int)x.size(); i++)
		{
			out3 << ddx[i][0] << ", " << ddx[i][1] << ", " << ddx[i][2] << ", " << ddx[i][3] << ", " << ddx[i][4] << ", " << ddx[i][5] << ";" << endl;
		}
		out3.close();
		const char* filename4 = "src/example/lineplanner/templ.csv";
		std::ofstream out4(filename4);
		for (int i=0; i<(int)x.size(); i++)
		{
			out4 << l[i] << "," << time[i] << endl;
		}
		out4.close();
		const char* filename5 = "src/example/lineplanner/tempdl.csv";
		std::ofstream out5(filename5);
		for (int i=0; i<(int)x.size(); i++)
		{
			out5 << dl[i] << "," << time[i] << endl;
		}
		out5.close();
		const char* filename6 = "src/example/lineplanner/tempddl.csv";
		std::ofstream out6(filename6);
		for (int i=0; i<(int)x.size(); i++)
		{
			out6 << ddl[i] << "," << time[i] << endl;
		}
		out6.close();
		/**> 长度询问测试 */
//		cout << "长度测试" << endl;
//		step = 10;
//		cout.precision(4);
//		cout << setfill('_') << setw(10) << "t"
//				<< setfill('_') << setw(10) << "l"
//				<< setfill('_') << setw(10) << "t"
//				<< setfill('_') << setw(10) << "error" << endl;
//		for (double t=0; t<=T; t+=T/(step - 1))
//		{
//			cout << setfill('_') << setw(10) << t
//					<< setfill('_') << setw(10) << lineIpr->getLIpr()->x(t)
//					<< setfill('_') << setw(10) << lineIpr->timeAt(lineIpr->getLIpr()->x(t))
//					<< setfill('_') << setw(10) << lineIpr->timeAt(lineIpr->getLIpr()->x(t)) - t<< endl;
//		}

		/**> 长度积分器测试　deleted */

	}
	catch (char const* msg)
	{
		std::cerr << msg;
	}
	return lineIpr;
}

#endif
