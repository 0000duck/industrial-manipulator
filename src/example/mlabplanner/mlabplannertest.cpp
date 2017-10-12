# include <math.h>
# include <string>
# include <vector>
# include <memory>
# include <fstream>
# include "../../pathplanner/MultiLineArcBlendPlanner.h"
# include "../../ik/SiasunSR4CSolver.h"
# include "../../parse/RobotXMLParser.h"

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



void mlabplannertest()
{
	println("*** test ***");

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

	MultiLineArcBlendPlanner mlabplanner(dqLim, ddqLim, solver, robot);

	vector<Q> path;
	path.push_back( Q::zero(6));
	path.push_back( Q(0.7, 0, -60.0/180*M_PI, 0, 0, 0));
	path.push_back( Q(1.4, 0, 80.0/180*M_PI, 0, 0, 0));

	vector<double> arcRatio;
	arcRatio.push_back(0.1);

	vector<double> velocity;
	velocity.push_back(1.0);
	velocity.push_back(1.0);

	vector<double> acceleration;
	acceleration.push_back(15.0);
	acceleration.push_back(15.0);

	vector<double> jerk(2, 50.0);

	MLABTrajectory::ptr mlabTrajectory;

	clock_t clockStart = clock();
	try{
		mlabTrajectory = mlabplanner.query(path, arcRatio, velocity, acceleration, jerk);
	}
	catch(char const* msg)
	{
		println(msg);
	}
	catch(string& msg)
	{
		println(msg);
	}
	clock_t clockEnd = clock();
	cout << "插补器构造用时: " << clockEnd - clockStart << "us" << endl;

	/**************** 路径测试
	vector<Interpolator<Vector3D<double> >::ptr> posIpr = mlabTrajectory->getPosIpr();
	println("get pos Ipr");
	std::vector<Vector3D<double> > position;
	for (int i=0; i< (int)posIpr.size(); i++)
	{
		double L = posIpr[i]->duration();
		int step = 100;
		double dl = L/(step - 1);
		for (double l=0; l<=L; l+=dl)
		{
			Vector3D<double> tempPosition = posIpr[i]->x(l);
			position.push_back(tempPosition);
		}
	}
	// 保存文件
	const char* filename1 = "src/example/tempx.csv";
	std::ofstream out1(filename1);
	for (int i=0; i<(int)position.size(); i++)
	{
		out1 << position[i](0) << "," << position[i](1) << "," << position[i](2) << endl;
	}
	out1.close();
	**************************/

	int step = 500;
	const double T = mlabTrajectory->duration();
	double dt = T/(step - 1);
	cout << "总时长: " << T << "s" << endl;
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
			x.push_back(mlabTrajectory->x(t));
			dx.push_back(mlabTrajectory->dx(t));
			ddx.push_back(mlabTrajectory->ddx(t));
			l.push_back(mlabTrajectory->l(t));
			dl.push_back(mlabTrajectory->dl(t));
			ddl.push_back(mlabTrajectory->ddl(t));
		}
	}
	catch(char const* msg)
	{
		println(msg);
	}
	catch(string& msg)
	{
		println(msg);
	}
	clockEnd = clock();
	cout << "每次插补用时: " << (clockEnd - clockStart)/(double)step << "us" << endl;
	const char* filename1 = "src/example/mlabplanner/tempx.csv";
	std::ofstream out1(filename1);
	for (int i=0; i<(int)x.size(); i++)
	{
		out1 << x[i][0] << ", " << x[i][1] << ", " << x[i][2] << ", " << x[i][3] << ", " << x[i][4] << ", " << x[i][5] << ";" << endl;
	}
	out1.close();
	const char* filename2 = "src/example/mlabplanner/tempdx.csv";
	std::ofstream out2(filename2);
	for (int i=0; i<(int)x.size(); i++)
	{
		out2 << dx[i][0] << ", " << dx[i][1] << ", " << dx[i][2] << ", " << dx[i][3] << ", " << dx[i][4] << ", " << dx[i][5] << ";" << endl;
	}
	out2.close();
	const char* filename3 = "src/example/mlabplanner/tempddx.csv";
	std::ofstream out3(filename3);
	for (int i=0; i<(int)x.size(); i++)
	{
		out3 << ddx[i][0] << ", " << ddx[i][1] << ", " << ddx[i][2] << ", " << ddx[i][3] << ", " << ddx[i][4] << ", " << ddx[i][5] << ";" << endl;
	}
	out3.close();
	const char* filename4 = "src/example/mlabplanner/templ.csv";
	std::ofstream out4(filename4);
	for (int i=0; i<(int)x.size(); i++)
	{
		out4 << l[i] << "," << time[i] << endl;
	}
	out4.close();
	const char* filename5 = "src/example/mlabplanner/tempdl.csv";
	std::ofstream out5(filename5);
	for (int i=0; i<(int)x.size(); i++)
	{
		out5 << dl[i] << "," << time[i] << endl;
	}
	out5.close();
	const char* filename6 = "src/example/mlabplanner/tempddl.csv";
	std::ofstream out6(filename6);
	for (int i=0; i<(int)x.size(); i++)
	{
		out6 << ddl[i] << "," << time[i] << endl;
	}
	out6.close();
}
