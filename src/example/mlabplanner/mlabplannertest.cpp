# include <math.h>
# include <string>
# include <vector>
# include <memory>
# include <fstream>
# include "../../pathplanner/MultiLineArcBlendPlanner.h"
# include "../../ik/SiasunSR4CSolver.h"

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

	MultiLineArcBlendPlanner mlabplanner(qMin, qMax, dqLim, ddqLim, solver, &robot);

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

	vector<double> jerk(2, 50);

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
	std::vector<Q> x;
	std::vector<Q> dx;
	std::vector<Q> ddx;
	std::vector<double> l;
	std::vector<double> dl;
	clockStart = clock();
	try{
		for (double t=0; t<=T; t+=dt)
		{
			x.push_back(mlabTrajectory->x(t));
			dx.push_back(mlabTrajectory->dx(t));
			ddx.push_back(mlabTrajectory->ddx(t));
			dl.push_back(mlabTrajectory->dl(t));
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
	const char* filename4 = "src/example/mlabplanner/tempdl.csv";
	std::ofstream out4(filename4);
	for (int i=0; i<(int)x.size(); i++)
	{
		out4 << dl[i] << endl;
	}
	out4.close();
	clockEnd = clock();
	cout << "每次插补用时: " << (clockEnd - clockStart)/(double)step << "us" << endl;
}
