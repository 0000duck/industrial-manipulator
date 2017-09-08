# include "smoothPlannerSampler.h"
# include <time.h>
# include "../../common/printAdvance.h"
# include <fstream>

using namespace robot::common;
using namespace robot::pathplanner;
using namespace robot::trajectory;

void smoothPlannerSampler(){
	SmoothMotionPlanner planner;
	double h = 100;
	double aMax = 15;
	double vMax = 2;
	double s = 2;
	clock_t clockStart = clock();
	Interpolator<double>* smoothLinearInterpolator = planner.query(s, h, aMax, vMax);
	clock_t clockEnd = clock();
	cout << "插补器构造用时: " << clockEnd - clockStart << "us" << endl;
	int step = 1000;
	double T = smoothLinearInterpolator->duration();
	double dt = T/(step - 1);
	cout << "总时长: " << T << "s" << endl;
	std::vector<double> x;
	std::vector<double> dx;
	std::vector<double> ddx;
	clockStart = clock();
	for (double t=0; t<T; t+=dt)
	{
		x.push_back(smoothLinearInterpolator->x(t));
		dx.push_back(smoothLinearInterpolator->dx(t));
		ddx.push_back(smoothLinearInterpolator->ddx(t));
	}
	clockEnd = clock();
	cout << "插补用时: " << clockEnd - clockStart << "us" << endl;

	/**> 保存文件 */
	const char* filename1 = "src/example/smoothplanner/tempx.txt";
	std::ofstream out1(filename1);
	for (int i=0; i<(int)x.size(); i++)
	{
		out1 << "(" << x[i] << ", " << dt*i << ")" << endl;;
	}
	const char* filename2 = "src/example/smoothplanner/tempdx.txt";
	std::ofstream out2(filename2);
	for (int i=0; i<(int)x.size(); i++)
	{
		out2 << "(" << dx[i] << ", " << dt*i << ")" << endl;;
	}
	const char* filename3 = "src/example/smoothplanner/tempddx.txt";
	std::ofstream out3(filename3);
	for (int i=0; i<(int)x.size(); i++)
	{
		out3 << "(" << ddx[i] << ", " << dt*i << ")" << endl;;
	}
	out1.close();
	out2.close();
	out3.close();
}
