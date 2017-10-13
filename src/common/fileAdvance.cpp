/*
 * fileAdvance.cpp
 *
 *  Created on: Oct 12, 2017
 *      Author: a1994846931931
 */
# include "fileAdvance.h"
# include <fstream>

namespace robot{
namespace common{

bool saveQPath(const char* filename, vector<Q>& qPath)
{
	std::ofstream out(filename);
	for (int i=0; i<(int)qPath.size(); i++)
	{
		out << qPath[i][0] << ", " << qPath[i][1] << ", " << qPath[i][2] << ", " << qPath[i][3] << ", " << qPath[i][4] << ", " << qPath[i][5] << ";\n";
	}
	out.close();
	return true;
}

bool savePosPath(const char* filename, vector<Vector3D<double>>& vPath)
{
	std::ofstream out(filename);
	for (int i=0; i<(int)vPath.size(); i++)
	{
		out << vPath[i][0] << ", " << vPath[i][1] << ", " << vPath[i][2] << "\n";
	}
	out.close();
	return true;
}

bool saveDoublePath(const char* filename, vector<double>& doublePath, vector<double>& time)
{
	std::ofstream out(filename);
	for (int i=0; i<(int)doublePath.size(); i++)
	{
		out << doublePath[i] << "," << time[i] << '\n';
	}
	out.close();
	return true;
}

}
}
