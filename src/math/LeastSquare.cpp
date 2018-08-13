/*
 * LeastSquare.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: a1994846931931
 */

#include "LeastSquare.h"
# include "../ext/Eigen/Dense"
# include "../common/printAdvance.h"

using namespace Eigen;

namespace robot {
namespace math {

LeastSquare::LeastSquare() {
	// TODO Auto-generated constructor stub

}

LeastSquare::~LeastSquare() {
	// TODO Auto-generated destructor stub
}

bool LeastSquare::fromMatrix(const vector<vector<double> > &A, const vector<double> &b, vector<double> &X)
{
	/** 获取大小和检查参数 */
	int row = (int)A.size();
	int colA = (int)A[0].size();
	if (row <= 0 ||row <colA || row != (int)b.size())
		return false;
	for  (int i=1; i<row; i++)
		if (colA != (int)A[i].size())
			return false;
	MatrixXd mA(row, colA);
	VectorXd vb(row);
	for (int i=0; i<row; i++)
	{
		for (int j=0; j<colA; j++)
			mA(i, j) = A[i][j];
		vb(i) = b[i];
	}
	cout << mA << endl << endl;
	cout << vb << endl << endl;
	VectorXd vX(colA);
	vX = mA.jacobiSvd(ComputeThinU | ComputeThinV).solve(vb);
	cout << vX << endl << endl;
	X.clear();
	for (int i=0; i<colA; i++)
		X.push_back(vX(i));
	return true;
}

} /* namespace model */
} /* namespace robot */
