/*
 * Jacobian.cpp
 *
 *  Created on: Aug 24, 2017
 *      Author: a1994846931931
 */

# include "Jacobian.h"
# include "../ext/Eigen/Dense"
# include "../common/printAdvance.h"

namespace robot {
namespace model {

Jacobian::Jacobian() {
	// TODO Auto-generated constructor stub

}

Jacobian::Jacobian(double (&j)[6][6]):_j(6, 6)
{
	for (int i=0; i<6; i++)
	{
		for (int k=0; k<6; k++)
		{
			_j(i, k) = j[i][k];
		}
	}
}

void Jacobian::doInverse()
{
	_j = _j.inverse();
}

Jacobian Jacobian::inverse() const
{
	Jacobian jcob;
	jcob.getMatrix().inverse();
	return jcob;
}

int Jacobian::rank() const
{
	Eigen::ColPivHouseholderQR<Eigen::MatrixXd > QR_decomp(_j);
	return QR_decomp.rank();
}

void Jacobian::print() const
{
	cout.precision(4);
	cout << setfill('_') << setw(14*6) <<  "_" << endl;
	for (int i=0; i<6; i++)
	{
		for (int j=0; j<6; j++)
		{
			cout << setfill(' ') << setw(12) << _j(i, j) << " |";
		}
		cout << endl;
	}
}

Jacobian::~Jacobian() {
	// TODO Auto-generated destructor stub
}


} /* namespace model */
} /* namespace robot */
