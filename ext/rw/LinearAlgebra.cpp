/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#include "LinearAlgebra.hpp"

#include "macros.hpp"

#include <Eigen/SVD>

using namespace rw::math;
using namespace boost::numeric::ublas;
typedef LinearAlgebra::BoostMatrix<double>::type BoostMatrixType;
typedef zero_matrix<double> BoostZeroMatrixType;
typedef matrix<double, column_major> BoostColumnMatrixType;
typedef matrix_range<BoostColumnMatrixType> BoostColumnMatrixTypeRange;

#ifdef RW_USE_UBLAS_LAPACK
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include <boost/numeric/bindings/traits/ublas_matrix.hpp>
#include <boost/numeric/bindings/traits/ublas_vector.hpp>
#include <boost/numeric/bindings/lapack/gesvd.hpp>

namespace lapack = boost::numeric::bindings::lapack;




extern "C" {

    //void sptsv_(int *N, int *NRHS, float *D, float *e, float *b, int *ldb, long *info);

    // same as above but using double precision instead
    //void dptsv_(int *n, int *nrhs, double *d, double *e, double *b, int *ldb, long *info);

}
#endif //RW_USE_UBLAS_LAPACK

#ifdef RW_USE_UBLAS_LAPACK

void LinearAlgebra::svd(
    const BoostMatrixType& M,
    BoostMatrixType& U,
	BoostVector<double>::type& sigma,
    BoostMatrixType& V)
{
    // rows
    const size_t m = M.size1();

    // columns
    const size_t n = M.size2();

    // Define the U, Sigma and V^t
    BoostColumnMatrixType u(m, n);
    vector<double> s(n);
    BoostColumnMatrixType vt(n, n);

    BoostColumnMatrixType Mc(M);
    // Calculate Singular Value Decomposition of A
    lapack::gesvd(Mc, s, u, vt);

    U = u;
    sigma = s;
    V = trans(vt);
}

#endif





BoostMatrixType LinearAlgebra::pseudoInverse(const BoostMatrixType& am, double precision)
{
	Eigen::MatrixXd em(am.size1(), am.size2());
	for (size_t i = 0; i<am.size1(); i++) {
		for (size_t j = 0; j<am.size2(); j++) {
			em(i,j) = am(i,j);
		}
	}
	const Eigen::MatrixXd er = pseudoInverse(em, precision);
	BoostMatrixType br(er.rows(), er.cols());
	for (size_t i = 0; i<br.size1(); i++) {
		for (size_t j = 0; j<br.size2(); j++) {
			br(i,j) = er(i,j);
		}
	}
	return br;

}


#ifdef RW_USE_UBLAS_LAPACK

LinearAlgebra::BoostMatrix<double>::type LinearAlgebra::pseudoInverseLapack(const BoostMatrix<double>::type& am, double precision)
{
    // rows
    const size_t m = am.size1();

    // columns
    const size_t n = am.size2();

    // If matrix is empty return an empty matrix
    if (m == 0 || n == 0)
        return BoostMatrixType();

    if (n > m) 
		return trans(pseudoInverse(trans(am), precision));

    // convert am to column_major form
    BoostColumnMatrixType a(am);

    // Define the U, Sigma and V^t
    BoostColumnMatrixType u(m, n);
    vector<double> s(n);
    BoostColumnMatrixType vt(n, n);

    // Calculate Singular Value Decomposition of A
    lapack::gesvd(a, s, u, vt);

    // Find rank of A (number of non-zero diagonal elements in Sigma)
    size_t rank = 0;
    while (rank < n && s[rank] >= precision) {
        rank++;
    }

    if (rank == 0) return BoostZeroMatrixType(n, m);

    BoostMatrixType s_ = BoostZeroMatrixType(rank, rank);
    for (size_t count = 0; count < rank; count++)
        s_(count, count) = 1.0 / s(count);

    // Calculate: V * S * Ut
    BoostMatrixType t1(prod(trans(BoostColumnMatrixTypeRange(vt,range(0, rank),range(0, n))), s_));

    return prod(t1, trans(BoostColumnMatrixTypeRange(u, range(0, m), range(0, rank))));
}

#endif

Eigen::MatrixXd LinearAlgebra::pseudoInverse(const Eigen::MatrixXd& am, double precision) {
    /*
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd = am.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sigma= svd.singularValues();
	for ( long i=0; i<sigma.cols(); ++i) {
		if ( sigma(i) > precision )
			sigma(i)=1.0/sigma(i);
		else 
			sigma(i)=0;
	}

	return svd.matrixV() * sigma.asDiagonal()* svd.matrixU().transpose();
    */

    if (am.rows() < am.cols()){
        //RW_THROW("pseudoInverse require rows >= to cols!");
        Eigen::MatrixXd a = am.transpose();
        Eigen::JacobiSVD < Eigen::MatrixXd > svd = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = precision * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs().maxCoeff();
        return
                (svd.matrixV()
                        * Eigen::MatrixXd(
                                (svd.singularValues().array().abs() > tolerance).select(
                                        svd.singularValues().array().inverse(), 0)).asDiagonal() * svd.matrixU().adjoint()).transpose();

    } else {

        Eigen::JacobiSVD < Eigen::MatrixXd > svd = am.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = precision * std::max(am.cols(), am.rows()) * svd.singularValues().array().abs().maxCoeff();
        return
                svd.matrixV()
                        * Eigen::MatrixXd(
                                (svd.singularValues().array().abs() > tolerance).select(
                                        svd.singularValues().array().inverse(), 0)).asDiagonal() * svd.matrixU().adjoint();
    }
}

void LinearAlgebra::svd(const Eigen::MatrixXd& M, Eigen::MatrixXd& U, Eigen::VectorXd& sigma, Eigen::MatrixXd& V) {
	const Eigen::JacobiSVD<Eigen::MatrixXd> svd = M.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
	U = svd.matrixU();
	sigma = svd.singularValues();
	V = svd.matrixV();
}

#ifdef RW_USE_UBLAS_LAPACK

bool LinearAlgebra::checkPenroseConditions(
    const BoostMatrixType& A,
    const BoostMatrixType& X,
    double prec)
{
    BoostMatrixType AX = prod(A, X);
    BoostMatrixType XA = prod(X, A);

    if (norm_inf(prod(AX, A) - A) > prec)
        return false;
    if (norm_inf(prod(XA, X) - X) > prec)
        return false;
    if (norm_inf(trans(AX) - AX) > prec)
        return false;
    if (norm_inf(trans(XA) - XA) > prec)
        return false;

    return true;
}

#endif

bool LinearAlgebra::checkPenroseConditions(
	const Eigen::MatrixXd& A,
	const Eigen::MatrixXd& X,
    double prec)
{
	const Eigen::MatrixXd AX = A*X;
	const Eigen::MatrixXd XA = X*A;

	if (((AX*A)-A).lpNorm<Eigen::Infinity>() > prec)
		return false;
	if (((XA*X)-X).lpNorm<Eigen::Infinity>() > prec)
		return false;
	if ((AX.transpose() - AX).lpNorm<Eigen::Infinity>() > prec)
		return false;
	if ((XA.transpose() - XA).lpNorm<Eigen::Infinity>() > prec)
		return false;
	return true;
}

#ifdef RW_USE_UBLAS_LAPACK

void LinearAlgebra::lapack_triDiagonalSolve(int *N, int *NRHS, float *D, float *e, float *b, int *ldb, int *info){
    LAPACK_SPTSV(N, NRHS, D, e, b, ldb, info);
}

void LinearAlgebra::lapack_triDiagonalSolve(int *N, int *NRHS, double *D, double *e, double *b, int *ldb, int *info){
    LAPACK_DPTSV(N,NRHS,D,e,b,ldb,info);
}

#endif //RW_USE_UBLAS_LAPACK

template<>
std::pair<LinearAlgebra::EigenMatrix<double>::type, LinearAlgebra::EigenVector<double>::type> LinearAlgebra::eigenDecompositionSymmetric<double>(const Eigen::MatrixXd& Am1) {
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver;
	eigenSolver.compute(Am1);
	return std::make_pair(eigenSolver.eigenvectors(), eigenSolver.eigenvalues());
}

template<>
std::pair<LinearAlgebra::EigenMatrix<std::complex<double> >::type, LinearAlgebra::EigenVector<std::complex<double> >::type > LinearAlgebra::eigenDecomposition<double>(const Eigen::MatrixXd& Am1) {
	Eigen::EigenSolver<Eigen::MatrixXd> eigenSolver;
	eigenSolver.compute(Am1);
	return std::make_pair(eigenSolver.eigenvectors(), eigenSolver.eigenvalues());
}
