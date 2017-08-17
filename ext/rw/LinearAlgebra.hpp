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


#ifndef RW_MATH_LINEARALGEBRA_HPP
#define RW_MATH_LINEARALGEBRA_HPP

//#define RW_USE_UBLAS_LAPACK
		
/**
 * @file LinearAlgebra.hpp
 */

#include <boost/numeric/ublas/matrix.hpp>
#define RW_USE_UBLAS_LAPACK // define
#ifdef RW_USE_UBLAS_LAPACK
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <boost/numeric/bindings/lapack/workspace.hpp>
#include <boost/numeric/bindings/lapack/syev.hpp>
#include <boost/numeric/bindings/lapack/geev.hpp>

#include <boost/numeric/bindings/traits/ublas_matrix.hpp>
#include <boost/numeric/bindings/traits/ublas_vector.hpp>
#endif

#ifdef RW_USE_UBLAS_LAPACK
#include <boost/numeric/bindings/lapack/gesvd.hpp>
#include <boost/numeric/bindings/lapack/ptsv.hpp>
#endif

#include "../Eigen/Eigen"

#include <limits>

namespace rw { namespace math {

    /** @addtogroup math */
    /* @{*/


    /**
     * @brief Collection of Linear Algebra functions
     */
    class LinearAlgebra
    {
    public:

    	//! @brief Type for Eigen matrices used to reduce namespace cluttering.
    	template<class T=double>
    	struct EigenMatrix {
    	    //! type of this matrix
			typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> type;
    	};

    	//! @brief Type for Eigen vectors, used to reduce namespace cluttering.
    	template<class T=double>
    	struct EigenVector {
    	    //! type of this Vector
			typedef Eigen::Matrix<T, Eigen::Dynamic, 1> type;
    	};

    	//! @brief Type for Boost matrices used to reduce namespace cluttering.
    	template<class T=double>
    	struct BoostMatrix {
    	    //! type of this matrix
    	    typedef boost::numeric::ublas::matrix<T> type;
    	};

#ifdef RW_USE_UBLAS_LAPACK
    	template<class T=double>
    	struct BoostVector {
    	    //! type of this Vector
    	    typedef boost::numeric::ublas::vector<T> type;
    	};


        //! The standard Boost ublas matrix type.
        //typedef boost::numeric::ublas::matrix<double> Matrix;

        //! A boost vector of complex numbers.
        typedef boost::numeric::ublas::vector<std::complex<double> > BoostComplexVector;
#endif

        //! An Eigen vector of complex numbers.
		//typedef Eigen::VectorXcd ComplexVector;

#ifdef RW_USE_UBLAS_LAPACK
        /**
         * @brief Performs a singular value decomposition (SVD)
         *
         * The SVD computes the decomposition
         * \f$ \mathbf{M}=\mathbf{U}*\mathbf{DiagonalMatrix(\sigma)}*\mathbf{V}^T \f$ .
         *
         * @param M [in] the matrix to decomposite
         * @param U [out] Result matrix \f$\mathbf{U}\f$
         * @param sigma [out] The \f$\mathbf{sigma}\f$ vector with diagonal elements
         * @param V [out] Result matrix \f$\mathbf{V}\f$
         */
        static void svd(const BoostMatrix<double>::type& M, BoostMatrix<double>::type& U,
                        boost::numeric::ublas::vector<double>& sigma,
                        BoostMatrix<double>::type& V);
#endif
		/**
		 * @brief Performs a singular value decomposition (SVD)
		 *
         * The SVD computes the decomposition
         * \f$ \mathbf{M}=\mathbf{U}*\mathbf{DiagonalMatrix(\sigma)}*\mathbf{V}^T \f$ .
         *
         * @param M [in] the matrix to decomposite
         * @param U [out] Result matrix \f$\mathbf{U}\f$
         * @param sigma [out] The \f$\mathbf{sigma}\f$ vector with diagonal elements
         * @param V [out] Result matrix \f$\mathbf{V}\f$
		 */
		static void svd(const Eigen::MatrixXd& M, Eigen::MatrixXd& U, Eigen::VectorXd& sigma, Eigen::MatrixXd& V);


#ifdef RW_USE_UBLAS_LAPACK
        /**
         * \brief Calculates the moore-penrose (pseudo) inverse of a matrix
         * @f$ \mathbf{M}^+@f$
         *
         * \param am [in] the matrix @f$ \mathbf{M} @f$ to be inverted
         *
         * \param precision [in] the precision to use, values below this
         * treshold are considered singular
         *
         * \return the pseudo-inverse @f$ \mathbf{M}^+@f$ of @f$ \mathbf{M} @f$
         *
         * \f$ \mathbf{M}^+=\mathbf{V}\mathbf{\Sigma} ^+\mathbf{U}^T \f$ where
         * \f$ \mathbf{V} \f$, \f$ \mathbf{\Sigma} \f$ and \f$ \mathbf{U} \f$
         * are optained using Singular Value Decomposition (SVD)
         *
         * This method uses gesvd from LAPACK to perform SVD
         *
         */
        static BoostMatrix<double>::type pseudoInverseLapack(const BoostMatrix<double>::type& am, double precision=1e-6);

		
        /**
         * @brief Checks the penrose conditions
         * @param A [in] a matrix
         * @param X [in] a pseudoinverse of A
         * @param prec [in] the tolerance
         *
         * @return true if the pseudoinverse X of A fullfills the penrose
         * conditions, false otherwise
         *
         * Checks the penrose conditions:
         *
         * @f$
         * AXA = A
         * @f$
         *
         * @f$
         * XAX = X
         * @f$
         *
         * @f$
         * (AX)^T = AX
         * @f$
         *
         * @f$
         * (XA)^T = XA
         * @f$
         */
        static bool checkPenroseConditions(const BoostMatrix<double>::type& A,
                                           const BoostMatrix<double>::type& X,
                                           double prec = 1e-6);

#endif

		static BoostMatrix<double>::type pseudoInverse(const BoostMatrix<double>::type& am, double precision=1e-6);

		 /**
         * \brief Calculates the moore-penrose (pseudo) inverse of a matrix
         * @f$ \mathbf{M}^+@f$
         *
         * \param am [in] the matrix @f$ \mathbf{M} @f$ to be inverted
         *
         * \param precision [in] the precision to use, values below this
         * treshold are considered singular
         *
         * \return the pseudo-inverse @f$ \mathbf{M}^+@f$ of @f$ \mathbf{M} @f$
         *
         * \f$ \mathbf{M}^+=\mathbf{V}\mathbf{\Sigma} ^+\mathbf{U}^T \f$ where
         * \f$ \mathbf{V} \f$, \f$ \mathbf{\Sigma} \f$ and \f$ \mathbf{U} \f$
         * are optained using Singular Value Decomposition (SVD)
         *
         *
         */
		static Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& am, double precision=1e-6);



        /**
         * @brief Checks the penrose conditions
         * @param A [in] a matrix
         * @param X [in] a pseudoinverse of A
         * @param prec [in] the tolerance
         *
         * @return true if the pseudoinverse X of A fullfills the penrose
         * conditions, false otherwise
         *
         * Checks the penrose conditions:
         *
         * @f$
         * AXA = A
         * @f$
         *
         * @f$
         * XAX = X
         * @f$
         *
         * @f$
         * (AX)^T = AX
         * @f$
         *
         * @f$
         * (XA)^T = XA
         * @f$
         */
		static bool checkPenroseConditions(
			const Eigen::MatrixXd& A,
			const Eigen::MatrixXd& X,
			double prec);


#ifdef RW_USE_UBLAS_LAPACK
        /**
         * \brief Calculates matrix determinant
         * \param m [in] a square matrix
         * \return the matrix determinant
         */
        template<class R>
        static inline double det(const boost::numeric::ublas::matrix_expression<R>& m)
        {
            assert(m().size1() == m().size2());

            boost::numeric::ublas::permutation_matrix<std::size_t> pivots(m().size1());

            // create copy
            boost::numeric::ublas::matrix<typename R::value_type> mLu(m);

            lu_factorize(mLu, pivots);

            double det = 1.0;

            for (std::size_t i=0; i < pivots.size(); ++i) {
                if (pivots(i) != i)
                    det *= -1.0;
                det *= mLu(i,i);
            }
            return det;
        }
#endif

        /**
         * \brief Calculates matrix determinant
         * \param m [in] a square matrix
         * \return the matrix determinant
         */
		template <class R> 
		static inline double det(const Eigen::MatrixBase<R>& m) {
			return m.determinant();
		}

#ifdef RW_USE_UBLAS_LAPACK
        /**
         * @brief Calculates matrix inverse using lu_factorize and lu_substitute
         * @param M [in] input matrix @f$ \mathbf{M} @f$ to invert
         * @param Minv [out] output matrix @f$ \mathbf{M}^{-1} @f$
         **/
        template<class T>
        static void invertMatrix(const boost::numeric::ublas::matrix_expression<T>& M,
                                 boost::numeric::ublas::matrix<typename T::value_type>& Minv)
        {
            using namespace boost::numeric::ublas;
            typedef permutation_matrix<std::size_t> pmatrix;

            // create a working copy of the input
            matrix<typename T::value_type> A(M);
            // create a permutation matrix for the LU-factorization
            pmatrix pm(A.size1());

            // perform LU-factorization
            lu_factorize(A,pm);

            // create identity matrix of "inverse"
            Minv.assign(identity_matrix<typename T::value_type>(A.size1()));

            // backsubstitute to get the inverse
            lu_substitute(A, pm, Minv);
        }



		/**
		 * @brief Calculates matrix inverse using lu_factorize and lu_substitute
         * @param M [in] input matrix @f$ \mathbf{M} @f$ to invert
         * @return output matrix @f$ \mathbf{M}^{-1} @f$
         **/
        template<class T>
        static boost::numeric::ublas::matrix<typename T::value_type> inverse(const boost::numeric::ublas::matrix_expression<T>& M)
        {
            using namespace boost::numeric::ublas;
            typedef permutation_matrix<std::size_t> pmatrix;

            // create a working copy of the input
            matrix<typename T::value_type> A(M);
            // create a permutation matrix for the LU-factorization
            pmatrix pm(A.size1());

            // perform LU-factorization
            lu_factorize(A,pm);

            // create identity matrix of "inverse"
            boost::numeric::ublas::matrix<typename T::value_type> Minv( A.size2(), A.size1() );

			Minv.assign(identity_matrix<typename T::value_type>(A.size1()));

            // backsubstitute to get the inverse
            lu_substitute(A, pm, Minv);
			return Minv;
        }

#endif
		/**
		 * @brief Calculates matrix inverse.
         * @param M [in] input matrix @f$ \mathbf{M} @f$ to invert
         * @return output matrix @f$ \mathbf{M}^{-1} @f$
         **/
        template<class T>
		static T inverse(const Eigen::MatrixBase<T>& M)
        {
			return M.inverse();
        }


#ifdef RW_USE_UBLAS_LAPACK
        /**
         * @brief Checks if a given matrix is in SO(n) (special orthogonal)
         * @param M [in] \f$ \mathbf{M} \f$
         * @return true if \f$ M\in SO(n) \f$
         *
         * \f$ SO(n) = {\mathbf{R}\in \mathbb{R}^{n\times n} :
         * \mathbf{R}\mathbf{R}^T=\mathbf{I}, det \mathbf{R}=+1} \f$
         *
         */
        template<class R>
        static inline bool isSO(const boost::numeric::ublas::matrix_expression<R>& M)
        {
            return isProperOrthonormal(M) && M().size1() == M().size2();
        }

#endif

        /**
         * @brief Checks if a given matrix is in SO(n) (special orthogonal)
         * @param M [in] \f$ \mathbf{M} \f$
         * @return true if \f$ M\in SO(n) \f$
         *
         * \f$ SO(n) = {\mathbf{R}\in \mathbb{R}^{n\times n} :
         * \mathbf{R}\mathbf{R}^T=\mathbf{I}, det \mathbf{R}=+1} \f$
         *
         */
        template<class R>
        static inline bool isSO(const Eigen::MatrixBase<R>& M)
        {
            return M.cols() == M.rows() && isProperOrthonormal(M);
        }

        /**
         * @brief Checks if a given matrix is in SO(n) (special orthogonal)
         * @param M [in] \f$ \mathbf{M} \f$
         * @param precision [in] the precision to use for floating point comparison
         * @return true if \f$ M\in SO(n) \f$
         *
         * \f$ SO(n) = {\mathbf{R}\in \mathbb{R}^{n\times n} :
         * \mathbf{R}\mathbf{R}^T=\mathbf{I}, det \mathbf{R}=+1} \f$
         *
         */
        template<class R>
        static inline bool isSO(const Eigen::MatrixBase<R>& M, typename R::Scalar precision)
        {
            return M.cols() == M.rows() && isProperOrthonormal(M, precision);
        }

        /**
         * @brief Checks if a given matrix is in so(n)
         * @param M [in] \f$ \mathbf{M} \f$
         * @return true if \f$ M\in so(n) \f$
         *
         \f$ so(n) = {\mathbf{S}\in \mathbb{R}^{n\times n}:\mathbf{S}^T =
         -\mathbf{S}} \f$
         */
        //template<class R>
        //static inline bool isso(const boost::numeric::ublas::matrix_expression<R>& M)
        //{
        //    return IsSkewSymmetric(M) && M().size1() == M().size2();
        //}

        /**
         * @brief Checks if a given matrix is skew-symmetrical
         * @param M [in] \f$ \mathbf{M} \f$ the matrix to check
         *
         * @return true if the property
         * \f$ \mathbf{M}=-\mathbf{M}^T \f$ holds,
         * false otherwise.
         */
        template<class R>
        static inline bool isSkewSymmetric(const boost::numeric::ublas::matrix_expression<R>& M)
        {
            return norm_inf(M+trans(M)) == 0.0;
        }

        /**
         * @brief Checks if a given matrix is skew-symmetrical
         * @param M [in] \f$ \mathbf{M} \f$ the matrix to check
         *
         * @return true if the property
         * \f$ \mathbf{M}=-\mathbf{M}^T \f$ holds,
         * false otherwise.
         */
        template<class R>
		static inline bool isSkewSymmetric(const Eigen::MatrixBase<R>& M)
        {
			return (M+M.transpose()).template lpNorm<Eigen::Infinity>() == 0.0;
        }


#ifdef RW_USE_UBLAS_LAPACK
        /**
         * @brief Checks if a given matrix is proper orthonormal
         * @return true if the matrix is proper orthonormal, false otherwise
         *
         * A matrix is proper orthonormal if it is orthonormal and its determinant
         * is equal to \f$ +1 \f$
         */
        template<class R>
        static inline bool isProperOrthonormal(const boost::numeric::ublas::matrix_expression<R>& r)
        {
            return isOrthonormal(r) && det(r) == 1.0;
        }
#endif //RW_USE_UBLAS_LAPACK

        /**
         * @brief Checks if a given matrix is proper orthonormal
         * @return true if the matrix is proper orthonormal, false otherwise
         *
         * A matrix is proper orthonormal if it is orthonormal and its determinant
         * is equal to \f$ +1 \f$
         */
        template<class R>
        static inline bool isProperOrthonormal(const Eigen::MatrixBase<R>& r, typename R::Scalar precision = std::numeric_limits<typename R::Scalar>::epsilon())
        {
            return isOrthonormal(r, precision) && fabs(r.determinant() - 1.0) <= precision;
        }

        /**
         * @brief Checks if a given matrix is orthonormal
         * @return true if the matrix is orthonormal, false otherwise
         *
         * A matrix is orthonormal if all of it's column's are mutually orthogonal
         * and all of it's column's has unit length.
         *
         * that is for any \f$ i, j \f$ the following holds
         * \f$ col_i . col_j = 0 \f$ and \f$ ||col_i|| = 1 \f$
         *
         * Another nessesary and sufficient condition of orthonormal matrices is that
         * \f$ \mathbf{M}\mathbf{M}^T=I \f$
         */
        template<class R>
        static inline bool isOrthonormal(const boost::numeric::ublas::matrix_expression<R>& r)
        {
            return norm_inf(
                prod(r, trans(r)) -
                boost::numeric::ublas::identity_matrix<typename R::value_type>(
                    r().size2(), r().size2())
                ) == 0.0;
        }

        /**
         * @brief Checks if a given matrix is orthonormal
         * @return true if the matrix is orthonormal, false otherwise
         *
         * A matrix is orthonormal if all of it's column's are mutually orthogonal
         * and all of it's column's has unit length.
         *
         * that is for any \f$ i, j \f$ the following holds
         * \f$ col_i . col_j = 0 \f$ and \f$ ||col_i|| = 1 \f$
         *
         * Another nessesary and sufficient condition of orthonormal matrices is that
         * \f$ \mathbf{M}\mathbf{M}^T=I \f$
         */
        template<class R>
        static inline bool isOrthonormal(const Eigen::MatrixBase<R>& r, typename R::Scalar precision = std::numeric_limits<typename R::Scalar>::epsilon())
        {			
			return (r * r.transpose()).isIdentity(precision);
			//const Eigen::MatrixBase<R> m = r*r.transpose() ;
			//return m.isIdentity(1e-15);
			//double scale = m.norm();//m.lpNorm<Eigen::Infinity>();
			//return scale == 0.0;
        }

#ifdef RW_USE_UBLAS_LAPACK
        /**
         * @brief Computes the eigenvalue decomposition of a symmetric matrix
         *
         * Given a symmetric matrix \f$ \mathbf{A} \in \mathbb{R}^n \f$ the
         * eigenvalue decomposition giving \f$\lambda_i, \mathbf{x}_i \f$ such
         * that \f$\lambda_i \mathbf{x}_i=\mathbf{A}\mathbf{x}_i \f$.
         *
         * @param Am [in] the matrix \f$\mathbf{A}\f$
         *
         * @return std::pair in which the eigenvectors and eigenvectors are
         * stored as columns in the matrix and elements in the vector
         * respectively.
         */
        template<class T, class L, class A>
        static std::pair<typename BoostMatrix<T>::type, typename BoostVector<T>::type >
			eigenDecompositionSymmetric(const boost::numeric::ublas::matrix<T,L,A>& Am)
        {
            typedef boost::numeric::ublas::matrix<T,
                    boost::numeric::ublas::column_major> TColumnMatrix;
            typedef boost::numeric::ublas::zero_vector<T> TZeroVector;
            typedef boost::numeric::ublas::vector<T> TVector;
            typedef boost::numeric::ublas::matrix<T> TMatrix;

            using namespace boost::numeric::bindings::lapack;

            assert(Am.size1() == Am.size2());
            size_t n = Am.size1();
            TColumnMatrix Ac = Am;

            TZeroVector Wz(n);
            TVector Wc(Wz);

            syev<TColumnMatrix, TVector> ('V', 'U', Ac, Wc,
                                          optimal_workspace());

            return std::make_pair(TMatrix(Ac), Wc);
        }

		        /**
         * @brief Computes the eigenvalue decomposition
         *
         * Given a matrix \f$ \mathbf{A} \in \mathbb{R}^n \f$ the eigenvalue
         * decomposition giving \f$\lambda_i, \mathbf{x}_i \f$ such that
         * \f$\lambda_i \mathbf{x}_i=\mathbf{A}\mathbf{x}_i \f$.
         *
         * @param Am [in] the matrix \f$\mathbf{A}\f$
         *
         * @return std::pair in which the eigenvectors and eigenvectors are
         * stored as columns in the matrix and elements in the vector
         * respectively.
         */
		template<class T, class L, class A>
        static std::pair<typename BoostMatrix<T>::type, BoostComplexVector>
			eigenDecomposition(const boost::numeric::ublas::matrix<T,L,A>& Am)
        {
            typedef boost::numeric::ublas::matrix<
                double,
                boost::numeric::ublas::column_major>
                ColumnMatrix;

            using namespace boost::numeric::bindings::lapack;

            assert(Am.size1() == Am.size2());

            size_t n = Am.size1();
            ColumnMatrix Ac(Am);

            optimal_workspace workspace;

            BoostComplexVector Wc(n);

            ColumnMatrix Vr(n,n);
            ColumnMatrix Vl(n,n);

            geev<ColumnMatrix, BoostComplexVector, ColumnMatrix>(
                Ac, Wc, &Vl, &Vr, workspace);

            //std::cout<<"Wc = "<<Wc<<std::endl;
            //std::cout<<"A = "<<Ac<<std::endl;
            //std::cout<<"Vl = "<<Vl<<std::endl;
            //std::cout<<"Vr = "<<Vr<<std::endl;

            return std::make_pair(typename BoostMatrix<T>::type(Vr), Wc);
        }




#endif //RW_USE_UBLAS_LAPACK

		/**
		 * @brief Decomposition for a symmetric matrix.
		 * @param Am1 [in] a symmetric matrix.
		 * @return the decomposition as a pair with eigenvectors and eigenvalues.
		 */
		template<class T>
		//static std::pair<typename EigenMatrix<T>::type, typename EigenVector<T>::type > eigenDecompositionSymmetric(const typename EigenMatrix<T>::type& Am1)
		static std::pair<typename EigenMatrix<T>::type, typename EigenVector<T>::type > eigenDecompositionSymmetric(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& Am1)
        {
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > eigenSolver;
			eigenSolver.compute(Am1);
			return std::make_pair(eigenSolver.eigenvectors(), eigenSolver.eigenvalues());
		}

		/**
		 * @brief Eigen decomposition of a matrix.
		 * @param Am1 [in] the matrix.
		 * @return the decomposition as a pair with eigenvectors and eigenvalues.
		 */
		template<class T>
		static std::pair<typename EigenMatrix<std::complex<T> >::type, typename EigenVector<std::complex<T> >::type > eigenDecomposition(const typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& Am1)
        {
			Eigen::EigenSolver<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > eigenSolver;
			eigenSolver.compute(Am1);

			Eigen::Matrix<std::complex<T>, Eigen::Dynamic, Eigen::Dynamic> vectors = eigenSolver.eigenvectors();
			Eigen::Matrix<std::complex<T>, Eigen::Dynamic, 1> values = eigenSolver.eigenvalues();
			return std::make_pair(vectors, values);
        }







#ifdef RW_USE_UBLAS_LAPACK

	    /**
	     * @brief computes the solution to a real system of linear equations
	     *  A*X = B, where A is an N-by-N symmetric positive definite tridiagonal
	     *  matrix, and X and B are N-by-1 matrices.
	     *
	     *  A is factored as A = L*D*L**T, and the factored form of A is then
	     *  used to solve the system of equations.
	     * @param d [in]  On entry, the n diagonal elements of the tridiagonal matrix
	     *          A.  On exit, the n diagonal elements of the diagonal matrix
	     *          D from the factorization A = L*D*L**T.
	     *
	     * @param e [in] E (input/output) REAL array, dimension (N-1)
	     *          On entry, the (n-1) subdiagonal elements of the tridiagonal
	     *          matrix A.  On exit, the (n-1) subdiagonal elements of the
	     *          unit bidiagonal factor L from the L*D*L**T factorization of
	     *          A.  (E can also be regarded as the superdiagonal of the unit
	     *          bidiagonal factor U from the U**T*D*U factorization of A.)
	     * @param b [in]   On entry, the N-by-1 right hand side matrix B.
	     *          On exit, if success, the N-by-1 solution matrix X.
	     * @return true if successfull, false otherwise
	     */
	    template<class T>
	    static bool triDiagonalSolve(boost::numeric::ublas::vector<T>& d,
	                                 boost::numeric::ublas::vector<T>& e,
	                                 boost::numeric::ublas::vector<T>& b)
	    {
	        RW_ASSERT(d.size()==b.size());
	        RW_ASSERT(d.size()==e.size()+1);


	        // should perhaps use boost::numeric::bindings::lapack::ptsv(d,e,b) but that require d as matrix

	        int N = d.size();
	        int LDB = N;
	        int info = 0;
	        int NRHS = 1;

	        lapack_triDiagonalSolve(&LDB, &NRHS, &d[0], &e[0], &b[0], &LDB, &info);
	        if(info==0)
	            return true;
	        return false;
	    }
#endif //#ifdef RW_USE_UBLAS_LAPACK

    private:

#ifdef RW_USE_UBLAS_LAPACK
	    /**
	     * @brief lapack wrap function - computes the solution to a real system of linear equations
	     *  A*X = B, where A is an N-by-N symmetric positive definite tridiagonal
	     *  matrix, and X and B are N-by-NRHS matrices.
	     *
	     *  A is factored as A = L*D*L**T, and the factored form of A is then
	     *  used to solve the system of equations.
	     * @param n [in] the order of the matrix A
	     * @param nrhs [in] The number of right hand sides, i.e., the number of columns
	     * of the matrix B.  NRHS >= 0.
	     * @param d [in]  On entry, the n diagonal elements of the tridiagonal matrix
	     *          A.  On exit, the n diagonal elements of the diagonal matrix
	     *          D from the factorization A = L*D*L**T.
	     *
	     * @param e [in] E (input/output) REAL array, dimension (N-1)
	     *          On entry, the (n-1) subdiagonal elements of the tridiagonal
	     *          matrix A.  On exit, the (n-1) subdiagonal elements of the
	     *          unit bidiagonal factor L from the L*D*L**T factorization of
	     *          A.  (E can also be regarded as the superdiagonal of the unit
	     *          bidiagonal factor U from the U**T*D*U factorization of A.)
	     * @param b [in]   On entry, the N-by-NRHS right hand side matrix B.
	     *          On exit, if INFO = 0, the N-by-NRHS solution matrix X.
	     * @param ldb [in] The leading dimension of the array B.  LDB >= max(1,N).
	     * @param info
	     */
	    static void lapack_triDiagonalSolve(int *N, int *NRHS, float *D, float *e, float *b, int *ldb, int *info);
	    static void lapack_triDiagonalSolve(int *N, int *NRHS, double *D, double *e, double *b, int *ldb, int *info);
#endif //RW_USE_UBLAS_LAPACK
    };

	template<>
	std::pair<typename LinearAlgebra::EigenMatrix<double>::type, typename LinearAlgebra::EigenVector<double>::type > LinearAlgebra::eigenDecompositionSymmetric<double>(const Eigen::MatrixXd& Am1);

	template<>
	std::pair<typename LinearAlgebra::EigenMatrix<std::complex<double> >::type, typename LinearAlgebra::EigenVector<std::complex<double> >::type > LinearAlgebra::eigenDecomposition<double>(const Eigen::MatrixXd& Am1);

    /*@}*/
}} // end namespaces

#endif // end include guard
