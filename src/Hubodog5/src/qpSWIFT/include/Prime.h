#ifndef __PRIME_H__
#define __PRIME_H__

#include "timer.h"
#include "../amd/include/amd.h"
#include "../amd/include/amd_internal.h"



//#include "qpSWIFT/include/GlobalOptions.h"
//#include "qpSWIFT/include/Matrices.h"
//#include "qpSWIFT/include/timer.h"
//#include "qpSWIFT/include/ldl.h"
//#include "qpSWIFT/amd/include/amd.h"
//#include "qpSWIFT/amd/include/amd_internal.h"
//#include "qpSWIFT/amd/include/SuiteSparse_config.h"






#ifdef __cplusplus
extern "C"{
#endif

/*! Sparse Matrix Storage Format */
/*! Use Sparse Matrix Setup to Initialise a Sparse Matrix */
typedef struct smat{
	qp_int* jc;						/*!< Vector to store column count ; Dim [n+1] */
	qp_int* ir;						/*!< Vector to store row indices in column major format ; Dim[nnz] */
	qp_real* pr;					/*!< Vector to store matrix values in column major format ; Dim[nnz] */
        qp_int m;						/*!< Number of Rows of the Sparse Matrix */
        qp_int n;						/*!< Number of Columns of the Sparse Matrix */
	qp_int nnz;						/*!< Number of nonzeros entries ; nnz = jc [n] */
}smat;



typedef struct kkt_type{
	smat *kktmatrix;				/*!< Sparse kkt matrix */
	qp_real *b;						/*!< b vector */
	qp_int *Parent;					/*!< LDL - workspace Vectors */
	qp_int *Flag;					/*!< LDL - workspace Vectors */
	qp_int *Lnz;					/*!< LDL - workspace Vectors */
	qp_int *Li;						/*!< ir vector of LDL Sparse Matrix in column compressed format */
	qp_int *Lp;						/*!< jc vector of LDL Sparse Matrix in column compressed format */
	qp_int *Lti;            		/*!< ir vector of the transpose of LDL Sparse Matrix in column compressed format */
	qp_int *Ltp;            		/*!< jc vector of the transpose of LDL Sparse Matrix in column compressed format */
	qp_int *Pattern;				/*!< LDL - workspace Vectors */
	qp_int *UPattern;				/*!< Nodes to be updated during every iteration */
	qp_real *Y;						/*!< LDL - workspace Vectors */
	qp_real *Lx;					/*!< pr vector of LDL Sparse Matrix in column compressed format */
	qp_real *D;						/*!< LDL - workspace Vectors */
	qp_int *P;						/*!< Permutation Vector ; Input */
	qp_int *Pinv;				    /*!< Permutation Vector Inverse */

}kkt_type;

typedef struct stats_type{

	/*!< Time Statistics */
	qp_real tsetup;					/*!< Setup Time ; Includes Initialisation Problem as well */
	qp_real tsolve;					/*!< Solve Time */
	qp_real kkt_time;				/*!< kkt Solve Time */
	qp_real ldl_numeric;			/*!< ldl_numeric time */
	/*!< Time Statsitics */

	/*!< Algorithmic Statistics */
	qp_int  IterationCount;			/*!< Iteration Count */
	qp_real n_rx;					/*!< Norm of residual vector rx */
	qp_real n_ry;					/*!< Norm of residual vector ry */
	qp_real n_rz;					/*!< Norm of residual vector rz */
	qp_real n_mu;					/*!< Complementary Slackness (s'z/m) */

	qp_real alpha_p;			    /*!< Primal Step Size */
	qp_real alpha_d;			    /*!< Dual Step Size  */
	/*!< Algorithmic Statistics */

	qp_real fval;					/*!< Function Value */
	qp_int Flag;					/*!< Solver FLAG */
    qp_int AMD_RESULT;          	/*!< AMD Compilation Result ; Non-negative means Successfull ; Negative means unsuccesfull ; -3 is unused */
} stats_type;

typedef struct settings{

	qp_int maxit;					/*!< Maximum Number of Iterations */
	qp_real reltol;					/*!< Residual Tolerances */
	qp_real abstol;					/*!< s and z Tolerances */
	qp_real sigma;					/*!< Sigma Desired */
	qp_int verbose;					/*!< Verbose Levels || 0 :: No Print
								     *                  || 1 :: Print Residuals and Step Sizes
									 * 					|| 2 :: Print Everything */

}settings;

typedef struct QPswift{

	qp_int n;						/*!< Dimension of P matrix */
	qp_int m;						/*!< First Dimension of G matrix */
	qp_int p;						/*!< First Dimension of A matrix */

	qp_real sigma_d;				/*!< Parameter */
	qp_real mu;						/*!< Barrier Function Parameter */
	qp_real rho;					/*!< Some Parameter */


	qp_real *x;						/*!<	Primal Variables ;  Dimensions [n,1] */
	qp_real *y ;			    			/*!<    Dual   Variables ;  Dimensions [p,1] */
	qp_real *z;						/*!<	Dual Variables	 ;	Dimensions [m,1] */
	qp_real *s;						/*!<	Primal Variables ;	Dimensions [m,1] */


	qp_real *rx;					/*!<	Residuals	;	Dimensions [n,1] */
	qp_real *ry;					/*!<    Residuals	;	Dimensions [p,1] */
	qp_real *rz;					/*!<	Residuals	;	Dimensions [m,1] */

	qp_real *delta;					/*!< [delta_x;delta_y;delta_z]	;	Dimensions [n + p + m,1] */
	qp_real *delta_x;				/*!< delta_x	;	Dimensions [n,1] */
	qp_real *delta_y ;				/*!< delta_y	;	Dimensions [p,1] */
	qp_real *delta_z;				/*!< delta_z	;	Dimensions [m,1] */
	qp_real *delta_s;				/*!< delta_s	;	Dimensions [m,1] */

	qp_real *ds;					/*!< ds	;	Dimensions [m,1] */
	qp_real *lambda;				/*!< lambda	;	Dimensions[m,1] */

	qp_real *temp;					/*!< Temporary Variables to Calculate Objective Function Value */

	smat *P;						/*!< Cost Function	:	Quadratic Part	:	Dimensions	[n,n] */
	qp_real *c;						/*!< Cost Function	:	linear term	:	Dimensions	[n,1] */
	smat *G;						/*!< Inequality Matrix	:	Gx<=h	:	Dimension	[m,n] */
	qp_real *h;						/*!< Inequality Matrix	:	Gx<=h	:	Dimension	[m,1] */
	smat *A ;						/*!< Equality Matrix		:	Ax=b	:	Dimension	[p,n] */
	qp_real *b ;					/*!< Equality Matrix		:	Ax=b	:	Dimension	[p,1] */

    	smat *At;     	  		      	   /*!< Transpose of Equality Matrix */
    	smat *Gt;     	            		   /*!< Transpose of InEquality Matrix */

    kkt_type *kkt;					   /*!< kkt Matrix */
	settings *options;			   /*!< Solver Settings */
    stats_type *stats;	               /*!< Solver Stats */



} QPswift;

/*!< Main Solver Functions */
/*!< Written in Prime.c */
/*!< QP Setup Function */



//QPswift* QP_SETUP(qp_int n, qp_int m, qp_int p, qp_int *Pjc, qp_int *Pir, qp_real *Ppr, qp_int*Ajc, qp_int *Air, qp_real *Apr, qp_int *Gjc, qp_int *Gir, qp_real *Gpr, qp_real *c, qp_real *h, qp_real *b, qp_real sigma_d, qp_int *Permut);
QPswift* QP_SETUP(qp_int n, qp_int m, qp_int p, qp_int *Pjc, qp_int *Pir, qp_real *Ppr, qp_int*Ajc, qp_int *Air, qp_real *Apr, qp_int *Gjc, qp_int *Gir, qp_real *Gpr, qp_real *c, qp_real *h, qp_real *b, qp_real sigma_d, qp_int *Permut);
/*!< QP Solve Function */
qp_int QP_SOLVE(QPswift *myQP);

/*!< QP Memory Clean Function */
void QP_CLEANUP(QPswift *myQP);



/*!< Auxillary Functions */
/*!< Written in Auxilary.c */

qp_int kkt_initialize(QPswift* myQP);

qp_int kktsolve_1(QPswift* myQP);

void kktsolve_2(QPswift* myQP);

void SparseMatrixSetup(qp_int m, qp_int n, qp_int nnz, qp_int* jc, qp_int* ir, qp_real* pr, smat* sparse);

void Transpose_Row_Count(qp_int m, qp_int n, qp_int *Li, qp_int *Lp, qp_int *Lti, qp_int *Ltp);

void computeresiduals(QPswift* myQP);

void SparseMatrixMultiply(smat *A, qp_real *x, qp_real *y, qp_int start);

void SparseMatrixTransMultiply(smat *A, qp_real* x, qp_real* y, qp_int start);

void form_ds(qp_real* ds, qp_real *lambda, qp_real *delta_s, qp_real *delta_z, qp_real sigma, qp_real mu, qp_int m, qp_int selector);

void formkktmatrix_U(smat* P, smat* G, smat* Gt, smat* kkt);

void formkktmatrix_full(smat *P, smat *G, smat *A, smat *Gt, smat *At, smat *kktmatrix);

void SparseMatrixTranspose(smat* A,smat *At);

void updatekktmatrix(smat *kkt, qp_real *s, qp_real*z, qp_real*delta_s, qp_real* delta_z, qp_real alpha_p, qp_real alpha_d, qp_int m, qp_int n, qp_int p, qp_int indicator);

void updatekktmatrix_b(qp_real *b, qp_real *rx, qp_real *ry, qp_real *rz, qp_real *ds, qp_real *z, qp_int n, qp_int m, qp_int p);

qp_int checksign(qp_real*s, qp_real *delta_s, qp_real alpha, qp_int count);

void updatevariables(qp_real *x, qp_real *delta_x, qp_real alpha, qp_int count);

void formlambda(qp_real *lambda, qp_real *s, qp_real*z, qp_int n);

qp_real formrho(qp_real *s, qp_real*delta_s, qp_real*z, qp_real*delta_z, qp_real alpha_p, qp_real alpha_d, qp_int n);

qp_int ldlinitialsolve(kkt_type* mykkt, qp_real*delta);

qp_int ldlitersolve(kkt_type* mykkt, qp_real*delta);

void test_reach(qp_int *Parent, qp_int *Pinv, qp_int *UPattern, qp_int n, qp_int m, qp_int p);

void findsteplength(qp_real* s, qp_real* delta_s, qp_real* z, qp_real* delta_z, qp_int m, qp_real* alpha_p, qp_real* alpha_d);

qp_real obj_value(smat *P, qp_real *c, qp_real *x,qp_real *temp);

qp_real innerproduct(qp_real* x, qp_real*y, qp_int n);

void densetosparse(qp_real*m,qp_real*n,qp_real *pr,smat *A);
#ifdef __cplusplus
}
#endif
#endif
