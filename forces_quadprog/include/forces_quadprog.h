/*
Header file containing definitions for C interface of forces_quadprog,
 a fast costumized optimization solver.
*/

#ifndef forces_quadprog_H
#define forces_quadprog_H

#include <stdio.h>

/* For Visual Studio 2015 Compatibility */
#if (_MSC_VER >= 1900)
FILE * __cdecl __iob_func(void);
#endif
/* DATA TYPE ------------------------------------------------------------*/
typedef double forces_quadprog_float;

#ifndef SOLVER_STANDARD_TYPES
#define SOLVER_STANDARD_TYPES

typedef signed char solver_int8_signed;
typedef unsigned char solver_int8_unsigned;
typedef char solver_int8_default;
typedef signed short int solver_int16_signed;
typedef unsigned short int solver_int16_unsigned;
typedef short int solver_int16_default;
typedef signed int solver_int32_signed;
typedef unsigned int solver_int32_unsigned;
typedef int solver_int32_default;
typedef signed long long int solver_int64_signed;
typedef unsigned long long int solver_int64_unsigned;
typedef long long int solver_int64_default;

#endif

/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef SET_PRINTLEVEL_forces_quadprog
#define SET_PRINTLEVEL_forces_quadprog    (0)
#endif

/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct
{
	/* column vector of length 3 */
	forces_quadprog_float x0[3];

} forces_quadprog_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct
{
	/* column vector of length 2 */
	forces_quadprog_float output1[2];

} forces_quadprog_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct
{
	/* iteration number */
	solver_int32_default it;

	/* number of iterations needed to optimality (branch-and-bound) */
	solver_int32_default it2opt;

	/* inf-norm of equality constraint residuals */
	forces_quadprog_float res_eq;

	/* inf-norm of inequality constraint residuals */
	forces_quadprog_float res_ineq;

	/* primal objective */
	forces_quadprog_float pobj;

	/* dual objective */
	forces_quadprog_float dobj;

	/* duality gap := pobj - dobj */
	forces_quadprog_float dgap;

	/* relative duality gap := |dgap / pobj | */
	forces_quadprog_float rdgap;

	/* duality measure */
	forces_quadprog_float mu;

	/* duality measure (after affine step) */
	forces_quadprog_float mu_aff;

	/* centering parameter */
	forces_quadprog_float sigma;

	/* number of backtracking line search steps (affine direction) */
	solver_int32_default lsit_aff;

	/* number of backtracking line search steps (combined direction) */
	solver_int32_default lsit_cc;

	/* step size (affine direction) */
	forces_quadprog_float step_aff;

	/* step size (combined direction) */
	forces_quadprog_float step_cc;

	/* solvertime */
	forces_quadprog_float solvetime;

} forces_quadprog_info;


/* SOLVER FUNCTION DEFINITION -------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

/* examine exitflag before using the result! */
extern solver_int32_default forces_quadprog_solve(forces_quadprog_params *params, forces_quadprog_output *output, forces_quadprog_info *info, FILE *fs);

#ifdef __cplusplus
}
#endif

#endif /* forces_quadprog_H */
