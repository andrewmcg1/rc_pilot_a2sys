/**
 * <math_utils.h>
 *
 * @brief   Supplement to rc/math/matrix.h and rc/math/vector.h
 *
 * @author Glen Haggin (ghaggin@umich.edu)
 *
 * @addtogroup MathUtils
 * @{
 */

#ifndef __MATH_UTILS__
#define __MATH_UTILS__

#include <rc/math/matrix.h>
#include <rc/math/vector.h>

/**
 *  @brief  Matrix left multiplied to collumn vector Ax = b
 *
 *  @param[in]  A   matrix
 *  @param[in]  x   vector
 *  @param[out] b   vector
 */
void matrix_times_vector(rc_matrix_t A, rc_vector_t x, rc_vector_t* b);

/**
 *  @brief  Matrix left multiplied to collumn vector Ax = b
 *
 *  @param[in]  A   matrix
 *  @param[in]  x   vector
 *  @param[out] x   vector
 */
void matrix_times_vector_inplace(rc_matrix_t A, rc_vector_t* x);

/**
 * @brief   Function pointer for dynamics function xd = f(t,x,u)
 *
 * dyn_fn(t, x, u, ret_vect)
 */
typedef void (*__dyn_fn_t)(double, rc_vector_t, rc_vector_t, rc_vector_t*);

/**
 *  @brief  Numerical jacobian of nonlinear dynamical function xd = f(t,x,u)
 *
 *  @param[in]  fn  dynamic function xd = f(t,x,u)
 *  @param[in]  t0  equilibrium time for f
 *  @param[in]  x0  equilibrium state vector for f
 *  @param[in]  u0  equilibrium inpout vector for f
 *  @param[out] J   Jacobian matrix
 */
void numjac(__dyn_fn_t fn, double t0, rc_vector_t x0, rc_vector_t u0, rc_matrix_t* J);

#endif /*__MATH_UTILS__ */

/* @} end group MathUtils */