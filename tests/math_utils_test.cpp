#include <boost/test/unit_test.hpp>

extern "C"
{
#include <math.h>

#include <rc/math/algebra.h>

#include <math_utils.h>
}

BOOST_AUTO_TEST_SUITE(Test_Math_Utils)

BOOST_AUTO_TEST_CASE(matrix_mult)
{
    rc_matrix_t A = RC_MATRIX_INITIALIZER;
    BOOST_CHECK(rc_matrix_alloc(&A, 2, 2) > -1);
    A.d[0][0] = 1;
    A.d[0][1] = 2;
    A.d[1][0] = 3;
    A.d[1][1] = 4;

    rc_matrix_t B = RC_MATRIX_INITIALIZER;
    BOOST_CHECK(rc_matrix_alloc(&B, 2, 2) > -1);
    B.d[0][0] = 5;
    B.d[0][1] = 6;
    B.d[1][0] = 7;
    B.d[1][1] = 8;

    rc_matrix_t C = RC_MATRIX_INITIALIZER;
    BOOST_CHECK(rc_matrix_alloc(&C, 2, 2) > -1);
    rc_matrix_multiply(A, B, &C);
    BOOST_CHECK_EQUAL(C.d[0][0], 19);
    BOOST_CHECK_EQUAL(C.d[0][1], 22);
    BOOST_CHECK_EQUAL(C.d[1][0], 43);
    BOOST_CHECK_EQUAL(C.d[1][1], 50);
}

BOOST_AUTO_TEST_CASE(vect_mult)
{
    rc_matrix_t A = RC_MATRIX_INITIALIZER;
    BOOST_CHECK(rc_matrix_alloc(&A, 2, 2) > -1);
    A.d[0][0] = 1;
    A.d[0][1] = 2;
    A.d[1][0] = 3;
    A.d[1][1] = 4;

    rc_vector_t x = RC_VECTOR_INITIALIZER;
    BOOST_CHECK(rc_vector_alloc(&x, 2) > -1);
    x.d[0] = 1;
    x.d[1] = 2;

    rc_vector_t b = RC_VECTOR_INITIALIZER;
    BOOST_CHECK(rc_vector_alloc(&b, 2) > -1);
    matrix_times_vector(A, x, &b);

    BOOST_CHECK_EQUAL(b.d[0], 5);
    BOOST_CHECK_EQUAL(b.d[1], 11);
}

BOOST_AUTO_TEST_CASE(vect_mult_inplace)
{
    rc_matrix_t A = RC_MATRIX_INITIALIZER;
    BOOST_CHECK(rc_matrix_alloc(&A, 2, 2) > -1);
    A.d[0][0] = 1;
    A.d[0][1] = 2;
    A.d[1][0] = 3;
    A.d[1][1] = 4;

    rc_vector_t x = RC_VECTOR_INITIALIZER;
    BOOST_CHECK(rc_vector_alloc(&x, 2) > -1);
    x.d[0] = 1;
    x.d[1] = 2;

    matrix_times_vector_inplace(A, &x);

    BOOST_CHECK_EQUAL(x.d[0], 5);
    BOOST_CHECK_EQUAL(x.d[1], 11);
}

BOOST_AUTO_TEST_CASE(invert_matrix)
{
    rc_matrix_t A = RC_MATRIX_INITIALIZER;
    BOOST_CHECK(rc_matrix_alloc(&A, 2, 2) > -1);
    A.d[0][0] = 1;
    A.d[0][1] = 2;
    A.d[1][0] = 3;
    A.d[1][1] = 4;

    rc_matrix_t B = RC_MATRIX_INITIALIZER;
    BOOST_CHECK(rc_matrix_alloc(&B, 2, 2) > -1);

    rc_algebra_invert_matrix(A, &B);
    double tol = 0.0001;
    BOOST_CHECK_CLOSE(B.d[0][0], -2.00, tol);
    BOOST_CHECK_CLOSE(B.d[0][1], 1.00, tol);
    BOOST_CHECK_CLOSE(B.d[1][0], 1.50, tol);
    BOOST_CHECK_CLOSE(B.d[1][1], -0.50, tol);
}

BOOST_AUTO_TEST_CASE(cross_product)
{
    rc_vector_t a = RC_VECTOR_INITIALIZER;
    rc_vector_t b = RC_VECTOR_INITIALIZER;
    rc_vector_t c = RC_VECTOR_INITIALIZER;
    BOOST_CHECK(rc_vector_alloc(&a, 3) > -1);
    BOOST_CHECK(rc_vector_alloc(&b, 3) > -1);
    BOOST_CHECK(rc_vector_alloc(&c, 3) > -1);

    a.d[0] = 1;
    a.d[1] = 2;
    a.d[2] = 3;

    b.d[0] = 4;
    b.d[1] = 5;
    b.d[2] = 6;

    rc_vector_cross_product(a, b, &c);
    double tol = 0.00001;
    BOOST_CHECK_CLOSE(c.d[0], -3, tol);
    BOOST_CHECK_CLOSE(c.d[1], 6, tol);
    BOOST_CHECK_CLOSE(c.d[2], -3, tol);
}

void dynamics(double t, rc_vector_t x, rc_vector_t u, rc_vector_t* ret)
{
    const double x1 = x.d[0];
    const double x2 = x.d[1];

    ret->d[0] = x1 * x1 * x2 - cos(x1);
    ret->d[1] = exp(x2) * sin(x1 * x2);

    // Using variables so compilter doesnt complain, these have
    // no effect on the function.
    if (t) t = 0;
    if (u.initialized) u.initialized = 0;
}

BOOST_AUTO_TEST_CASE(numjac_test)
{
    rc_vector_t x0 = RC_VECTOR_INITIALIZER;
    rc_vector_t u0 = RC_VECTOR_INITIALIZER;
    rc_vector_alloc(&x0, 2);
    rc_vector_alloc(&u0, 2);
    double t0 = 0;

    x0.d[0] = 1;
    x0.d[1] = 2;

    rc_matrix_t Jac = RC_MATRIX_INITIALIZER;
    rc_matrix_alloc(&Jac, 2, 2);

    numjac(dynamics, t0, x0, u0, &Jac);

    double tol = 0.001;
    BOOST_CHECK_CLOSE(Jac.d[0][0], 4.8415, tol);
    BOOST_CHECK_CLOSE(Jac.d[1][0], -6.1499, tol);
    BOOST_CHECK_CLOSE(Jac.d[0][1], 1.0000, tol);
    BOOST_CHECK_CLOSE(Jac.d[1][1], 3.6439, tol);
}


BOOST_AUTO_TEST_SUITE_END()