/**
 * @file math_utils.c
 **/

#include <assert.h>
#include <stddef.h>

#include <math_utils.h>

void matrix_times_vector(rc_matrix_t A, rc_vector_t x, rc_vector_t* b)
{
    int i, j;
    double sum;

    assert(A.cols == x.len);
    assert(b->len == A.rows);

    for (i = 0; i < A.rows; ++i)
    {
        sum = 0;
        for (j = 0; j < A.cols; ++j)
        {
            sum += A.d[i][j] * x.d[j];
        }
        b->d[i] = sum;
    }
}

void matrix_times_vector_inplace(rc_matrix_t A, rc_vector_t* x)
{
    int i, j;
    double sum;

    assert(A.cols == x->len);
    assert(x->len == A.rows);

    rc_vector_t b = RC_VECTOR_INITIALIZER;
    assert(rc_vector_alloc(&b, x->len) > -1);

    for (i = 0; i < A.rows; ++i)
    {
        sum = 0;
        for (j = 0; j < A.cols; ++j)
        {
            sum += A.d[i][j] * x->d[j];
        }
        b.d[i] = sum;
    }

    for (i = 0; i < b.len; ++i)
    {
        x->d[i] = b.d[i];
    }

    rc_vector_free(&b);
}

void numjac(__dyn_fn_t fn, double t0, rc_vector_t x0, rc_vector_t u0, rc_matrix_t* Jac)
{
    static const double dh = 0.00001;
    rc_vector_t h = RC_VECTOR_INITIALIZER;
    rc_vector_t tmp1 = RC_VECTOR_INITIALIZER;
    rc_vector_t tmp2 = RC_VECTOR_INITIALIZER;
    rc_vector_alloc(&tmp1, x0.len);
    rc_vector_alloc(&tmp2, x0.len);

    int i, j;

    assert(Jac->rows == Jac->cols && Jac->rows == x0.len);

    for (j = 0; j < Jac->cols; ++j)
    {
        rc_vector_zeros(&h, x0.len);

        h.d[j] = dh;
        rc_vector_sum(x0, h, &tmp1);
        h.d[j] = -dh;
        rc_vector_sum(x0, h, &tmp2);
        fn(t0, tmp1, u0, &tmp1);
        fn(t0, tmp2, u0, &tmp2);

        rc_vector_subtract(tmp1, tmp2, &tmp1);
        rc_vector_times_scalar(&tmp1, 1 / (2 * dh));

        for (i = 0; i < Jac->rows; ++i)
        {
            Jac->d[i][j] = tmp1.d[i];
        }
    }

    rc_vector_free(&h);
    rc_vector_free(&tmp1);
    rc_vector_free(&tmp2);
}
