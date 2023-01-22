#include <boost/test/unit_test.hpp>

extern "C"
{
#include <path.h>
}

BOOST_AUTO_TEST_SUITE(Test_RSP_Splined_Guidance)

BOOST_AUTO_TEST_CASE(test_make_1d_quintic_spline)
{
    float dx = 1.7;
    float dt = 2.1;
    quintic_spline_1d_t the_spline = make_1d_quintic_spline(dx, dt);

    BOOST_CHECK_CLOSE(the_spline.c0, 0.0, 0.00001);
    BOOST_CHECK_CLOSE(the_spline.c1, 0.0, 0.00001);
    BOOST_CHECK_CLOSE(the_spline.c2, 0.0, 0.00001);
    BOOST_CHECK_CLOSE(the_spline.c3, 1.835654896879386,  0.0001);
    BOOST_CHECK_CLOSE(the_spline.c4, -1.311182069199562, 0.0001);
    BOOST_CHECK_CLOSE(the_spline.c5, 0.249748965561821,  0.0001);

    // BOOST_TEST_MESSAGE("This is a message within test_make_1d_quintic_spline");
}

BOOST_AUTO_TEST_CASE(test_compute_spline_position)
{
    float dx = 1.7;
    float dt = 2.1;
    quintic_spline_1d_t the_spline = make_1d_quintic_spline(dx, dt);

    float t = 1.05;
    float ds = compute_spline_position(&the_spline, t);

    BOOST_CHECK_CLOSE(ds, 0.85,  0.0001);

}

BOOST_AUTO_TEST_CASE(test_path_plan_from_rsp_cmd)
{
    // TODO
}


BOOST_AUTO_TEST_SUITE_END()