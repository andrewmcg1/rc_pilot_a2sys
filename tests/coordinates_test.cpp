#include <boost/test/unit_test.hpp>

extern "C"
{
#include <coordinates.h>
}

BOOST_AUTO_TEST_SUITE(Test_Coordinates)

BOOST_AUTO_TEST_CASE(test1_lla2ecef)
{
    lla_t test1_wpt_lla = {43, -82, 0};
    ecef_waypoint_t test1_wpt_ecef = lla2ecef(&test1_wpt_lla);

    const double test1_x = 648216.4639;
    const double test1_y = -4612299.801;
    const double test1_z = 4343307.920;

    BOOST_CHECK_CLOSE(test1_wpt_ecef.x, test1_x, 0.00001);
    BOOST_CHECK_CLOSE(test1_wpt_ecef.y, test1_y, 0.00001);
    BOOST_CHECK_CLOSE(test1_wpt_ecef.z, test1_z, 0.00001);
}

BOOST_AUTO_TEST_CASE(test1_lla2ned)
{
    if (coordinates_init() < 0)
    {
        BOOST_CHECK(false);
    }

    lla_t origin_lla = {42, -83, 0};
    set_origin(&origin_lla);

    lla_t test1_wpt_lla = {43, -82, 0};
    ned_waypoint_t test1_wpt_ned = lla2ned(&test1_wpt_lla);

    const double test1_x = 111620.3526;
    const double test1_y = 81286.80836;
    const double test1_z = 1497.125205;

    BOOST_CHECK_CLOSE(test1_wpt_ned.x, test1_x, 0.00001);
    BOOST_CHECK_CLOSE(test1_wpt_ned.y, test1_y, 0.00001);
    BOOST_CHECK_CLOSE(test1_wpt_ned.z, test1_z, 0.00001);
}

BOOST_AUTO_TEST_SUITE_END()