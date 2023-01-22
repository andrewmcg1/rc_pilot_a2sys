#include <boost/test/unit_test.hpp>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

extern "C"
{
#include <controller.h>
#include <feedback.h>
#include <input_manager.h>
#include <path.h>
#include <setpoint_manager.h>

#include <rc/time.h>
}

using namespace std::chrono;
using std::this_thread::sleep_for;

BOOST_AUTO_TEST_SUITE(Test_Waypoint)

BOOST_AUTO_TEST_CASE(waypoint_read)
{
    double k;
    int i, j;
    int num_waypoints = 1;
    int dim = 14;
    const char* path_file_name = "/tmp/tmp_path";

    // Create temporary test path file
    FILE* path_file = fopen(path_file_name, "w");
    k = 0;
    for (i = 0; i < num_waypoints; ++i)
    {
        for (j = 0; j < dim; ++j)
        {
            // Make sure to print the flag state as an integer
            if (j < (dim - 1))
            {
                fprintf(path_file, "%lf ", k++);
            }
            else
            {
                fprintf(path_file, "%i ", (int)k++);
            }
        }
        fprintf(path_file, "\n");
    }
    fclose(path_file);

    // Check that the generated test path file is read in correctly
    if (path_load_from_file(path_file_name) != -1)
    {
        k = 0;
        for (i = 0; i < num_waypoints; ++i)
        {
            BOOST_CHECK_EQUAL(path.waypoints[i].t, k++);
            BOOST_CHECK_EQUAL(path.waypoints[i].x, k++);
            BOOST_CHECK_EQUAL(path.waypoints[i].y, k++);
            BOOST_CHECK_EQUAL(path.waypoints[i].z, k++);
            BOOST_CHECK_EQUAL(path.waypoints[i].xd, k++);
            BOOST_CHECK_EQUAL(path.waypoints[i].yd, k++);
            BOOST_CHECK_EQUAL(path.waypoints[i].zd, k++);
            BOOST_CHECK_EQUAL(path.waypoints[i].roll, k++);
            BOOST_CHECK_EQUAL(path.waypoints[i].pitch, k++);
            BOOST_CHECK_EQUAL(path.waypoints[i].yaw, k++);
            BOOST_CHECK_EQUAL(path.waypoints[i].p, k++);
            BOOST_CHECK_EQUAL(path.waypoints[i].q, k++);
            BOOST_CHECK_EQUAL(path.waypoints[i].r, k++);
            BOOST_CHECK_EQUAL(path.waypoints[i].flag, k++);
        }
    }
    else
    {
        BOOST_CHECK_EQUAL(path.initialized, 0);
        BOOST_CHECK(false);  // Fail if no path is provided
    }

    // Clean up path variable
    path_cleanup();
}

BOOST_AUTO_TEST_CASE(waypoint_transitions)
{
    size_t num_waypoints = 10;
    double delta_t = 0.01;
    double *ptr1 = nullptr, *ptr2 = nullptr;  // throwaway ptrs that should not be used

    std::string path_file_name = "/tmp/temp_path_file";
    std::ofstream fout1(path_file_name);
    for (size_t i = 0; i < num_waypoints; ++i)
    {
        fout1 << i * delta_t << " ";
        for (int j = 0; j < 12; ++j)
        {
            fout1 << i << " ";
        }
        fout1 << 0 << " \n";
    }
    fout1.close();

    setpoint_manager_init();
    user_input.flight_mode = AUTONOMOUS;
    user_input.requested_arm_mode = ARMED;
    fstate.arm_state = ARMED;

    path_load_from_file(path_file_name.c_str());

    BOOST_CHECK_EQUAL(path.len, num_waypoints);

    // BOOST_TEST_MESSAGE("path.len = " << path.len);

    for (size_t i = 0; i < path.len; ++i)
    {
        setpoint_manager_update();
        feedback_controller(ptr1, ptr2);
        BOOST_CHECK_EQUAL(setpoint.X, i);
        sleep_for(duration<double>(delta_t));
    }

    setpoint_manager_cleanup();
}

BOOST_AUTO_TEST_CASE(path_transitions)
{
    int num_waypoints = 10;
    double delta_t = 0.01;
    double *ptr1 = nullptr, *ptr2 = nullptr;  // throwaway ptrs that should not be used

    std::string path_file_name1 = "/tmp/temp_path_file1";
    std::ofstream fout1(path_file_name1);
    for (int i = 0; i < num_waypoints; ++i)
    {
        fout1 << i * delta_t << " ";
        for (int j = 0; j < 12; ++j)
        {
            fout1 << i << " ";
        }
        fout1 << 0 << " \n";
    }
    fout1.close();

    std::string path_file_name2 = "/tmp/temp_path_file2";
    std::ofstream fout2(path_file_name2);
    for (int i = 0; i < num_waypoints; ++i)
    {
        fout2 << i * delta_t << " ";
        for (int j = 0; j < 12; ++j)
        {
            fout2 << i + num_waypoints << " ";
        }
        fout2 << 0 << " \n";
    }
    fout2.close();

    setpoint_manager_init();
    user_input.flight_mode = AUTONOMOUS;
    user_input.requested_arm_mode = ARMED;
    fstate.arm_state = ARMED;

    path_load_from_file(path_file_name1.c_str());
    for (int i = 0; i < 2 * num_waypoints; ++i)
    {
        if (i == num_waypoints) set_new_path(path_file_name2.c_str());
        setpoint_manager_update();
        feedback_controller(ptr1, ptr2);
        BOOST_CHECK_EQUAL(setpoint.X, i);
        sleep_for(duration<double>(delta_t));
    }

    setpoint_manager_cleanup();
}

BOOST_AUTO_TEST_CASE(cleanup)
{
    // clean up
    path_cleanup();
}

BOOST_AUTO_TEST_SUITE_END()