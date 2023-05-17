#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>


using namespace std::chrono;
using std::this_thread::sleep_for;


int main()
{
    double k;
    int i, j;
    int num_waypoints = 2;
    int dim = 14;
    const char* path_file_name = "testfile.txt";

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
    //fclose(path_file);
}