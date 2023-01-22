/**
 * <benchmark.h>
 *
 * @brief       Code for timing portions of the software.
 *              Recommended to use rc_nanos_since_epoch() for timers for consistency with other
 *              timestamps throughout rc_pilot. 
 *
 * 
 * @author     Prince Kuevor 
 * @date       02/23/2020 (MM/DD/YYYY)
 * 
 * 
 * @addtogroup  BENCHMARK
 * @{
 */

#ifndef __BENCHMARK__
#define __BENCHMARK__

#include <inttypes.h>
#include <time.h>	// for timespec

/**
 * @brief   User-specified timers for benchmarking rc_pilot functionality
 */
typedef struct benchmark_t{
    /** @name Timers for imu_isr() */
	///@{
    uint64_t tIMU, tIMU_END, tSM, tXBEE, tGPS, tPNI, tNAV, tGUI, tCTR, tLOG, tNTP;
    ///@}
} benchmark_t;

extern benchmark_t benchmark_timers;


#endif //__BENCHMARK__