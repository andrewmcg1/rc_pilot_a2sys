/**
 * <ntp_read.h>
 *
 * @brief       Code for reading the NTP timer offset in order to beaglebones.
 *              Only used if "log_ntp" flag is 'true' in settings file and
 *              if NTP server is running on HP laptop and
 *              BeagleBoneBlue is running NTP client and both laptop and BeagleBoneBlue are connected to A2sys router
 *
 * 
 * @author     Prince Kuevor 
 * @date       05/21/2020 (MM/DD/YYYY)
 * 
 * 
 * @addtogroup  NTP_READ
 * @{
 */

#ifndef __NTP_READ__
#define __NTP_READ__

#include <stdint.h>

#define NTP_COUNTER_THRESH 200*20   //Number of iterations through imu_isr() between each update of NTP offset
#define NTP_ERR_NUM 9999.9999       //Special value returned when unable to read NTP offset

unsigned ntpCounter;

/**
 * @brief   Offset read from NTP daemon
 */
typedef struct ntp_struct
{
    double ntp_offset_ms;
    uint64_t ntp_time_updated_ns;
} ntp_struct;

extern ntp_struct ntp_data;

int ntp_offset_init();
int ntp_offset_cleanup();
double get_ntp_offset();


#endif //__NTP_READ__