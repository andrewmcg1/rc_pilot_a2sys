/**
 * @file ntp_read.c
 */

#include <rc/pthread.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#include <stdio.h>
#include <stdlib.h>

#include <ntp_read.h>



#define NTP_PATH_SIZE 1024          //Size of character buffer when reading NTP offset values

//Thread priority. Higher number is higher priority. 
//Priorities of other threads: input manager is 80, printf_manager is 60, log_manager is 50
#define NTP_OFFSET_PRI 40 

#define NTP_SLEEP_TIME_S 16// [s] Wait N seconds before checking for NTP offset again

#define NTP_CLEANUP_TIMEOUT 2.0 // [s] Timeout for trying to terminate NTP thread

//Extern variables
ntp_struct ntp_data;

//Static variabls
static pthread_t ntp_offset_thread;
static int ntp_offset_enabled;

static void* __ntp_offset_func(__attribute__((unused)) void* ptr)
{
    double temp;
    rc_state_t curr_state = rc_get_state();
    while (1)
    {
        temp = get_ntp_offset();

        if(temp != NTP_ERR_NUM)
        {
            ntp_data.ntp_offset_ms = temp;
            ntp_data.ntp_time_updated_ns = rc_nanos_since_epoch();
        }
        rc_nanosleep(1e9 * NTP_SLEEP_TIME_S);
        
        //Get new state and break if it's exiting
        curr_state = rc_get_state();
        if(curr_state == EXITING || !ntp_offset_enabled)
        {
            break;
        }
    }

    // fprintf(stderr, "Quitting NTP thread\n");
    return NULL;
}

int ntp_offset_init()
{
    ntp_data.ntp_offset_ms = NTP_ERR_NUM;
    ntp_offset_enabled = 0;

    if (rc_pthread_create(&ntp_offset_thread, __ntp_offset_func, NULL, SCHED_FIFO, NTP_OFFSET_PRI) < 0)
    {
        fprintf(stderr, "ERROR in ntp_offset_init, failed to start thread\n");
        return -1;
    }

    ntp_offset_enabled = 1;
    rc_usleep(1000);

    return 0;
}

int ntp_offset_cleanup()
{
    if(ntp_offset_enabled == 0) return 0;

    //Disable gathering of NTP offsets
    ntp_offset_enabled = 0;

    //Attempt to terminate the thread
    int ret = rc_pthread_timed_join(ntp_offset_thread, NULL, NTP_CLEANUP_TIMEOUT);
    if (ret == 1)
    {
        fprintf(stderr, "WARNING: ntp_offset_thread exit timeout\n");
    }
    else if (ret == -1)
    {
        fprintf(stderr, "ERROR: failed to join ntp_offset_thread thread\n");
    }
    return ret;
}


//To get synchronized time, symply ADD this offset value to all timestamps received.    
double get_ntp_offset(){
    FILE *ntpFp;
    char ntpResult[NTP_PATH_SIZE];
    double offsetms = 0;

    //Parse string from NTP
    ntpFp = popen("/usr/bin/ntpq -pn", "r");

    //Leave immediately if we couldn't run the command
    if(ntpFp == NULL)
    {
        return NTP_ERR_NUM;
    }

    //Skip first two LINES of ntpq's output
    for(unsigned i = 0; i < 2; ++i){
        if(fgets(ntpResult, NTP_PATH_SIZE, ntpFp) == NULL){
            return NTP_ERR_NUM;
        }
    }

    //On the third line, skip everything before the OFFSET
    for(unsigned i = 0; i < 8; ++i){
        if(fscanf(ntpFp, "%s", ntpResult) == EOF){
            return NTP_ERR_NUM;
        }
    }

    //Next thing in the buffer should be the offset for the first NTP server defined in the /etc/ntp.conf file
    if(fscanf(ntpFp, "%s", ntpResult) != EOF){
        offsetms = atof(ntpResult);
    }

    //Close the filestream
    pclose(ntpFp);

    return offsetms;

}