/**
 * <thread_defs.h>
 *
 * @brief   Thread speeds, prioritites, and close timeouts
 *
 * @addtogroup ThreadDefs
 * @{
 */

#ifndef __THREAD_DEFS__
#define __THREAD_DEFS__

#define INPUT_MANAGER_HZ 20
#define INPUT_MANAGER_PRI 80
#define INPUT_MANAGER_TOUT 0.5
#define LOG_MANAGER_HZ 20
#define LOG_MANAGER_PRI 50
#define LOG_MANAGER_TOUT 2.0
#define PRINTF_MANAGER_HZ 4
#define PRINTF_MANAGER_PRI 60
#define PRINTF_MANAGER_TOUT 0.5
#define BUTTON_EXIT_CHECK_HZ 10
#define BUTTON_EXIT_TIME_S 2

#endif /* __THREAD_DEFS__ */

/* @} end group ThreadDefs */