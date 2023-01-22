/**
 * <mavlink_manager.h>
 *
 * @brief   Functions to start and stop the mavlink manager
 *
 * @addtogroup MavlinkManager
 * @{
 */

#ifndef __MAVLINK_MANAGER__
#define __MAVLINK_MANAGER__

/**
 * @brief   Starts the mavlink manager
 *
 * @return  0 on success, -1 on failure
 */
int mavlink_manager_init(void);

/**
 * @brief   stops the mavlink manager
 *
 * @return  0 if thread exited cleanly, -1 if exit timed out.
 */
int mavlink_manager_cleanup(void);

#endif /* __MAVLINK_MANAGER__ */

/* @} end group Mavlink Manager */
