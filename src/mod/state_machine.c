/**
 * @file state_machine.c
 */

// NOTE: All functionality for LOITER will be added at a later time if at all.
// LOITER was not used much in a previous version of this project

#include <rc/time.h>
#include <setpoint_manager.h>
#include <settings.h>
#include <state_machine.h>
#include <stdio.h>
#include <string.h>

static const char* sm_alph_strings[] = {
    "ENTER_STANDBY",
    "ENTER_TAKEOFF",
    "ENTER_GUIDED",
    "ENTER_LANDING",
    "ENTER_SM_LOITER",
    "ENTER_NAILING",
    "ENTER_RETURN",
    "NO_EVENT",
};

state_machine_t waypoint_state_machine = STATE_MACHINE_INITIALIZER;

static char waypoint_filename[200];

/**
 * @brief Concatennates 'folder' and 'file' strings and stores them in 'dest' string
 */
static void __build_waypoit_filename(char* dest, char* folder, char* file)
{
    dest[0] = '\0';
    strcat(dest, folder);
    strcat(dest, file);
}

/**
 * @brief Initialize statemachine to STANDY state
 */
int sm_init(state_machine_t* sm)
{
    sm->current_state = STANDBY;
    sm->state_transition_time = 0;
    sm->changedState = false;

    return 1;
}

/**
 * @brief Parse the input and transition to new state if applicable
 */
void sm_transition(state_machine_t* sm, sm_alphabet input)
{
    // static sm_alphabet last_input = NO_EVENT;

    // Do nothing if the input hasn't changed
    // if (input == last_input)
    // {
    //     return;
    // }
    // last_input = input;

    // Things that should be done for all state transitions

    // Unique things that should be done for each state
    switch (sm->current_state)
    {
        /** STANDBY: Assumes vehicle is on the ground and waiting to take off.
         * Valid Transitions: STANDBY, TAKEOFF
         */
        case STANDBY:
            switch (input)
            {
                case ENTER_STANDBY:
                    break;

                case ENTER_TAKEOFF:
                    sm->current_state = TAKEOFF;
                    sm->changedState = true;
                    break;

                default:
                    fprintf(stderr, "\nSTANDBY cannot transition with event %s\n",
                        sm_alph_strings[input]);
                    break;
            }
            break;

        /** TAKEOFF: Assumes vehicle is on the ground. Parses through waypoints to get from
         * ground into the air Valid Transitions: TAKEOFF, GUIDED, LANDING, LOITER, NAILING,
         * RETURN
         */
        case TAKEOFF:

            // Actions associated with this state
            if (sm->changedState)
            {
                // Load new waypoint file
                __build_waypoit_filename(
                    waypoint_filename, settings.wp_folder, settings.wp_takeoff_filename);

                set_new_path(waypoint_filename);
                sm->changedState = false;
            }

            // State transition
            switch (input)
            {
                case ENTER_TAKEOFF:
                    break;

                case ENTER_STANDBY:
                    sm->current_state = STANDBY;
                    sm->changedState = true;
                    break;

                case ENTER_GUIDED:
                    sm->current_state = GUIDED;
                    sm->changedState = true;
                    break;

                case ENTER_LANDING:
                    sm->current_state = LANDING;
                    sm->changedState = true;
                    break;

                case ENTER_SM_LOITER:
                    sm->current_state = SM_LOITER;
                    sm->changedState = true;
                    break;

                case ENTER_NAILING:
                    sm->current_state = NAILING;
                    sm->changedState = true;
                    break;

                case ENTER_RETURN:
                    sm->current_state = RETURN;
                    sm->changedState = true;
                    break;

                default:
                    fprintf(stderr, "\nTAKEOFF cannot transition with event %s\n",
                        sm_alph_strings[input]);
                    break;
            }
            break;

        /** GUIDED: Assumes vehicle is in the air. Parses through waypoints of desried
         * trajectory. Valid Transitions: GUIDED, LANDING, LOITER, NAILING, RETURN
         */
        case GUIDED:
            // Actions associated with this state
            if (sm->changedState)
            {
                __build_waypoit_filename(
                    waypoint_filename, settings.wp_folder, settings.wp_guided_filename);

                set_new_path(waypoint_filename);
                sm->changedState = false;
            }

            // State transition
            switch (input)
            {
                case ENTER_GUIDED:
                    break;

                case ENTER_STANDBY:
                    sm->current_state = STANDBY;
                    sm->changedState = true;
                    break;

                case ENTER_LANDING:
                    sm->current_state = LANDING;
                    sm->changedState = true;
                    // TODO: Load waypoints from LANDING file
                    break;

                case ENTER_SM_LOITER:
                    sm->current_state = SM_LOITER;
                    sm->changedState = true;
                    break;

                case ENTER_NAILING:
                    sm->current_state = NAILING;
                    sm->changedState = true;
                    // TODO: Load waypoints from NAILING file
                    break;

                case ENTER_RETURN:
                    sm->current_state = RETURN;
                    sm->changedState = true;
                    // TODO: Load waypoints from RETURN file
                    break;

                default:
                    fprintf(stderr, "\nGUIDED cannot transition with event %s\n",
                        sm_alph_strings[input]);
                    break;
            }
            break;

        /** LANDING: Assumes vehicle is in the air. Parses through waypoints to get from the air
         * to the ground Valid Transitions: STANDBY, LANDING, LOITER
         */
        case LANDING:
            // Actions associated with this state
            if (sm->changedState)
            {
                __build_waypoit_filename(
                    waypoint_filename, settings.wp_folder, settings.wp_landing_filename);

                set_new_path(waypoint_filename);
                sm->changedState = false;
            }

            // State transition
            switch (input)
            {
                case ENTER_STANDBY:
                    sm->current_state = STANDBY;
                    sm->changedState = true;
                    break;

                case ENTER_LANDING:

                    break;

                case ENTER_SM_LOITER:
                    sm->current_state = SM_LOITER;
                    sm->changedState = true;
                    break;

                default:
                    fprintf(stderr, "\nLANDING cannot transition with event %s\n",
                        sm_alph_strings[input]);
                    break;
            }
            break;

        // States that have not yet been implemented
        case SM_LOITER:
            fprintf(stderr,
                "\nSM_LOITER mode not yet implemented. Switching state to STANDBY. Input: %s\n",
                sm_alph_strings[input]);
            sm->current_state = STANDBY;
            sm->changedState = true;
            break;
        case NAILING:
            fprintf(stderr,
                "\nNAILING mode not yet implemented. Switching state to STANDBY. Input: %s\n",
                sm_alph_strings[input]);
            sm->current_state = STANDBY;
            sm->changedState = true;
            break;
        case RETURN:
            fprintf(stderr,
                "\nRETURN mode not yet implemented. Switching state to STANDBY. Input: %s\n",
                sm_alph_strings[input]);
            sm->current_state = STANDBY;
            sm->changedState = true;
            break;
    }
}
