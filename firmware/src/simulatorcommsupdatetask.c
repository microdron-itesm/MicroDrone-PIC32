/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    simulatorcommsupdatetask.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "simulatorcommsupdatetask.h"
#include "SimCommsUpdateTask/SimCommsUpdateTask.h"
#include "simulatorComms.h"
#include "definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the SIMULATORCOMMSUPDATETASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

SIMULATORCOMMSUPDATETASK_DATA simulatorcommsupdatetaskData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
extern QueueHandle_t g_mavLinkSIMSendQueue;
static mavlink_message_t msg;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SIMULATORCOMMSUPDATETASK_Initialize ( void )

  Remarks:
    See prototype in simulatorcommsupdatetask.h.
 */

void SIMULATORCOMMSUPDATETASK_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    simulatorcommsupdatetaskData.state = SIMULATORCOMMSUPDATETASK_STATE_INIT;



    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void SIMULATORCOMMSUPDATETASK_Tasks ( void )

  Remarks:
    See prototype in simulatorcommsupdatetask.h.
 */

void SIMULATORCOMMSUPDATETASK_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( simulatorcommsupdatetaskData.state )
    {
        /* Application's initial state. */
        case SIMULATORCOMMSUPDATETASK_STATE_INIT:
        {
            bool appInitialized = true;
            
            g_mavLinkSIMSendQueue = xQueueCreate(10, sizeof(mavlink_message_t));
            
            hal_sim_comms_init(NULL);
            SimCommsUpdate_Init(NULL);


            if (appInitialized)
            {

                simulatorcommsupdatetaskData.state = SIMULATORCOMMSUPDATETASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case SIMULATORCOMMSUPDATETASK_STATE_SERVICE_TASKS:
        {
            SimCommsUpdate_Update(NULL);
            mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,   MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |  MAV_MODE_MANUAL_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0xDEAD, MAV_STATE_ACTIVE);
            sendSIM_MAVLinkMessage(&msg);
            break;
        }

        /* TODO: implement your application state machine.*/


        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}


/*******************************************************************************
 End of File
 */
