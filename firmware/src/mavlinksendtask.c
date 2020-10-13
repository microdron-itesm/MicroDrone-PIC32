/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    mavlinksendtask.c

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

#include "mavlinksendtask.h"
#include "MAVLink/MAVLinkSender.h"
#include "Tasks/MAVLink/MAVLinkSendTask.h"

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
    This structure should be initialized by the MAVLINKSENDTASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

MAVLINKSENDTASK_DATA mavlinksendtaskData;

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
QueueHandle_t g_mavLinkSendQueue;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MAVLINKSENDTASK_Initialize ( void )

  Remarks:
    See prototype in mavlinksendtask.h.
 */

void MAVLINKSENDTASK_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    mavlinksendtaskData.state = MAVLINKSENDTASK_STATE_INIT;



    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    hal_comms_init(NULL, 0);
}


/******************************************************************************
  Function:
    void MAVLINKSENDTASK_Tasks ( void )

  Remarks:
    See prototype in mavlinksendtask.h.
 */

void MAVLINKSENDTASK_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( mavlinksendtaskData.state )
    {
        /* Application's initial state. */
        case MAVLINKSENDTASK_STATE_INIT:
        {
            bool appInitialized = true;
            
            g_mavLinkSendQueue = xQueueCreate(10, sizeof(mavlink_message_t));
            MAVLinkSend_Init(NULL);


            if (appInitialized)
            {

                mavlinksendtaskData.state = MAVLINKSENDTASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case MAVLINKSENDTASK_STATE_SERVICE_TASKS:
        {
            MAVLinkSend_Update(NULL);
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
