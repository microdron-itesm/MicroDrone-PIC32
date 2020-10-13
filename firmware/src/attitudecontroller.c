/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    attitudecontroller.c

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

#include "attitudecontroller.h"
#include "Tasks/AttitudeControllerTask/AttitudeControllerTask.h"
#include "motors.h"

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
    This structure should be initialized by the ATTITUDECONTROLLER_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

ATTITUDECONTROLLER_DATA attitudecontrollerData;

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


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ATTITUDECONTROLLER_Initialize ( void )

  Remarks:
    See prototype in attitudecontroller.h.
 */

void ATTITUDECONTROLLER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    attitudecontrollerData.state = ATTITUDECONTROLLER_STATE_INIT;



    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    hal_motors_init();
}


/******************************************************************************
  Function:
    void ATTITUDECONTROLLER_Tasks ( void )

  Remarks:
    See prototype in attitudecontroller.h.
 */

void ATTITUDECONTROLLER_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( attitudecontrollerData.state )
    {
        /* Application's initial state. */
        case ATTITUDECONTROLLER_STATE_INIT:
        {
            bool appInitialized = true;

            AttitudeController_Init(NULL);

            if (appInitialized)
            {

                attitudecontrollerData.state = ATTITUDECONTROLLER_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ATTITUDECONTROLLER_STATE_SERVICE_TASKS:
        {
            AttitudeController_Update(NULL);
            
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
