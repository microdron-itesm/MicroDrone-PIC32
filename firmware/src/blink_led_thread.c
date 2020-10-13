/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    blink_led_thread.c

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

#include "blink_led_thread.h"
#include <math.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

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
    This structure should be initialized by the BLINK_LED_THREAD_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

BLINK_LED_THREAD_DATA blink_led_threadData;

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
double angle;

double angleDistance(double alpha, double beta) {
        double phi = fmod(fabs(beta - alpha), 2.0 * M_PI);       // This is either the distance or 360 - distance
        double distance = phi > M_PI ? (2.0 * M_PI) - phi : phi;
        return distance;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void BLINK_LED_THREAD_Initialize ( void )

  Remarks:
    See prototype in blink_led_thread.h.
 */

void BLINK_LED_THREAD_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    blink_led_threadData.state = BLINK_LED_THREAD_STATE_INIT;



    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    OCMP6_Enable();
    OCMP4_Enable();
    OCMP5_Enable();
    OCMP3_Enable();
            
    TMR2_Start();
}


/******************************************************************************
  Function:
    void BLINK_LED_THREAD_Tasks ( void )

  Remarks:
    See prototype in blink_led_thread.h.
 */

void BLINK_LED_THREAD_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( blink_led_threadData.state )
    {
        /* Application's initial state. */
        case BLINK_LED_THREAD_STATE_INIT:
        {
            bool appInitialized = true;
            
            if (appInitialized)
            {

                blink_led_threadData.state = BLINK_LED_THREAD_STATE_SERVICE_TASKS;
            }
            break;
        }

        case BLINK_LED_THREAD_STATE_SERVICE_TASKS:
        {
            if(USR_BTN_Get()){
                //LED0_Toggle();
                //LED1_Toggle();
                //LED2_Toggle();
                //LED3_Toggle();
                //SetDCOC1PWM();
                
                angle += 0.01;
                if(angle > 2.0 * M_PI){
                    angle = 0;
                }
                
                double topLeft = angleDistance(M_PI, angle) / M_PI;
                double topRight = angleDistance(M_PI / 2.0, angle) / M_PI;
                double backRight = angleDistance(0, angle) / M_PI;
                double backLeft = angleDistance(M_PI + M_PI / 2.0, angle) / M_PI;
                
                //OCMP3_CompareSecondaryValueSet(backLeft * (double) 0x4FFF); //BackLeft
                //OCMP6_CompareSecondaryValueSet(topLeft * (double) 0x4FFF); //Top Left
                //OCMP4_CompareSecondaryValueSet(topRight * (double) 0x4FFF); //Top Right
                //OCMP5_CompareSecondaryValueSet(backRight * (double) 0x4FFF); //BackRight
                //SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "TopLeft: %f\r\n", topLeft);
            } else{
                SYS_DEBUG_MESSAGE(SYS_ERROR_DEBUG, "Button Pressed!\n");
            }
            
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
