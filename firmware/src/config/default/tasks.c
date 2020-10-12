/*******************************************************************************
 System Tasks File

  File Name:
    tasks.c

  Summary:
    This file contains source code necessary to maintain system's polled tasks.

  Description:
    This file contains source code necessary to maintain system's polled tasks.
    It implements the "SYS_Tasks" function that calls the individual "Tasks"
    functions for all polled MPLAB Harmony modules in the system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    polled in the system.  These handles are passed into the individual module
    "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "configuration.h"
#include "definitions.h"


// *****************************************************************************
// *****************************************************************************
// Section: RTOS "Tasks" Routine
// *****************************************************************************
// *****************************************************************************
void _SYS_CONSOLE_2_Tasks(  void *pvParameters  )
{
    while(1)
    {
        SYS_CONSOLE_Tasks(SYS_CONSOLE_INDEX_2);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


void _SYS_CONSOLE_1_Tasks(  void *pvParameters  )
{
    while(1)
    {
        SYS_CONSOLE_Tasks(SYS_CONSOLE_INDEX_1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


void _USB_DEVICE_Tasks(  void *pvParameters  )
{
    while(1)
    {
				 /* USB Device layer tasks routine */
        USB_DEVICE_Tasks(sysObj.usbDevObject0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void _DRV_USBHS_Tasks(  void *pvParameters  )
{
    while(1)
    {
				 /* USB FS Driver Task Routine */
        DRV_USBHS_Tasks(sysObj.drvUSBHSObject);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void _SYS_CONSOLE_0_Tasks(  void *pvParameters  )
{
    while(1)
    {
        SYS_CONSOLE_Tasks(SYS_CONSOLE_INDEX_0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


/* Handle for the BLINK_LED_THREAD_Tasks. */
TaskHandle_t xBLINK_LED_THREAD_Tasks;

void _BLINK_LED_THREAD_Tasks(  void *pvParameters  )
{   
    portTASK_USES_FLOATING_POINT();
    
    while(1)
    {
        BLINK_LED_THREAD_Tasks();
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}
/* Handle for the HEARTBEAT_LED_TASK_Tasks. */
TaskHandle_t xHEARTBEAT_LED_TASK_Tasks;

void _HEARTBEAT_LED_TASK_Tasks(  void *pvParameters  )
{   
    while(1)
    {
        HEARTBEAT_LED_TASK_Tasks();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
/* Handle for the MAVLINKRECVTASK_Tasks. */
TaskHandle_t xMAVLINKRECVTASK_Tasks;

void _MAVLINKRECVTASK_Tasks(  void *pvParameters  )
{   
    while(1)
    {
        MAVLINKRECVTASK_Tasks();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
/* Handle for the MAVLINKSENDTASK_Tasks. */
TaskHandle_t xMAVLINKSENDTASK_Tasks;

void _MAVLINKSENDTASK_Tasks(  void *pvParameters  )
{   
    while(1)
    {
        MAVLINKSENDTASK_Tasks();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}




// *****************************************************************************
// *****************************************************************************
// Section: System "Tasks" Routine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Tasks ( void )

  Remarks:
    See prototype in system/common/sys_module.h.
*/
void SYS_Tasks ( void )
{
    /* Maintain system services */
        xTaskCreate( _SYS_CONSOLE_2_Tasks,
        "SYS_CONSOLE_2_TASKS",
        SYS_CONSOLE_RTOS_STACK_SIZE_IDX2,
        (void*)NULL,
        SYS_CONSOLE_RTOS_TASK_PRIORITY_IDX2,
        (TaskHandle_t*)NULL
    );

    xTaskCreate( _SYS_CONSOLE_1_Tasks,
        "SYS_CONSOLE_1_TASKS",
        SYS_CONSOLE_RTOS_STACK_SIZE_IDX1,
        (void*)NULL,
        SYS_CONSOLE_RTOS_TASK_PRIORITY_IDX1,
        (TaskHandle_t*)NULL
    );

    xTaskCreate( _SYS_CONSOLE_0_Tasks,
        "SYS_CONSOLE_0_TASKS",
        SYS_CONSOLE_RTOS_STACK_SIZE_IDX0,
        (void*)NULL,
        SYS_CONSOLE_RTOS_TASK_PRIORITY_IDX0,
        (TaskHandle_t*)NULL
    );



    /* Maintain Device Drivers */
    

    /* Maintain Middleware & Other Libraries */
        /* Create OS Thread for USB_DEVICE_Tasks. */
    xTaskCreate( _USB_DEVICE_Tasks,
        "USB_DEVICE_TASKS",
        1024,
        (void*)NULL,
        1,
        (TaskHandle_t*)NULL
    );

	/* Create OS Thread for USB Driver Tasks. */
    xTaskCreate( _DRV_USBHS_Tasks,
        "DRV_USBHS_TASKS",
        1024,
        (void*)NULL,
        1,
        (TaskHandle_t*)NULL
    );



    /* Maintain the application's state machine. */
        /* Create OS Thread for BLINK_LED_THREAD_Tasks. */
    xTaskCreate((TaskFunction_t) _BLINK_LED_THREAD_Tasks,
                "BLINK_LED_THREAD_Tasks",
                1024,
                NULL,
                1,
                &xBLINK_LED_THREAD_Tasks);

    /* Create OS Thread for HEARTBEAT_LED_TASK_Tasks. */
    xTaskCreate((TaskFunction_t) _HEARTBEAT_LED_TASK_Tasks,
                "HEARTBEAT_LED_TASK_Tasks",
                64,
                NULL,
                0,
                &xHEARTBEAT_LED_TASK_Tasks);

    /* Create OS Thread for MAVLINKRECVTASK_Tasks. */
    xTaskCreate((TaskFunction_t) _MAVLINKRECVTASK_Tasks,
                "MAVLINKRECVTASK_Tasks",
                1024,
                NULL,
                2,
                &xMAVLINKRECVTASK_Tasks);

    /* Create OS Thread for MAVLINKSENDTASK_Tasks. */
    xTaskCreate((TaskFunction_t) _MAVLINKSENDTASK_Tasks,
                "MAVLINKSENDTASK_Tasks",
                4024,
                NULL,
                3,
                &xMAVLINKSENDTASK_Tasks);




    /* Start RTOS Scheduler. */
    
     /**********************************************************************
     * Create all Threads for APP Tasks before starting FreeRTOS Scheduler *
     ***********************************************************************/
    vTaskStartScheduler(); /* This function never returns. */

}

/*******************************************************************************
 End of File
 */

