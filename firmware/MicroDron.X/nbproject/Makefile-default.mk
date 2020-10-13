#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../MicroDrone-Firmware/HAL/PIC/simulatorComms.c ../MicroDrone-Firmware/HAL/PIC/comms.c ../MicroDrone-Firmware/HAL/PIC/tof.c ../MicroDrone-Firmware/HAL/PIC/motors.c ../MicroDrone-Firmware/HAL/PIC/imu.c ../MicroDrone-Firmware/HAL/SIM/sim.c ../MicroDrone-Firmware/HAL/SimCommsUpdateTask/SimCommsUpdateTask.c ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c ../MicroDrone-Firmware/src/Control/SensFusion.c ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c ../MicroDrone-Firmware/src/Control/PID.c ../MicroDrone-Firmware/src/Control/AttitudeController.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c ../MicroDrone-Firmware/src/Utils/num.c ../MicroDrone-Firmware/libs/Math/Quaternion.c ../MicroDrone-Firmware/libs/Math/Vector3D.c ../src/config/default/peripheral/ocmp/plib_ocmp6.c ../src/config/default/system/console/src/sys_console_usb_cdc.c ../src/config/default/usb/src/usb_device_cdc.c ../src/config/default/usb/src/usb_device_cdc_acm.c ../src/main.c ../src/config/default/initialization.c ../src/config/default/interrupts.c ../src/config/default/interrupts_a.S ../src/config/default/exceptions.c ../src/config/default/stdio/xc32_monitor.c ../src/config/default/peripheral/clk/plib_clk.c ../src/config/default/peripheral/gpio/plib_gpio.c ../src/config/default/peripheral/cache/plib_cache.c ../src/config/default/peripheral/cache/plib_cache_pic32mz.S ../src/config/default/peripheral/evic/plib_evic.c ../src/config/default/peripheral/dmac/plib_dmac.c ../src/config/default/usb_device_init_data.c ../src/config/default/usb/src/usb_device.c ../src/config/default/peripheral/ocmp/plib_ocmp4.c ../src/config/default/peripheral/ocmp/plib_ocmp5.c ../src/config/default/peripheral/ocmp/plib_ocmp3.c ../src/config/default/system/debug/src/sys_debug.c ../src/config/default/system/time/src/sys_time.c ../src/config/default/peripheral/coretimer/plib_coretimer.c ../src/config/default/driver/usb/usbhs/src/drv_usbhs.c ../src/config/default/driver/usb/usbhs/src/drv_usbhs_device.c ../src/config/default/peripheral/uart/plib_uart1.c ../src/config/default/peripheral/tmr/plib_tmr2.c ../src/config/default/system/console/src/sys_console.c ../src/config/default/freertos_hooks.c ../src/third_party/rtos/FreeRTOS/Source/croutine.c ../src/third_party/rtos/FreeRTOS/Source/list.c ../src/third_party/rtos/FreeRTOS/Source/queue.c ../src/third_party/rtos/FreeRTOS/Source/FreeRTOS_tasks.c ../src/third_party/rtos/FreeRTOS/Source/timers.c ../src/third_party/rtos/FreeRTOS/Source/event_groups.c ../src/third_party/rtos/FreeRTOS/Source/stream_buffer.c ../src/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c ../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c ../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S ../src/config/default/system/int/src/sys_int.c ../src/config/default/system/cache/sys_cache.c ../src/config/default/system/dma/sys_dma.c ../src/config/default/osal/osal_freertos.c ../src/blink_led_thread.c ../src/heartbeat_led_task.c ../src/mavlinkrecvtask.c ../src/mavlinksendtask.c ../src/mavlinkstatustask.c ../src/simulatorcommsupdatetask.c ../src/attitudecontroller.c ../src/config/default/tasks.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/2101238180/simulatorComms.o ${OBJECTDIR}/_ext/2101238180/comms.o ${OBJECTDIR}/_ext/2101238180/tof.o ${OBJECTDIR}/_ext/2101238180/motors.o ${OBJECTDIR}/_ext/2101238180/imu.o ${OBJECTDIR}/_ext/2101241073/sim.o ${OBJECTDIR}/_ext/882548988/SimCommsUpdateTask.o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ${OBJECTDIR}/_ext/1204493032/PID.o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ${OBJECTDIR}/_ext/1313821444/num.o ${OBJECTDIR}/_ext/971049567/Quaternion.o ${OBJECTDIR}/_ext/971049567/Vector3D.o ${OBJECTDIR}/_ext/1865480137/plib_ocmp6.o ${OBJECTDIR}/_ext/1832805299/sys_console_usb_cdc.o ${OBJECTDIR}/_ext/308758920/usb_device_cdc.o ${OBJECTDIR}/_ext/308758920/usb_device_cdc_acm.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1171490990/initialization.o ${OBJECTDIR}/_ext/1171490990/interrupts.o ${OBJECTDIR}/_ext/1171490990/interrupts_a.o ${OBJECTDIR}/_ext/1171490990/exceptions.o ${OBJECTDIR}/_ext/163028504/xc32_monitor.o ${OBJECTDIR}/_ext/60165520/plib_clk.o ${OBJECTDIR}/_ext/1865254177/plib_gpio.o ${OBJECTDIR}/_ext/1984157808/plib_cache.o ${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o ${OBJECTDIR}/_ext/1865200349/plib_evic.o ${OBJECTDIR}/_ext/1865161661/plib_dmac.o ${OBJECTDIR}/_ext/1171490990/usb_device_init_data.o ${OBJECTDIR}/_ext/308758920/usb_device.o ${OBJECTDIR}/_ext/1865480137/plib_ocmp4.o ${OBJECTDIR}/_ext/1865480137/plib_ocmp5.o ${OBJECTDIR}/_ext/1865480137/plib_ocmp3.o ${OBJECTDIR}/_ext/944882569/sys_debug.o ${OBJECTDIR}/_ext/101884895/sys_time.o ${OBJECTDIR}/_ext/1249264884/plib_coretimer.o ${OBJECTDIR}/_ext/2071311437/drv_usbhs.o ${OBJECTDIR}/_ext/2071311437/drv_usbhs_device.o ${OBJECTDIR}/_ext/1865657120/plib_uart1.o ${OBJECTDIR}/_ext/60181895/plib_tmr2.o ${OBJECTDIR}/_ext/1832805299/sys_console.o ${OBJECTDIR}/_ext/1171490990/freertos_hooks.o ${OBJECTDIR}/_ext/404212886/croutine.o ${OBJECTDIR}/_ext/404212886/list.o ${OBJECTDIR}/_ext/404212886/queue.o ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o ${OBJECTDIR}/_ext/404212886/timers.o ${OBJECTDIR}/_ext/404212886/event_groups.o ${OBJECTDIR}/_ext/404212886/stream_buffer.o ${OBJECTDIR}/_ext/1665200909/heap_1.o ${OBJECTDIR}/_ext/951553246/port.o ${OBJECTDIR}/_ext/951553246/port_asm.o ${OBJECTDIR}/_ext/1881668453/sys_int.o ${OBJECTDIR}/_ext/1014039709/sys_cache.o ${OBJECTDIR}/_ext/14461671/sys_dma.o ${OBJECTDIR}/_ext/1529399856/osal_freertos.o ${OBJECTDIR}/_ext/1360937237/blink_led_thread.o ${OBJECTDIR}/_ext/1360937237/heartbeat_led_task.o ${OBJECTDIR}/_ext/1360937237/mavlinkrecvtask.o ${OBJECTDIR}/_ext/1360937237/mavlinksendtask.o ${OBJECTDIR}/_ext/1360937237/mavlinkstatustask.o ${OBJECTDIR}/_ext/1360937237/simulatorcommsupdatetask.o ${OBJECTDIR}/_ext/1360937237/attitudecontroller.o ${OBJECTDIR}/_ext/1171490990/tasks.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/2101238180/simulatorComms.o.d ${OBJECTDIR}/_ext/2101238180/comms.o.d ${OBJECTDIR}/_ext/2101238180/tof.o.d ${OBJECTDIR}/_ext/2101238180/motors.o.d ${OBJECTDIR}/_ext/2101238180/imu.o.d ${OBJECTDIR}/_ext/2101241073/sim.o.d ${OBJECTDIR}/_ext/882548988/SimCommsUpdateTask.o.d ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d ${OBJECTDIR}/_ext/1204493032/SensFusion.o.d ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d ${OBJECTDIR}/_ext/1204493032/PID.o.d ${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d ${OBJECTDIR}/_ext/1313821444/num.o.d ${OBJECTDIR}/_ext/971049567/Quaternion.o.d ${OBJECTDIR}/_ext/971049567/Vector3D.o.d ${OBJECTDIR}/_ext/1865480137/plib_ocmp6.o.d ${OBJECTDIR}/_ext/1832805299/sys_console_usb_cdc.o.d ${OBJECTDIR}/_ext/308758920/usb_device_cdc.o.d ${OBJECTDIR}/_ext/308758920/usb_device_cdc_acm.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1171490990/initialization.o.d ${OBJECTDIR}/_ext/1171490990/interrupts.o.d ${OBJECTDIR}/_ext/1171490990/interrupts_a.o.d ${OBJECTDIR}/_ext/1171490990/exceptions.o.d ${OBJECTDIR}/_ext/163028504/xc32_monitor.o.d ${OBJECTDIR}/_ext/60165520/plib_clk.o.d ${OBJECTDIR}/_ext/1865254177/plib_gpio.o.d ${OBJECTDIR}/_ext/1984157808/plib_cache.o.d ${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o.d ${OBJECTDIR}/_ext/1865200349/plib_evic.o.d ${OBJECTDIR}/_ext/1865161661/plib_dmac.o.d ${OBJECTDIR}/_ext/1171490990/usb_device_init_data.o.d ${OBJECTDIR}/_ext/308758920/usb_device.o.d ${OBJECTDIR}/_ext/1865480137/plib_ocmp4.o.d ${OBJECTDIR}/_ext/1865480137/plib_ocmp5.o.d ${OBJECTDIR}/_ext/1865480137/plib_ocmp3.o.d ${OBJECTDIR}/_ext/944882569/sys_debug.o.d ${OBJECTDIR}/_ext/101884895/sys_time.o.d ${OBJECTDIR}/_ext/1249264884/plib_coretimer.o.d ${OBJECTDIR}/_ext/2071311437/drv_usbhs.o.d ${OBJECTDIR}/_ext/2071311437/drv_usbhs_device.o.d ${OBJECTDIR}/_ext/1865657120/plib_uart1.o.d ${OBJECTDIR}/_ext/60181895/plib_tmr2.o.d ${OBJECTDIR}/_ext/1832805299/sys_console.o.d ${OBJECTDIR}/_ext/1171490990/freertos_hooks.o.d ${OBJECTDIR}/_ext/404212886/croutine.o.d ${OBJECTDIR}/_ext/404212886/list.o.d ${OBJECTDIR}/_ext/404212886/queue.o.d ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o.d ${OBJECTDIR}/_ext/404212886/timers.o.d ${OBJECTDIR}/_ext/404212886/event_groups.o.d ${OBJECTDIR}/_ext/404212886/stream_buffer.o.d ${OBJECTDIR}/_ext/1665200909/heap_1.o.d ${OBJECTDIR}/_ext/951553246/port.o.d ${OBJECTDIR}/_ext/951553246/port_asm.o.d ${OBJECTDIR}/_ext/1881668453/sys_int.o.d ${OBJECTDIR}/_ext/1014039709/sys_cache.o.d ${OBJECTDIR}/_ext/14461671/sys_dma.o.d ${OBJECTDIR}/_ext/1529399856/osal_freertos.o.d ${OBJECTDIR}/_ext/1360937237/blink_led_thread.o.d ${OBJECTDIR}/_ext/1360937237/heartbeat_led_task.o.d ${OBJECTDIR}/_ext/1360937237/mavlinkrecvtask.o.d ${OBJECTDIR}/_ext/1360937237/mavlinksendtask.o.d ${OBJECTDIR}/_ext/1360937237/mavlinkstatustask.o.d ${OBJECTDIR}/_ext/1360937237/simulatorcommsupdatetask.o.d ${OBJECTDIR}/_ext/1360937237/attitudecontroller.o.d ${OBJECTDIR}/_ext/1171490990/tasks.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/2101238180/simulatorComms.o ${OBJECTDIR}/_ext/2101238180/comms.o ${OBJECTDIR}/_ext/2101238180/tof.o ${OBJECTDIR}/_ext/2101238180/motors.o ${OBJECTDIR}/_ext/2101238180/imu.o ${OBJECTDIR}/_ext/2101241073/sim.o ${OBJECTDIR}/_ext/882548988/SimCommsUpdateTask.o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ${OBJECTDIR}/_ext/1204493032/PID.o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ${OBJECTDIR}/_ext/1313821444/num.o ${OBJECTDIR}/_ext/971049567/Quaternion.o ${OBJECTDIR}/_ext/971049567/Vector3D.o ${OBJECTDIR}/_ext/1865480137/plib_ocmp6.o ${OBJECTDIR}/_ext/1832805299/sys_console_usb_cdc.o ${OBJECTDIR}/_ext/308758920/usb_device_cdc.o ${OBJECTDIR}/_ext/308758920/usb_device_cdc_acm.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1171490990/initialization.o ${OBJECTDIR}/_ext/1171490990/interrupts.o ${OBJECTDIR}/_ext/1171490990/interrupts_a.o ${OBJECTDIR}/_ext/1171490990/exceptions.o ${OBJECTDIR}/_ext/163028504/xc32_monitor.o ${OBJECTDIR}/_ext/60165520/plib_clk.o ${OBJECTDIR}/_ext/1865254177/plib_gpio.o ${OBJECTDIR}/_ext/1984157808/plib_cache.o ${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o ${OBJECTDIR}/_ext/1865200349/plib_evic.o ${OBJECTDIR}/_ext/1865161661/plib_dmac.o ${OBJECTDIR}/_ext/1171490990/usb_device_init_data.o ${OBJECTDIR}/_ext/308758920/usb_device.o ${OBJECTDIR}/_ext/1865480137/plib_ocmp4.o ${OBJECTDIR}/_ext/1865480137/plib_ocmp5.o ${OBJECTDIR}/_ext/1865480137/plib_ocmp3.o ${OBJECTDIR}/_ext/944882569/sys_debug.o ${OBJECTDIR}/_ext/101884895/sys_time.o ${OBJECTDIR}/_ext/1249264884/plib_coretimer.o ${OBJECTDIR}/_ext/2071311437/drv_usbhs.o ${OBJECTDIR}/_ext/2071311437/drv_usbhs_device.o ${OBJECTDIR}/_ext/1865657120/plib_uart1.o ${OBJECTDIR}/_ext/60181895/plib_tmr2.o ${OBJECTDIR}/_ext/1832805299/sys_console.o ${OBJECTDIR}/_ext/1171490990/freertos_hooks.o ${OBJECTDIR}/_ext/404212886/croutine.o ${OBJECTDIR}/_ext/404212886/list.o ${OBJECTDIR}/_ext/404212886/queue.o ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o ${OBJECTDIR}/_ext/404212886/timers.o ${OBJECTDIR}/_ext/404212886/event_groups.o ${OBJECTDIR}/_ext/404212886/stream_buffer.o ${OBJECTDIR}/_ext/1665200909/heap_1.o ${OBJECTDIR}/_ext/951553246/port.o ${OBJECTDIR}/_ext/951553246/port_asm.o ${OBJECTDIR}/_ext/1881668453/sys_int.o ${OBJECTDIR}/_ext/1014039709/sys_cache.o ${OBJECTDIR}/_ext/14461671/sys_dma.o ${OBJECTDIR}/_ext/1529399856/osal_freertos.o ${OBJECTDIR}/_ext/1360937237/blink_led_thread.o ${OBJECTDIR}/_ext/1360937237/heartbeat_led_task.o ${OBJECTDIR}/_ext/1360937237/mavlinkrecvtask.o ${OBJECTDIR}/_ext/1360937237/mavlinksendtask.o ${OBJECTDIR}/_ext/1360937237/mavlinkstatustask.o ${OBJECTDIR}/_ext/1360937237/simulatorcommsupdatetask.o ${OBJECTDIR}/_ext/1360937237/attitudecontroller.o ${OBJECTDIR}/_ext/1171490990/tasks.o

# Source Files
SOURCEFILES=../MicroDrone-Firmware/HAL/PIC/simulatorComms.c ../MicroDrone-Firmware/HAL/PIC/comms.c ../MicroDrone-Firmware/HAL/PIC/tof.c ../MicroDrone-Firmware/HAL/PIC/motors.c ../MicroDrone-Firmware/HAL/PIC/imu.c ../MicroDrone-Firmware/HAL/SIM/sim.c ../MicroDrone-Firmware/HAL/SimCommsUpdateTask/SimCommsUpdateTask.c ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c ../MicroDrone-Firmware/src/Control/SensFusion.c ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c ../MicroDrone-Firmware/src/Control/PID.c ../MicroDrone-Firmware/src/Control/AttitudeController.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c ../MicroDrone-Firmware/src/Utils/num.c ../MicroDrone-Firmware/libs/Math/Quaternion.c ../MicroDrone-Firmware/libs/Math/Vector3D.c ../src/config/default/peripheral/ocmp/plib_ocmp6.c ../src/config/default/system/console/src/sys_console_usb_cdc.c ../src/config/default/usb/src/usb_device_cdc.c ../src/config/default/usb/src/usb_device_cdc_acm.c ../src/main.c ../src/config/default/initialization.c ../src/config/default/interrupts.c ../src/config/default/interrupts_a.S ../src/config/default/exceptions.c ../src/config/default/stdio/xc32_monitor.c ../src/config/default/peripheral/clk/plib_clk.c ../src/config/default/peripheral/gpio/plib_gpio.c ../src/config/default/peripheral/cache/plib_cache.c ../src/config/default/peripheral/cache/plib_cache_pic32mz.S ../src/config/default/peripheral/evic/plib_evic.c ../src/config/default/peripheral/dmac/plib_dmac.c ../src/config/default/usb_device_init_data.c ../src/config/default/usb/src/usb_device.c ../src/config/default/peripheral/ocmp/plib_ocmp4.c ../src/config/default/peripheral/ocmp/plib_ocmp5.c ../src/config/default/peripheral/ocmp/plib_ocmp3.c ../src/config/default/system/debug/src/sys_debug.c ../src/config/default/system/time/src/sys_time.c ../src/config/default/peripheral/coretimer/plib_coretimer.c ../src/config/default/driver/usb/usbhs/src/drv_usbhs.c ../src/config/default/driver/usb/usbhs/src/drv_usbhs_device.c ../src/config/default/peripheral/uart/plib_uart1.c ../src/config/default/peripheral/tmr/plib_tmr2.c ../src/config/default/system/console/src/sys_console.c ../src/config/default/freertos_hooks.c ../src/third_party/rtos/FreeRTOS/Source/croutine.c ../src/third_party/rtos/FreeRTOS/Source/list.c ../src/third_party/rtos/FreeRTOS/Source/queue.c ../src/third_party/rtos/FreeRTOS/Source/FreeRTOS_tasks.c ../src/third_party/rtos/FreeRTOS/Source/timers.c ../src/third_party/rtos/FreeRTOS/Source/event_groups.c ../src/third_party/rtos/FreeRTOS/Source/stream_buffer.c ../src/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c ../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c ../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S ../src/config/default/system/int/src/sys_int.c ../src/config/default/system/cache/sys_cache.c ../src/config/default/system/dma/sys_dma.c ../src/config/default/osal/osal_freertos.c ../src/blink_led_thread.c ../src/heartbeat_led_task.c ../src/mavlinkrecvtask.c ../src/mavlinksendtask.c ../src/mavlinkstatustask.c ../src/simulatorcommsupdatetask.c ../src/attitudecontroller.c ../src/config/default/tasks.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MZ2048EFM144
MP_LINKER_FILE_OPTION=,--script="../src/config/default/p32MZ2048EFM144.ld"
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1171490990/interrupts_a.o: ../src/config/default/interrupts_a.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts_a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts_a.o 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts_a.o.ok ${OBJECTDIR}/_ext/1171490990/interrupts_a.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/config/default" -MMD -MF "${OBJECTDIR}/_ext/1171490990/interrupts_a.o.d"  -o ${OBJECTDIR}/_ext/1171490990/interrupts_a.o ../src/config/default/interrupts_a.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1171490990/interrupts_a.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,-I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1171490990/interrupts_a.o.d" "${OBJECTDIR}/_ext/1171490990/interrupts_a.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o: ../src/config/default/peripheral/cache/plib_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1984157808" 
	@${RM} ${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/config/default" -MMD -MF "${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o ../src/config/default/peripheral/cache/plib_cache_pic32mz.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,-I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/951553246/port_asm.o: ../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/951553246" 
	@${RM} ${OBJECTDIR}/_ext/951553246/port_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/951553246/port_asm.o 
	@${RM} ${OBJECTDIR}/_ext/951553246/port_asm.o.ok ${OBJECTDIR}/_ext/951553246/port_asm.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/config/default" -MMD -MF "${OBJECTDIR}/_ext/951553246/port_asm.o.d"  -o ${OBJECTDIR}/_ext/951553246/port_asm.o ../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/951553246/port_asm.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,-I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/951553246/port_asm.o.d" "${OBJECTDIR}/_ext/951553246/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/1171490990/interrupts_a.o: ../src/config/default/interrupts_a.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts_a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts_a.o 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts_a.o.ok ${OBJECTDIR}/_ext/1171490990/interrupts_a.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/config/default" -MMD -MF "${OBJECTDIR}/_ext/1171490990/interrupts_a.o.d"  -o ${OBJECTDIR}/_ext/1171490990/interrupts_a.o ../src/config/default/interrupts_a.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1171490990/interrupts_a.o.asm.d",--gdwarf-2,-I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1171490990/interrupts_a.o.d" "${OBJECTDIR}/_ext/1171490990/interrupts_a.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o: ../src/config/default/peripheral/cache/plib_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1984157808" 
	@${RM} ${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/config/default" -MMD -MF "${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o ../src/config/default/peripheral/cache/plib_cache_pic32mz.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o.asm.d",--gdwarf-2,-I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/1984157808/plib_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/951553246/port_asm.o: ../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/951553246" 
	@${RM} ${OBJECTDIR}/_ext/951553246/port_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/951553246/port_asm.o 
	@${RM} ${OBJECTDIR}/_ext/951553246/port_asm.o.ok ${OBJECTDIR}/_ext/951553246/port_asm.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/config/default" -MMD -MF "${OBJECTDIR}/_ext/951553246/port_asm.o.d"  -o ${OBJECTDIR}/_ext/951553246/port_asm.o ../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/951553246/port_asm.o.asm.d",--gdwarf-2,-I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/951553246/port_asm.o.d" "${OBJECTDIR}/_ext/951553246/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/2101238180/simulatorComms.o: ../MicroDrone-Firmware/HAL/PIC/simulatorComms.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2101238180" 
	@${RM} ${OBJECTDIR}/_ext/2101238180/simulatorComms.o.d 
	@${RM} ${OBJECTDIR}/_ext/2101238180/simulatorComms.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/2101238180/simulatorComms.o.d" -o ${OBJECTDIR}/_ext/2101238180/simulatorComms.o ../MicroDrone-Firmware/HAL/PIC/simulatorComms.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/2101238180/simulatorComms.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/2101238180/comms.o: ../MicroDrone-Firmware/HAL/PIC/comms.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2101238180" 
	@${RM} ${OBJECTDIR}/_ext/2101238180/comms.o.d 
	@${RM} ${OBJECTDIR}/_ext/2101238180/comms.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/2101238180/comms.o.d" -o ${OBJECTDIR}/_ext/2101238180/comms.o ../MicroDrone-Firmware/HAL/PIC/comms.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/2101238180/comms.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/2101238180/tof.o: ../MicroDrone-Firmware/HAL/PIC/tof.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2101238180" 
	@${RM} ${OBJECTDIR}/_ext/2101238180/tof.o.d 
	@${RM} ${OBJECTDIR}/_ext/2101238180/tof.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/2101238180/tof.o.d" -o ${OBJECTDIR}/_ext/2101238180/tof.o ../MicroDrone-Firmware/HAL/PIC/tof.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/2101238180/tof.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/2101238180/motors.o: ../MicroDrone-Firmware/HAL/PIC/motors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2101238180" 
	@${RM} ${OBJECTDIR}/_ext/2101238180/motors.o.d 
	@${RM} ${OBJECTDIR}/_ext/2101238180/motors.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/2101238180/motors.o.d" -o ${OBJECTDIR}/_ext/2101238180/motors.o ../MicroDrone-Firmware/HAL/PIC/motors.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/2101238180/motors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/2101238180/imu.o: ../MicroDrone-Firmware/HAL/PIC/imu.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2101238180" 
	@${RM} ${OBJECTDIR}/_ext/2101238180/imu.o.d 
	@${RM} ${OBJECTDIR}/_ext/2101238180/imu.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/2101238180/imu.o.d" -o ${OBJECTDIR}/_ext/2101238180/imu.o ../MicroDrone-Firmware/HAL/PIC/imu.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/2101238180/imu.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/2101241073/sim.o: ../MicroDrone-Firmware/HAL/SIM/sim.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2101241073" 
	@${RM} ${OBJECTDIR}/_ext/2101241073/sim.o.d 
	@${RM} ${OBJECTDIR}/_ext/2101241073/sim.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/2101241073/sim.o.d" -o ${OBJECTDIR}/_ext/2101241073/sim.o ../MicroDrone-Firmware/HAL/SIM/sim.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/2101241073/sim.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/882548988/SimCommsUpdateTask.o: ../MicroDrone-Firmware/HAL/SimCommsUpdateTask/SimCommsUpdateTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/882548988" 
	@${RM} ${OBJECTDIR}/_ext/882548988/SimCommsUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/882548988/SimCommsUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/882548988/SimCommsUpdateTask.o.d" -o ${OBJECTDIR}/_ext/882548988/SimCommsUpdateTask.o ../MicroDrone-Firmware/HAL/SimCommsUpdateTask/SimCommsUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/882548988/SimCommsUpdateTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o: ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1035051164" 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d" -o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o: ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d" -o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/SensFusion.o: ../MicroDrone-Firmware/src/Control/SensFusion.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1204493032/SensFusion.o.d" -o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ../MicroDrone-Firmware/src/Control/SensFusion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/SensFusion.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o: ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d" -o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/PID.o: ../MicroDrone-Firmware/src/Control/PID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PID.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PID.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1204493032/PID.o.d" -o ${OBJECTDIR}/_ext/1204493032/PID.o ../MicroDrone-Firmware/src/Control/PID.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/PID.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/AttitudeController.o: ../MicroDrone-Firmware/src/Control/AttitudeController.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d" -o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ../MicroDrone-Firmware/src/Control/AttitudeController.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/149289479/MAVLinkSender.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o: ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1715884449" 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d" -o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o: ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1313821444/num.o: ../MicroDrone-Firmware/src/Utils/num.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1313821444" 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o.d 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1313821444/num.o.d" -o ${OBJECTDIR}/_ext/1313821444/num.o ../MicroDrone-Firmware/src/Utils/num.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1313821444/num.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/971049567/Quaternion.o: ../MicroDrone-Firmware/libs/Math/Quaternion.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/971049567/Quaternion.o.d" -o ${OBJECTDIR}/_ext/971049567/Quaternion.o ../MicroDrone-Firmware/libs/Math/Quaternion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/971049567/Quaternion.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/971049567/Vector3D.o: ../MicroDrone-Firmware/libs/Math/Vector3D.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/971049567/Vector3D.o.d" -o ${OBJECTDIR}/_ext/971049567/Vector3D.o ../MicroDrone-Firmware/libs/Math/Vector3D.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/971049567/Vector3D.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1865480137/plib_ocmp6.o: ../src/config/default/peripheral/ocmp/plib_ocmp6.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1865480137" 
	@${RM} ${OBJECTDIR}/_ext/1865480137/plib_ocmp6.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865480137/plib_ocmp6.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1865480137/plib_ocmp6.o.d" -o ${OBJECTDIR}/_ext/1865480137/plib_ocmp6.o ../src/config/default/peripheral/ocmp/plib_ocmp6.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1865480137/plib_ocmp6.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1832805299/sys_console_usb_cdc.o: ../src/config/default/system/console/src/sys_console_usb_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1832805299" 
	@${RM} ${OBJECTDIR}/_ext/1832805299/sys_console_usb_cdc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1832805299/sys_console_usb_cdc.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1832805299/sys_console_usb_cdc.o.d" -o ${OBJECTDIR}/_ext/1832805299/sys_console_usb_cdc.o ../src/config/default/system/console/src/sys_console_usb_cdc.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1832805299/sys_console_usb_cdc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/308758920/usb_device_cdc.o: ../src/config/default/usb/src/usb_device_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/308758920" 
	@${RM} ${OBJECTDIR}/_ext/308758920/usb_device_cdc.o.d 
	@${RM} ${OBJECTDIR}/_ext/308758920/usb_device_cdc.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/308758920/usb_device_cdc.o.d" -o ${OBJECTDIR}/_ext/308758920/usb_device_cdc.o ../src/config/default/usb/src/usb_device_cdc.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/308758920/usb_device_cdc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/308758920/usb_device_cdc_acm.o: ../src/config/default/usb/src/usb_device_cdc_acm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/308758920" 
	@${RM} ${OBJECTDIR}/_ext/308758920/usb_device_cdc_acm.o.d 
	@${RM} ${OBJECTDIR}/_ext/308758920/usb_device_cdc_acm.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/308758920/usb_device_cdc_acm.o.d" -o ${OBJECTDIR}/_ext/308758920/usb_device_cdc_acm.o ../src/config/default/usb/src/usb_device_cdc_acm.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/308758920/usb_device_cdc_acm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1171490990/initialization.o: ../src/config/default/initialization.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/initialization.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/initialization.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1171490990/initialization.o.d" -o ${OBJECTDIR}/_ext/1171490990/initialization.o ../src/config/default/initialization.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1171490990/initialization.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1171490990/interrupts.o: ../src/config/default/interrupts.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1171490990/interrupts.o.d" -o ${OBJECTDIR}/_ext/1171490990/interrupts.o ../src/config/default/interrupts.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1171490990/interrupts.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1171490990/exceptions.o: ../src/config/default/exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/exceptions.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1171490990/exceptions.o.d" -o ${OBJECTDIR}/_ext/1171490990/exceptions.o ../src/config/default/exceptions.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1171490990/exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/163028504/xc32_monitor.o: ../src/config/default/stdio/xc32_monitor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/163028504" 
	@${RM} ${OBJECTDIR}/_ext/163028504/xc32_monitor.o.d 
	@${RM} ${OBJECTDIR}/_ext/163028504/xc32_monitor.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/163028504/xc32_monitor.o.d" -o ${OBJECTDIR}/_ext/163028504/xc32_monitor.o ../src/config/default/stdio/xc32_monitor.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/163028504/xc32_monitor.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/60165520/plib_clk.o: ../src/config/default/peripheral/clk/plib_clk.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/60165520" 
	@${RM} ${OBJECTDIR}/_ext/60165520/plib_clk.o.d 
	@${RM} ${OBJECTDIR}/_ext/60165520/plib_clk.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/60165520/plib_clk.o.d" -o ${OBJECTDIR}/_ext/60165520/plib_clk.o ../src/config/default/peripheral/clk/plib_clk.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/60165520/plib_clk.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1865254177/plib_gpio.o: ../src/config/default/peripheral/gpio/plib_gpio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1865254177" 
	@${RM} ${OBJECTDIR}/_ext/1865254177/plib_gpio.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865254177/plib_gpio.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1865254177/plib_gpio.o.d" -o ${OBJECTDIR}/_ext/1865254177/plib_gpio.o ../src/config/default/peripheral/gpio/plib_gpio.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1865254177/plib_gpio.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1984157808/plib_cache.o: ../src/config/default/peripheral/cache/plib_cache.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1984157808" 
	@${RM} ${OBJECTDIR}/_ext/1984157808/plib_cache.o.d 
	@${RM} ${OBJECTDIR}/_ext/1984157808/plib_cache.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1984157808/plib_cache.o.d" -o ${OBJECTDIR}/_ext/1984157808/plib_cache.o ../src/config/default/peripheral/cache/plib_cache.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1984157808/plib_cache.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1865200349/plib_evic.o: ../src/config/default/peripheral/evic/plib_evic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1865200349" 
	@${RM} ${OBJECTDIR}/_ext/1865200349/plib_evic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865200349/plib_evic.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1865200349/plib_evic.o.d" -o ${OBJECTDIR}/_ext/1865200349/plib_evic.o ../src/config/default/peripheral/evic/plib_evic.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1865200349/plib_evic.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1865161661/plib_dmac.o: ../src/config/default/peripheral/dmac/plib_dmac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1865161661" 
	@${RM} ${OBJECTDIR}/_ext/1865161661/plib_dmac.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865161661/plib_dmac.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1865161661/plib_dmac.o.d" -o ${OBJECTDIR}/_ext/1865161661/plib_dmac.o ../src/config/default/peripheral/dmac/plib_dmac.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1865161661/plib_dmac.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1171490990/usb_device_init_data.o: ../src/config/default/usb_device_init_data.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/usb_device_init_data.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/usb_device_init_data.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1171490990/usb_device_init_data.o.d" -o ${OBJECTDIR}/_ext/1171490990/usb_device_init_data.o ../src/config/default/usb_device_init_data.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1171490990/usb_device_init_data.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/308758920/usb_device.o: ../src/config/default/usb/src/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/308758920" 
	@${RM} ${OBJECTDIR}/_ext/308758920/usb_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/308758920/usb_device.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/308758920/usb_device.o.d" -o ${OBJECTDIR}/_ext/308758920/usb_device.o ../src/config/default/usb/src/usb_device.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/308758920/usb_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1865480137/plib_ocmp4.o: ../src/config/default/peripheral/ocmp/plib_ocmp4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1865480137" 
	@${RM} ${OBJECTDIR}/_ext/1865480137/plib_ocmp4.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865480137/plib_ocmp4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1865480137/plib_ocmp4.o.d" -o ${OBJECTDIR}/_ext/1865480137/plib_ocmp4.o ../src/config/default/peripheral/ocmp/plib_ocmp4.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1865480137/plib_ocmp4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1865480137/plib_ocmp5.o: ../src/config/default/peripheral/ocmp/plib_ocmp5.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1865480137" 
	@${RM} ${OBJECTDIR}/_ext/1865480137/plib_ocmp5.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865480137/plib_ocmp5.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1865480137/plib_ocmp5.o.d" -o ${OBJECTDIR}/_ext/1865480137/plib_ocmp5.o ../src/config/default/peripheral/ocmp/plib_ocmp5.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1865480137/plib_ocmp5.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1865480137/plib_ocmp3.o: ../src/config/default/peripheral/ocmp/plib_ocmp3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1865480137" 
	@${RM} ${OBJECTDIR}/_ext/1865480137/plib_ocmp3.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865480137/plib_ocmp3.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1865480137/plib_ocmp3.o.d" -o ${OBJECTDIR}/_ext/1865480137/plib_ocmp3.o ../src/config/default/peripheral/ocmp/plib_ocmp3.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1865480137/plib_ocmp3.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/944882569/sys_debug.o: ../src/config/default/system/debug/src/sys_debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/944882569" 
	@${RM} ${OBJECTDIR}/_ext/944882569/sys_debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/944882569/sys_debug.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/944882569/sys_debug.o.d" -o ${OBJECTDIR}/_ext/944882569/sys_debug.o ../src/config/default/system/debug/src/sys_debug.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/944882569/sys_debug.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/101884895/sys_time.o: ../src/config/default/system/time/src/sys_time.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/101884895" 
	@${RM} ${OBJECTDIR}/_ext/101884895/sys_time.o.d 
	@${RM} ${OBJECTDIR}/_ext/101884895/sys_time.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/101884895/sys_time.o.d" -o ${OBJECTDIR}/_ext/101884895/sys_time.o ../src/config/default/system/time/src/sys_time.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/101884895/sys_time.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1249264884/plib_coretimer.o: ../src/config/default/peripheral/coretimer/plib_coretimer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1249264884" 
	@${RM} ${OBJECTDIR}/_ext/1249264884/plib_coretimer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1249264884/plib_coretimer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1249264884/plib_coretimer.o.d" -o ${OBJECTDIR}/_ext/1249264884/plib_coretimer.o ../src/config/default/peripheral/coretimer/plib_coretimer.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1249264884/plib_coretimer.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/2071311437/drv_usbhs.o: ../src/config/default/driver/usb/usbhs/src/drv_usbhs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2071311437" 
	@${RM} ${OBJECTDIR}/_ext/2071311437/drv_usbhs.o.d 
	@${RM} ${OBJECTDIR}/_ext/2071311437/drv_usbhs.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/2071311437/drv_usbhs.o.d" -o ${OBJECTDIR}/_ext/2071311437/drv_usbhs.o ../src/config/default/driver/usb/usbhs/src/drv_usbhs.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/2071311437/drv_usbhs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/2071311437/drv_usbhs_device.o: ../src/config/default/driver/usb/usbhs/src/drv_usbhs_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2071311437" 
	@${RM} ${OBJECTDIR}/_ext/2071311437/drv_usbhs_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/2071311437/drv_usbhs_device.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/2071311437/drv_usbhs_device.o.d" -o ${OBJECTDIR}/_ext/2071311437/drv_usbhs_device.o ../src/config/default/driver/usb/usbhs/src/drv_usbhs_device.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/2071311437/drv_usbhs_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1865657120/plib_uart1.o: ../src/config/default/peripheral/uart/plib_uart1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1865657120" 
	@${RM} ${OBJECTDIR}/_ext/1865657120/plib_uart1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865657120/plib_uart1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1865657120/plib_uart1.o.d" -o ${OBJECTDIR}/_ext/1865657120/plib_uart1.o ../src/config/default/peripheral/uart/plib_uart1.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1865657120/plib_uart1.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/60181895/plib_tmr2.o: ../src/config/default/peripheral/tmr/plib_tmr2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/60181895" 
	@${RM} ${OBJECTDIR}/_ext/60181895/plib_tmr2.o.d 
	@${RM} ${OBJECTDIR}/_ext/60181895/plib_tmr2.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/60181895/plib_tmr2.o.d" -o ${OBJECTDIR}/_ext/60181895/plib_tmr2.o ../src/config/default/peripheral/tmr/plib_tmr2.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/60181895/plib_tmr2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1832805299/sys_console.o: ../src/config/default/system/console/src/sys_console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1832805299" 
	@${RM} ${OBJECTDIR}/_ext/1832805299/sys_console.o.d 
	@${RM} ${OBJECTDIR}/_ext/1832805299/sys_console.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1832805299/sys_console.o.d" -o ${OBJECTDIR}/_ext/1832805299/sys_console.o ../src/config/default/system/console/src/sys_console.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1832805299/sys_console.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1171490990/freertos_hooks.o: ../src/config/default/freertos_hooks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/freertos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/freertos_hooks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1171490990/freertos_hooks.o.d" -o ${OBJECTDIR}/_ext/1171490990/freertos_hooks.o ../src/config/default/freertos_hooks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1171490990/freertos_hooks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/404212886/croutine.o: ../src/third_party/rtos/FreeRTOS/Source/croutine.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/croutine.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/404212886/croutine.o.d" -o ${OBJECTDIR}/_ext/404212886/croutine.o ../src/third_party/rtos/FreeRTOS/Source/croutine.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/404212886/croutine.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/404212886/list.o: ../src/third_party/rtos/FreeRTOS/Source/list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/list.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/404212886/list.o.d" -o ${OBJECTDIR}/_ext/404212886/list.o ../src/third_party/rtos/FreeRTOS/Source/list.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/404212886/list.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/404212886/queue.o: ../src/third_party/rtos/FreeRTOS/Source/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/queue.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/404212886/queue.o.d" -o ${OBJECTDIR}/_ext/404212886/queue.o ../src/third_party/rtos/FreeRTOS/Source/queue.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/404212886/queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o: ../src/third_party/rtos/FreeRTOS/Source/FreeRTOS_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o.d" -o ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o ../src/third_party/rtos/FreeRTOS/Source/FreeRTOS_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/404212886/timers.o: ../src/third_party/rtos/FreeRTOS/Source/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/404212886/timers.o.d" -o ${OBJECTDIR}/_ext/404212886/timers.o ../src/third_party/rtos/FreeRTOS/Source/timers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/404212886/timers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/404212886/event_groups.o: ../src/third_party/rtos/FreeRTOS/Source/event_groups.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/event_groups.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/404212886/event_groups.o.d" -o ${OBJECTDIR}/_ext/404212886/event_groups.o ../src/third_party/rtos/FreeRTOS/Source/event_groups.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/404212886/event_groups.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/404212886/stream_buffer.o: ../src/third_party/rtos/FreeRTOS/Source/stream_buffer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/stream_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/stream_buffer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/404212886/stream_buffer.o.d" -o ${OBJECTDIR}/_ext/404212886/stream_buffer.o ../src/third_party/rtos/FreeRTOS/Source/stream_buffer.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/404212886/stream_buffer.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1665200909/heap_1.o: ../src/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1665200909" 
	@${RM} ${OBJECTDIR}/_ext/1665200909/heap_1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1665200909/heap_1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1665200909/heap_1.o.d" -o ${OBJECTDIR}/_ext/1665200909/heap_1.o ../src/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1665200909/heap_1.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/951553246/port.o: ../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/951553246" 
	@${RM} ${OBJECTDIR}/_ext/951553246/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/951553246/port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/951553246/port.o.d" -o ${OBJECTDIR}/_ext/951553246/port.o ../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/951553246/port.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1881668453/sys_int.o: ../src/config/default/system/int/src/sys_int.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1881668453" 
	@${RM} ${OBJECTDIR}/_ext/1881668453/sys_int.o.d 
	@${RM} ${OBJECTDIR}/_ext/1881668453/sys_int.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1881668453/sys_int.o.d" -o ${OBJECTDIR}/_ext/1881668453/sys_int.o ../src/config/default/system/int/src/sys_int.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1881668453/sys_int.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1014039709/sys_cache.o: ../src/config/default/system/cache/sys_cache.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1014039709" 
	@${RM} ${OBJECTDIR}/_ext/1014039709/sys_cache.o.d 
	@${RM} ${OBJECTDIR}/_ext/1014039709/sys_cache.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1014039709/sys_cache.o.d" -o ${OBJECTDIR}/_ext/1014039709/sys_cache.o ../src/config/default/system/cache/sys_cache.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1014039709/sys_cache.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/14461671/sys_dma.o: ../src/config/default/system/dma/sys_dma.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/14461671" 
	@${RM} ${OBJECTDIR}/_ext/14461671/sys_dma.o.d 
	@${RM} ${OBJECTDIR}/_ext/14461671/sys_dma.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/14461671/sys_dma.o.d" -o ${OBJECTDIR}/_ext/14461671/sys_dma.o ../src/config/default/system/dma/sys_dma.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/14461671/sys_dma.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1529399856/osal_freertos.o: ../src/config/default/osal/osal_freertos.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1529399856" 
	@${RM} ${OBJECTDIR}/_ext/1529399856/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/1529399856/osal_freertos.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1529399856/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/1529399856/osal_freertos.o ../src/config/default/osal/osal_freertos.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1529399856/osal_freertos.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/blink_led_thread.o: ../src/blink_led_thread.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/blink_led_thread.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/blink_led_thread.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/blink_led_thread.o.d" -o ${OBJECTDIR}/_ext/1360937237/blink_led_thread.o ../src/blink_led_thread.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/blink_led_thread.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/heartbeat_led_task.o: ../src/heartbeat_led_task.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/heartbeat_led_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/heartbeat_led_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/heartbeat_led_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/heartbeat_led_task.o ../src/heartbeat_led_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/heartbeat_led_task.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/mavlinkrecvtask.o: ../src/mavlinkrecvtask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlinkrecvtask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlinkrecvtask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlinkrecvtask.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlinkrecvtask.o ../src/mavlinkrecvtask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/mavlinkrecvtask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/mavlinksendtask.o: ../src/mavlinksendtask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlinksendtask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlinksendtask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlinksendtask.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlinksendtask.o ../src/mavlinksendtask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/mavlinksendtask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/mavlinkstatustask.o: ../src/mavlinkstatustask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlinkstatustask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlinkstatustask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlinkstatustask.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlinkstatustask.o ../src/mavlinkstatustask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/mavlinkstatustask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/simulatorcommsupdatetask.o: ../src/simulatorcommsupdatetask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/simulatorcommsupdatetask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/simulatorcommsupdatetask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/simulatorcommsupdatetask.o.d" -o ${OBJECTDIR}/_ext/1360937237/simulatorcommsupdatetask.o ../src/simulatorcommsupdatetask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/simulatorcommsupdatetask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/attitudecontroller.o: ../src/attitudecontroller.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/attitudecontroller.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/attitudecontroller.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/attitudecontroller.o.d" -o ${OBJECTDIR}/_ext/1360937237/attitudecontroller.o ../src/attitudecontroller.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/attitudecontroller.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1171490990/tasks.o: ../src/config/default/tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1171490990/tasks.o.d" -o ${OBJECTDIR}/_ext/1171490990/tasks.o ../src/config/default/tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1171490990/tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/2101238180/simulatorComms.o: ../MicroDrone-Firmware/HAL/PIC/simulatorComms.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2101238180" 
	@${RM} ${OBJECTDIR}/_ext/2101238180/simulatorComms.o.d 
	@${RM} ${OBJECTDIR}/_ext/2101238180/simulatorComms.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/2101238180/simulatorComms.o.d" -o ${OBJECTDIR}/_ext/2101238180/simulatorComms.o ../MicroDrone-Firmware/HAL/PIC/simulatorComms.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/2101238180/simulatorComms.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/2101238180/comms.o: ../MicroDrone-Firmware/HAL/PIC/comms.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2101238180" 
	@${RM} ${OBJECTDIR}/_ext/2101238180/comms.o.d 
	@${RM} ${OBJECTDIR}/_ext/2101238180/comms.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/2101238180/comms.o.d" -o ${OBJECTDIR}/_ext/2101238180/comms.o ../MicroDrone-Firmware/HAL/PIC/comms.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/2101238180/comms.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/2101238180/tof.o: ../MicroDrone-Firmware/HAL/PIC/tof.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2101238180" 
	@${RM} ${OBJECTDIR}/_ext/2101238180/tof.o.d 
	@${RM} ${OBJECTDIR}/_ext/2101238180/tof.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/2101238180/tof.o.d" -o ${OBJECTDIR}/_ext/2101238180/tof.o ../MicroDrone-Firmware/HAL/PIC/tof.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/2101238180/tof.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/2101238180/motors.o: ../MicroDrone-Firmware/HAL/PIC/motors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2101238180" 
	@${RM} ${OBJECTDIR}/_ext/2101238180/motors.o.d 
	@${RM} ${OBJECTDIR}/_ext/2101238180/motors.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/2101238180/motors.o.d" -o ${OBJECTDIR}/_ext/2101238180/motors.o ../MicroDrone-Firmware/HAL/PIC/motors.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/2101238180/motors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/2101238180/imu.o: ../MicroDrone-Firmware/HAL/PIC/imu.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2101238180" 
	@${RM} ${OBJECTDIR}/_ext/2101238180/imu.o.d 
	@${RM} ${OBJECTDIR}/_ext/2101238180/imu.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/2101238180/imu.o.d" -o ${OBJECTDIR}/_ext/2101238180/imu.o ../MicroDrone-Firmware/HAL/PIC/imu.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/2101238180/imu.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/2101241073/sim.o: ../MicroDrone-Firmware/HAL/SIM/sim.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2101241073" 
	@${RM} ${OBJECTDIR}/_ext/2101241073/sim.o.d 
	@${RM} ${OBJECTDIR}/_ext/2101241073/sim.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/2101241073/sim.o.d" -o ${OBJECTDIR}/_ext/2101241073/sim.o ../MicroDrone-Firmware/HAL/SIM/sim.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/2101241073/sim.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/882548988/SimCommsUpdateTask.o: ../MicroDrone-Firmware/HAL/SimCommsUpdateTask/SimCommsUpdateTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/882548988" 
	@${RM} ${OBJECTDIR}/_ext/882548988/SimCommsUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/882548988/SimCommsUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/882548988/SimCommsUpdateTask.o.d" -o ${OBJECTDIR}/_ext/882548988/SimCommsUpdateTask.o ../MicroDrone-Firmware/HAL/SimCommsUpdateTask/SimCommsUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/882548988/SimCommsUpdateTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o: ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1035051164" 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d" -o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o: ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d" -o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/SensFusion.o: ../MicroDrone-Firmware/src/Control/SensFusion.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1204493032/SensFusion.o.d" -o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ../MicroDrone-Firmware/src/Control/SensFusion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/SensFusion.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o: ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d" -o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/PID.o: ../MicroDrone-Firmware/src/Control/PID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PID.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PID.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1204493032/PID.o.d" -o ${OBJECTDIR}/_ext/1204493032/PID.o ../MicroDrone-Firmware/src/Control/PID.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/PID.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/AttitudeController.o: ../MicroDrone-Firmware/src/Control/AttitudeController.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d" -o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ../MicroDrone-Firmware/src/Control/AttitudeController.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/149289479/MAVLinkSender.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o: ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1715884449" 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d" -o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o: ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1313821444/num.o: ../MicroDrone-Firmware/src/Utils/num.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1313821444" 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o.d 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1313821444/num.o.d" -o ${OBJECTDIR}/_ext/1313821444/num.o ../MicroDrone-Firmware/src/Utils/num.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1313821444/num.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/971049567/Quaternion.o: ../MicroDrone-Firmware/libs/Math/Quaternion.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/971049567/Quaternion.o.d" -o ${OBJECTDIR}/_ext/971049567/Quaternion.o ../MicroDrone-Firmware/libs/Math/Quaternion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/971049567/Quaternion.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/971049567/Vector3D.o: ../MicroDrone-Firmware/libs/Math/Vector3D.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/971049567/Vector3D.o.d" -o ${OBJECTDIR}/_ext/971049567/Vector3D.o ../MicroDrone-Firmware/libs/Math/Vector3D.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/971049567/Vector3D.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1865480137/plib_ocmp6.o: ../src/config/default/peripheral/ocmp/plib_ocmp6.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1865480137" 
	@${RM} ${OBJECTDIR}/_ext/1865480137/plib_ocmp6.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865480137/plib_ocmp6.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1865480137/plib_ocmp6.o.d" -o ${OBJECTDIR}/_ext/1865480137/plib_ocmp6.o ../src/config/default/peripheral/ocmp/plib_ocmp6.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1865480137/plib_ocmp6.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1832805299/sys_console_usb_cdc.o: ../src/config/default/system/console/src/sys_console_usb_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1832805299" 
	@${RM} ${OBJECTDIR}/_ext/1832805299/sys_console_usb_cdc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1832805299/sys_console_usb_cdc.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1832805299/sys_console_usb_cdc.o.d" -o ${OBJECTDIR}/_ext/1832805299/sys_console_usb_cdc.o ../src/config/default/system/console/src/sys_console_usb_cdc.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1832805299/sys_console_usb_cdc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/308758920/usb_device_cdc.o: ../src/config/default/usb/src/usb_device_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/308758920" 
	@${RM} ${OBJECTDIR}/_ext/308758920/usb_device_cdc.o.d 
	@${RM} ${OBJECTDIR}/_ext/308758920/usb_device_cdc.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/308758920/usb_device_cdc.o.d" -o ${OBJECTDIR}/_ext/308758920/usb_device_cdc.o ../src/config/default/usb/src/usb_device_cdc.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/308758920/usb_device_cdc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/308758920/usb_device_cdc_acm.o: ../src/config/default/usb/src/usb_device_cdc_acm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/308758920" 
	@${RM} ${OBJECTDIR}/_ext/308758920/usb_device_cdc_acm.o.d 
	@${RM} ${OBJECTDIR}/_ext/308758920/usb_device_cdc_acm.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/308758920/usb_device_cdc_acm.o.d" -o ${OBJECTDIR}/_ext/308758920/usb_device_cdc_acm.o ../src/config/default/usb/src/usb_device_cdc_acm.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/308758920/usb_device_cdc_acm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1171490990/initialization.o: ../src/config/default/initialization.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/initialization.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/initialization.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1171490990/initialization.o.d" -o ${OBJECTDIR}/_ext/1171490990/initialization.o ../src/config/default/initialization.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1171490990/initialization.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1171490990/interrupts.o: ../src/config/default/interrupts.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1171490990/interrupts.o.d" -o ${OBJECTDIR}/_ext/1171490990/interrupts.o ../src/config/default/interrupts.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1171490990/interrupts.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1171490990/exceptions.o: ../src/config/default/exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/exceptions.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1171490990/exceptions.o.d" -o ${OBJECTDIR}/_ext/1171490990/exceptions.o ../src/config/default/exceptions.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1171490990/exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/163028504/xc32_monitor.o: ../src/config/default/stdio/xc32_monitor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/163028504" 
	@${RM} ${OBJECTDIR}/_ext/163028504/xc32_monitor.o.d 
	@${RM} ${OBJECTDIR}/_ext/163028504/xc32_monitor.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/163028504/xc32_monitor.o.d" -o ${OBJECTDIR}/_ext/163028504/xc32_monitor.o ../src/config/default/stdio/xc32_monitor.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/163028504/xc32_monitor.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/60165520/plib_clk.o: ../src/config/default/peripheral/clk/plib_clk.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/60165520" 
	@${RM} ${OBJECTDIR}/_ext/60165520/plib_clk.o.d 
	@${RM} ${OBJECTDIR}/_ext/60165520/plib_clk.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/60165520/plib_clk.o.d" -o ${OBJECTDIR}/_ext/60165520/plib_clk.o ../src/config/default/peripheral/clk/plib_clk.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/60165520/plib_clk.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1865254177/plib_gpio.o: ../src/config/default/peripheral/gpio/plib_gpio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1865254177" 
	@${RM} ${OBJECTDIR}/_ext/1865254177/plib_gpio.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865254177/plib_gpio.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1865254177/plib_gpio.o.d" -o ${OBJECTDIR}/_ext/1865254177/plib_gpio.o ../src/config/default/peripheral/gpio/plib_gpio.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1865254177/plib_gpio.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1984157808/plib_cache.o: ../src/config/default/peripheral/cache/plib_cache.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1984157808" 
	@${RM} ${OBJECTDIR}/_ext/1984157808/plib_cache.o.d 
	@${RM} ${OBJECTDIR}/_ext/1984157808/plib_cache.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1984157808/plib_cache.o.d" -o ${OBJECTDIR}/_ext/1984157808/plib_cache.o ../src/config/default/peripheral/cache/plib_cache.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1984157808/plib_cache.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1865200349/plib_evic.o: ../src/config/default/peripheral/evic/plib_evic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1865200349" 
	@${RM} ${OBJECTDIR}/_ext/1865200349/plib_evic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865200349/plib_evic.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1865200349/plib_evic.o.d" -o ${OBJECTDIR}/_ext/1865200349/plib_evic.o ../src/config/default/peripheral/evic/plib_evic.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1865200349/plib_evic.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1865161661/plib_dmac.o: ../src/config/default/peripheral/dmac/plib_dmac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1865161661" 
	@${RM} ${OBJECTDIR}/_ext/1865161661/plib_dmac.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865161661/plib_dmac.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1865161661/plib_dmac.o.d" -o ${OBJECTDIR}/_ext/1865161661/plib_dmac.o ../src/config/default/peripheral/dmac/plib_dmac.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1865161661/plib_dmac.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1171490990/usb_device_init_data.o: ../src/config/default/usb_device_init_data.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/usb_device_init_data.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/usb_device_init_data.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1171490990/usb_device_init_data.o.d" -o ${OBJECTDIR}/_ext/1171490990/usb_device_init_data.o ../src/config/default/usb_device_init_data.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1171490990/usb_device_init_data.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/308758920/usb_device.o: ../src/config/default/usb/src/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/308758920" 
	@${RM} ${OBJECTDIR}/_ext/308758920/usb_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/308758920/usb_device.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/308758920/usb_device.o.d" -o ${OBJECTDIR}/_ext/308758920/usb_device.o ../src/config/default/usb/src/usb_device.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/308758920/usb_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1865480137/plib_ocmp4.o: ../src/config/default/peripheral/ocmp/plib_ocmp4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1865480137" 
	@${RM} ${OBJECTDIR}/_ext/1865480137/plib_ocmp4.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865480137/plib_ocmp4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1865480137/plib_ocmp4.o.d" -o ${OBJECTDIR}/_ext/1865480137/plib_ocmp4.o ../src/config/default/peripheral/ocmp/plib_ocmp4.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1865480137/plib_ocmp4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1865480137/plib_ocmp5.o: ../src/config/default/peripheral/ocmp/plib_ocmp5.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1865480137" 
	@${RM} ${OBJECTDIR}/_ext/1865480137/plib_ocmp5.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865480137/plib_ocmp5.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1865480137/plib_ocmp5.o.d" -o ${OBJECTDIR}/_ext/1865480137/plib_ocmp5.o ../src/config/default/peripheral/ocmp/plib_ocmp5.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1865480137/plib_ocmp5.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1865480137/plib_ocmp3.o: ../src/config/default/peripheral/ocmp/plib_ocmp3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1865480137" 
	@${RM} ${OBJECTDIR}/_ext/1865480137/plib_ocmp3.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865480137/plib_ocmp3.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1865480137/plib_ocmp3.o.d" -o ${OBJECTDIR}/_ext/1865480137/plib_ocmp3.o ../src/config/default/peripheral/ocmp/plib_ocmp3.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1865480137/plib_ocmp3.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/944882569/sys_debug.o: ../src/config/default/system/debug/src/sys_debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/944882569" 
	@${RM} ${OBJECTDIR}/_ext/944882569/sys_debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/944882569/sys_debug.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/944882569/sys_debug.o.d" -o ${OBJECTDIR}/_ext/944882569/sys_debug.o ../src/config/default/system/debug/src/sys_debug.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/944882569/sys_debug.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/101884895/sys_time.o: ../src/config/default/system/time/src/sys_time.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/101884895" 
	@${RM} ${OBJECTDIR}/_ext/101884895/sys_time.o.d 
	@${RM} ${OBJECTDIR}/_ext/101884895/sys_time.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/101884895/sys_time.o.d" -o ${OBJECTDIR}/_ext/101884895/sys_time.o ../src/config/default/system/time/src/sys_time.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/101884895/sys_time.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1249264884/plib_coretimer.o: ../src/config/default/peripheral/coretimer/plib_coretimer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1249264884" 
	@${RM} ${OBJECTDIR}/_ext/1249264884/plib_coretimer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1249264884/plib_coretimer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1249264884/plib_coretimer.o.d" -o ${OBJECTDIR}/_ext/1249264884/plib_coretimer.o ../src/config/default/peripheral/coretimer/plib_coretimer.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1249264884/plib_coretimer.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/2071311437/drv_usbhs.o: ../src/config/default/driver/usb/usbhs/src/drv_usbhs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2071311437" 
	@${RM} ${OBJECTDIR}/_ext/2071311437/drv_usbhs.o.d 
	@${RM} ${OBJECTDIR}/_ext/2071311437/drv_usbhs.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/2071311437/drv_usbhs.o.d" -o ${OBJECTDIR}/_ext/2071311437/drv_usbhs.o ../src/config/default/driver/usb/usbhs/src/drv_usbhs.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/2071311437/drv_usbhs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/2071311437/drv_usbhs_device.o: ../src/config/default/driver/usb/usbhs/src/drv_usbhs_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2071311437" 
	@${RM} ${OBJECTDIR}/_ext/2071311437/drv_usbhs_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/2071311437/drv_usbhs_device.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/2071311437/drv_usbhs_device.o.d" -o ${OBJECTDIR}/_ext/2071311437/drv_usbhs_device.o ../src/config/default/driver/usb/usbhs/src/drv_usbhs_device.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/2071311437/drv_usbhs_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1865657120/plib_uart1.o: ../src/config/default/peripheral/uart/plib_uart1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1865657120" 
	@${RM} ${OBJECTDIR}/_ext/1865657120/plib_uart1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865657120/plib_uart1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1865657120/plib_uart1.o.d" -o ${OBJECTDIR}/_ext/1865657120/plib_uart1.o ../src/config/default/peripheral/uart/plib_uart1.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1865657120/plib_uart1.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/60181895/plib_tmr2.o: ../src/config/default/peripheral/tmr/plib_tmr2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/60181895" 
	@${RM} ${OBJECTDIR}/_ext/60181895/plib_tmr2.o.d 
	@${RM} ${OBJECTDIR}/_ext/60181895/plib_tmr2.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/60181895/plib_tmr2.o.d" -o ${OBJECTDIR}/_ext/60181895/plib_tmr2.o ../src/config/default/peripheral/tmr/plib_tmr2.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/60181895/plib_tmr2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1832805299/sys_console.o: ../src/config/default/system/console/src/sys_console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1832805299" 
	@${RM} ${OBJECTDIR}/_ext/1832805299/sys_console.o.d 
	@${RM} ${OBJECTDIR}/_ext/1832805299/sys_console.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1832805299/sys_console.o.d" -o ${OBJECTDIR}/_ext/1832805299/sys_console.o ../src/config/default/system/console/src/sys_console.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1832805299/sys_console.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1171490990/freertos_hooks.o: ../src/config/default/freertos_hooks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/freertos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/freertos_hooks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1171490990/freertos_hooks.o.d" -o ${OBJECTDIR}/_ext/1171490990/freertos_hooks.o ../src/config/default/freertos_hooks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1171490990/freertos_hooks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/404212886/croutine.o: ../src/third_party/rtos/FreeRTOS/Source/croutine.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/croutine.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/404212886/croutine.o.d" -o ${OBJECTDIR}/_ext/404212886/croutine.o ../src/third_party/rtos/FreeRTOS/Source/croutine.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/404212886/croutine.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/404212886/list.o: ../src/third_party/rtos/FreeRTOS/Source/list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/list.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/404212886/list.o.d" -o ${OBJECTDIR}/_ext/404212886/list.o ../src/third_party/rtos/FreeRTOS/Source/list.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/404212886/list.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/404212886/queue.o: ../src/third_party/rtos/FreeRTOS/Source/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/queue.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/404212886/queue.o.d" -o ${OBJECTDIR}/_ext/404212886/queue.o ../src/third_party/rtos/FreeRTOS/Source/queue.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/404212886/queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o: ../src/third_party/rtos/FreeRTOS/Source/FreeRTOS_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o.d" -o ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o ../src/third_party/rtos/FreeRTOS/Source/FreeRTOS_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/404212886/timers.o: ../src/third_party/rtos/FreeRTOS/Source/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/404212886/timers.o.d" -o ${OBJECTDIR}/_ext/404212886/timers.o ../src/third_party/rtos/FreeRTOS/Source/timers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/404212886/timers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/404212886/event_groups.o: ../src/third_party/rtos/FreeRTOS/Source/event_groups.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/event_groups.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/404212886/event_groups.o.d" -o ${OBJECTDIR}/_ext/404212886/event_groups.o ../src/third_party/rtos/FreeRTOS/Source/event_groups.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/404212886/event_groups.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/404212886/stream_buffer.o: ../src/third_party/rtos/FreeRTOS/Source/stream_buffer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/stream_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/stream_buffer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/404212886/stream_buffer.o.d" -o ${OBJECTDIR}/_ext/404212886/stream_buffer.o ../src/third_party/rtos/FreeRTOS/Source/stream_buffer.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/404212886/stream_buffer.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1665200909/heap_1.o: ../src/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1665200909" 
	@${RM} ${OBJECTDIR}/_ext/1665200909/heap_1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1665200909/heap_1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1665200909/heap_1.o.d" -o ${OBJECTDIR}/_ext/1665200909/heap_1.o ../src/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1665200909/heap_1.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/951553246/port.o: ../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/951553246" 
	@${RM} ${OBJECTDIR}/_ext/951553246/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/951553246/port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/951553246/port.o.d" -o ${OBJECTDIR}/_ext/951553246/port.o ../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/951553246/port.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1881668453/sys_int.o: ../src/config/default/system/int/src/sys_int.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1881668453" 
	@${RM} ${OBJECTDIR}/_ext/1881668453/sys_int.o.d 
	@${RM} ${OBJECTDIR}/_ext/1881668453/sys_int.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1881668453/sys_int.o.d" -o ${OBJECTDIR}/_ext/1881668453/sys_int.o ../src/config/default/system/int/src/sys_int.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1881668453/sys_int.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1014039709/sys_cache.o: ../src/config/default/system/cache/sys_cache.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1014039709" 
	@${RM} ${OBJECTDIR}/_ext/1014039709/sys_cache.o.d 
	@${RM} ${OBJECTDIR}/_ext/1014039709/sys_cache.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1014039709/sys_cache.o.d" -o ${OBJECTDIR}/_ext/1014039709/sys_cache.o ../src/config/default/system/cache/sys_cache.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1014039709/sys_cache.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/14461671/sys_dma.o: ../src/config/default/system/dma/sys_dma.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/14461671" 
	@${RM} ${OBJECTDIR}/_ext/14461671/sys_dma.o.d 
	@${RM} ${OBJECTDIR}/_ext/14461671/sys_dma.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/14461671/sys_dma.o.d" -o ${OBJECTDIR}/_ext/14461671/sys_dma.o ../src/config/default/system/dma/sys_dma.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/14461671/sys_dma.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1529399856/osal_freertos.o: ../src/config/default/osal/osal_freertos.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1529399856" 
	@${RM} ${OBJECTDIR}/_ext/1529399856/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/1529399856/osal_freertos.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1529399856/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/1529399856/osal_freertos.o ../src/config/default/osal/osal_freertos.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1529399856/osal_freertos.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/blink_led_thread.o: ../src/blink_led_thread.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/blink_led_thread.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/blink_led_thread.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/blink_led_thread.o.d" -o ${OBJECTDIR}/_ext/1360937237/blink_led_thread.o ../src/blink_led_thread.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/blink_led_thread.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/heartbeat_led_task.o: ../src/heartbeat_led_task.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/heartbeat_led_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/heartbeat_led_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/heartbeat_led_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/heartbeat_led_task.o ../src/heartbeat_led_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/heartbeat_led_task.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/mavlinkrecvtask.o: ../src/mavlinkrecvtask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlinkrecvtask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlinkrecvtask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlinkrecvtask.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlinkrecvtask.o ../src/mavlinkrecvtask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/mavlinkrecvtask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/mavlinksendtask.o: ../src/mavlinksendtask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlinksendtask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlinksendtask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlinksendtask.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlinksendtask.o ../src/mavlinksendtask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/mavlinksendtask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/mavlinkstatustask.o: ../src/mavlinkstatustask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlinkstatustask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlinkstatustask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlinkstatustask.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlinkstatustask.o ../src/mavlinkstatustask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/mavlinkstatustask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/simulatorcommsupdatetask.o: ../src/simulatorcommsupdatetask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/simulatorcommsupdatetask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/simulatorcommsupdatetask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/simulatorcommsupdatetask.o.d" -o ${OBJECTDIR}/_ext/1360937237/simulatorcommsupdatetask.o ../src/simulatorcommsupdatetask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/simulatorcommsupdatetask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/attitudecontroller.o: ../src/attitudecontroller.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/attitudecontroller.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/attitudecontroller.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/attitudecontroller.o.d" -o ${OBJECTDIR}/_ext/1360937237/attitudecontroller.o ../src/attitudecontroller.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/attitudecontroller.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1171490990/tasks.o: ../src/config/default/tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src" -I"../src/config/default" -I"../src/packs/PIC32MZ2048EFM144_DFP" -I"../MicroDrone-Firmware/src" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/libs/MAVLinkV2/standard" -I"../MicroDrone-Firmware/HAL" -Wall -MMD -MF "${OBJECTDIR}/_ext/1171490990/tasks.o.d" -o ${OBJECTDIR}/_ext/1171490990/tasks.o ../src/config/default/tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1171490990/tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    ../src/config/default/p32MZ2048EFM144.ld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g   -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x37F   -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=_min_heap_size=1024,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   ../src/config/default/p32MZ2048EFM144.ld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=1024,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	${MP_CC_DIR}/xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
