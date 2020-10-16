/*******************************************************************************
  System Interrupts File

  Company:
    Microchip Technology Inc.

  File Name:
    interrupt_a.S

  Summary:
    Raw ISR definitions.

  Description:
    This file maps all the interrupt vectors to their corresponding
    implementations. If a particular module interrupt is used, then its ISR
    definition can be found in corresponding PLIB source file. If a module
    interrupt is not used, then its ISR implementation is mapped to dummy
    handler.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
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

#include <xc.h>
#include "ISR_Support.h"

    .extern  CORE_TIMER_Handler

    .section   .vector_0,code, keep
    .equ     __vector_dispatch_0, IntVectorCORE_TIMER_Handler
    .global  __vector_dispatch_0
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorCORE_TIMER_Handler

IntVectorCORE_TIMER_Handler:
    portSAVE_CONTEXT
    la    s6,  CORE_TIMER_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorCORE_TIMER_Handler
    .extern  TIMER_2_Handler

    .section   .vector_9,code, keep
    .equ     __vector_dispatch_9, IntVectorTIMER_2_Handler
    .global  __vector_dispatch_9
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorTIMER_2_Handler

IntVectorTIMER_2_Handler:
    portSAVE_CONTEXT
    la    s6,  TIMER_2_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorTIMER_2_Handler
    .extern  OUTPUT_COMPARE_3_Handler

    .section   .vector_17,code, keep
    .equ     __vector_dispatch_17, IntVectorOUTPUT_COMPARE_3_Handler
    .global  __vector_dispatch_17
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorOUTPUT_COMPARE_3_Handler

IntVectorOUTPUT_COMPARE_3_Handler:
    portSAVE_CONTEXT
    la    s6,  OUTPUT_COMPARE_3_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorOUTPUT_COMPARE_3_Handler
    .extern  OUTPUT_COMPARE_4_Handler

    .section   .vector_22,code, keep
    .equ     __vector_dispatch_22, IntVectorOUTPUT_COMPARE_4_Handler
    .global  __vector_dispatch_22
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorOUTPUT_COMPARE_4_Handler

IntVectorOUTPUT_COMPARE_4_Handler:
    portSAVE_CONTEXT
    la    s6,  OUTPUT_COMPARE_4_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorOUTPUT_COMPARE_4_Handler
    .extern  OUTPUT_COMPARE_5_Handler

    .section   .vector_27,code, keep
    .equ     __vector_dispatch_27, IntVectorOUTPUT_COMPARE_5_Handler
    .global  __vector_dispatch_27
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorOUTPUT_COMPARE_5_Handler

IntVectorOUTPUT_COMPARE_5_Handler:
    portSAVE_CONTEXT
    la    s6,  OUTPUT_COMPARE_5_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorOUTPUT_COMPARE_5_Handler
    .extern  OUTPUT_COMPARE_6_Handler

    .section   .vector_31,code, keep
    .equ     __vector_dispatch_31, IntVectorOUTPUT_COMPARE_6_Handler
    .global  __vector_dispatch_31
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorOUTPUT_COMPARE_6_Handler

IntVectorOUTPUT_COMPARE_6_Handler:
    portSAVE_CONTEXT
    la    s6,  OUTPUT_COMPARE_6_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorOUTPUT_COMPARE_6_Handler
    .extern  UART1_FAULT_Handler

    .section   .vector_112,code, keep
    .equ     __vector_dispatch_112, IntVectorUART1_FAULT_Handler
    .global  __vector_dispatch_112
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorUART1_FAULT_Handler

IntVectorUART1_FAULT_Handler:
    portSAVE_CONTEXT
    la    s6,  UART1_FAULT_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorUART1_FAULT_Handler
    .extern  UART1_RX_Handler

    .section   .vector_113,code, keep
    .equ     __vector_dispatch_113, IntVectorUART1_RX_Handler
    .global  __vector_dispatch_113
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorUART1_RX_Handler

IntVectorUART1_RX_Handler:
    portSAVE_CONTEXT
    la    s6,  UART1_RX_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorUART1_RX_Handler
    .extern  UART1_TX_Handler

    .section   .vector_114,code, keep
    .equ     __vector_dispatch_114, IntVectorUART1_TX_Handler
    .global  __vector_dispatch_114
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorUART1_TX_Handler

IntVectorUART1_TX_Handler:
    portSAVE_CONTEXT
    la    s6,  UART1_TX_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorUART1_TX_Handler
    .extern  USB_Handler

    .section   .vector_132,code, keep
    .equ     __vector_dispatch_132, IntVectorUSB_Handler
    .global  __vector_dispatch_132
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorUSB_Handler

IntVectorUSB_Handler:
    portSAVE_CONTEXT
    la    s6,  USB_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorUSB_Handler
    .extern  USB_DMA_Handler

    .section   .vector_133,code, keep
    .equ     __vector_dispatch_133, IntVectorUSB_DMA_Handler
    .global  __vector_dispatch_133
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorUSB_DMA_Handler

IntVectorUSB_DMA_Handler:
    portSAVE_CONTEXT
    la    s6,  USB_DMA_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorUSB_DMA_Handler
    .extern  DMA0_Handler

    .section   .vector_134,code, keep
    .equ     __vector_dispatch_134, IntVectorDMA0_Handler
    .global  __vector_dispatch_134
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorDMA0_Handler

IntVectorDMA0_Handler:
    portSAVE_CONTEXT
    la    s6,  DMA0_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorDMA0_Handler
    .extern  UART2_FAULT_Handler

    .section   .vector_145,code, keep
    .equ     __vector_dispatch_145, IntVectorUART2_FAULT_Handler
    .global  __vector_dispatch_145
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorUART2_FAULT_Handler

IntVectorUART2_FAULT_Handler:
    portSAVE_CONTEXT
    la    s6,  UART2_FAULT_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorUART2_FAULT_Handler
    .extern  UART2_RX_Handler

    .section   .vector_146,code, keep
    .equ     __vector_dispatch_146, IntVectorUART2_RX_Handler
    .global  __vector_dispatch_146
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorUART2_RX_Handler

IntVectorUART2_RX_Handler:
    portSAVE_CONTEXT
    la    s6,  UART2_RX_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorUART2_RX_Handler
    .extern  UART2_TX_Handler

    .section   .vector_147,code, keep
    .equ     __vector_dispatch_147, IntVectorUART2_TX_Handler
    .global  __vector_dispatch_147
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorUART2_TX_Handler

IntVectorUART2_TX_Handler:
    portSAVE_CONTEXT
    la    s6,  UART2_TX_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorUART2_TX_Handler

