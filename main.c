/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Class-B Safety CANFD Test for CAN
*              Block code example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Includes
********************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "SelfTest.h"
#include <stdio.h>

#define MAX_INDEX_VAL 0xFFF0u
#define CANFD_SUCCESS         (0UL)
#define CANFD_FAILURE         (1UL)


#if COMPONENT_CAT1A
    #define INTR_SRC_CANFD canfd_0_interrupts0_0_IRQn
#elif COMPONENT_CAT1C
    #define INTR_SRC_CANFD ((NvicMux3_IRQn << CY_SYSINT_INTRSRC_MUXIRQ_SHIFT) | canfd_0_interrupts0_0_IRQn)
#endif

uint32_t initCANFD(void);

/*******************************************************************************
* Global Variables
*******************************************************************************/
cy_stc_scb_uart_context_t CYBSP_UART_context;
cy_en_scb_uart_status_t initstatus;
uint32_t can_channel = 0;
cy_stc_canfd_context_t context;

/*******************************************************************************
* Function Name: main
****************************************************************************//**
*
* Initialises retarget-io,can-FD and invokes the self test for CanFD.
*
* \param None
*
* \return None
* 
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    /* Initialize the device and board peripherals */
    init_cycfg_all();
    Cy_SysLib_Delay(250);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init_fc(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
            CYBSP_DEBUG_UART_CTS,CYBSP_DEBUG_UART_RTS,CY_RETARGET_IO_BAUDRATE);

    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    initCANFD();

    uint16_t idx = 0u;

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("Class B CANFD test:\r\n");

    for (;;)
    {

        /*******************************/
        /* Run Program Counter Test... */
        /*******************************/
        if(OK_STATUS != SelfTest_CANFD(CANFD_HW, can_channel, &CANFD_config, &context, CY_CANFD_TEST_MODE_INTERNAL_LOOP_BACK))
        {
            printf("Error: CANFD\r\n");
            while (1)
            {
            }
        }
        
        else
        {
            printf("No Error: CANFD");
        }
        /* Print test counter */
        printf(", Test Counter = %x\r\n",idx);
        idx++;
        if (idx > MAX_INDEX_VAL)
        {
            idx = 0u;
        }
    }
}

/*******************************************************************************
* Function Name: CanfdInterruptHandler
****************************************************************************//**
*
* Invokes the CanfdInterruptHandler() PDL driver function.
*
* \param None
*
* \return None
* 
*******************************************************************************/
void CanfdInterruptHandler(void)
{
    /* Just call the IRQ handler with the current channel number and context */
    Cy_CANFD_IrqHandler(CANFD_HW, can_channel, &context);
}

/*******************************************************************************
* Function Name: initCANFD
********************************************************************************
*
* This function enables and initiates CANFD.
*
* \param None
*
* \return
* Status of initialization
*
*******************************************************************************/
uint32_t initCANFD(void)
{
    cy_en_canfd_status_t initStatus;
    cy_en_sysint_status_t sysStatus;

    uint32_t channelMask = 0x00000001UL; /* Enables channel 0 */
    uint16_t delay = 0x0006U; /* A delay in usec before the MRAM can be used */

    /* Setup the CANFD interrupt */
    /* Populate the configuration structure */

    cy_stc_sysint_t irq_cfg =
    {
        /* The upper bits of intrSrc (defined by CY_SYSINT_INTRSRC_MUXIRQ_SHIFT) are used to
           store system interrupt value and the remaining lower bits store the CPU IRQ value */
        .intrSrc = (INTR_SRC_CANFD),
        .intrPriority = 2UL,
    };

    Cy_CANFD_EnableMRAM(CANFD_HW, channelMask, delay);

    initStatus = Cy_CANFD_Init (CANFD_HW, can_channel, &CANFD_config, &context);
    if(CY_CANFD_SUCCESS != initStatus)
    {
        return CANFD_FAILURE;
    }

    /* Hook the interrupt service routine and enable the interrupt */
    sysStatus = Cy_SysInt_Init(&irq_cfg, &CanfdInterruptHandler);
    if(CY_SYSINT_SUCCESS != sysStatus)
    {
        return CANFD_FAILURE;
    }
    
    /* Enable interrupt in NVIC */
    #if COMPONENT_CAT1A
    NVIC_EnableIRQ((IRQn_Type) irq_cfg.intrSrc);
    #elif COMPONENT_CAT1C
        NVIC_EnableIRQ((IRQn_Type) NvicMux3_IRQn);
    #endif

    return CANFD_SUCCESS;

}

/* [] END OF FILE */

