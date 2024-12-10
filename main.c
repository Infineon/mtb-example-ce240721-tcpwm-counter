/******************************************************************************
* File Name:   main.c
*
* Description: This code example demonstrates how to use the tcpwm counter.
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
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define CC0_INTERRUPT_PRIORITY (7u)

/*******************************************************************************
* Global Variables
********************************************************************************/
const cy_stc_sysint_t intrCfg =
{
        .intrSrc = TCPWM_COUNTER_IRQ,
        .intrPriority = CC0_INTERRUPT_PRIORITY
};

/*******************************************************************************
* Function Name: cc0_Interrupt_handler_pdl
********************************************************************************
*
*  Summary:
*  Counter cc0 interrupt handler for the PDL example.
*
*  Parameters:
*  None
*
*  Return:
*  None
*
**********************************************************************************/
void cc0_interrupt_handler_pdl(void)
{
    uint32_t interrupts = Cy_TCPWM_GetInterruptStatusMasked(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM);

    /* Clear the interrupt */
    Cy_TCPWM_ClearInterrupt(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM, interrupts);

    if (0UL != (CY_TCPWM_INT_ON_TC & interrupts))
    {
        /* Handle the Terminal Count event */
        Cy_GPIO_Inv(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN); /* toggle LED */
    }

    if (0UL != (CY_TCPWM_INT_ON_CC & interrupts))
    {
        /* Handle the Compare/Capture event */
        Cy_GPIO_Inv(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN); /* toggle LED */
    }

}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This counter will be triggered by software, and counter period is
* 60000 with 200Khz clock frequency. Enable the CC0 match interrupt, CC0 match value
* is 50000. It will generate the interrupt when counter up to CC0 match value, then
* toggle user LED with interrupt generated each 0.25s.
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable interrupts */
    __enable_irq();

    /*TCPWM Counter Mode initial*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM, &TCPWM_COUNTER_config))
    {
        CY_ASSERT(0);
    }

    /* Enable the initialized counter */
    Cy_TCPWM_Counter_Enable(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM);

    /* Configure and enable Counter CC0 interrupt */
    Cy_SysInt_Init(&intrCfg, cc0_interrupt_handler_pdl);
    NVIC_EnableIRQ(TCPWM_COUNTER_IRQ);

    /* Start the counter */
    Cy_TCPWM_TriggerStart_Single(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM);


    for (;;)
    {
      /*empty loop*/
    }

}
