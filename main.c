/******************************************************************************
* File Name:   main.c
*
* Description: This code example demonstrates how to use the tcpwm counter.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2024-2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
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
