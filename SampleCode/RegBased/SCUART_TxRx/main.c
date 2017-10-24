/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 4 $
 * $Date: 14/11/27 2:34p $
 * @brief    Show smartcard UART mode by connecting PA.2 and PA.3 pins.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC200Series.h"

uint8_t au8TxBuf[] = "Hello World!";
#define PLLCON_SETTING  CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK       50000000

/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART_Open(UART_T* uart, uint32_t u32baudrate);
uint32_t SCUART_Open(SC_T* sc, uint32_t u32baudrate);
void SCUART_Write(SC_T* sc, uint8_t *pu8TxBuf, uint32_t u32WriteBytes);

/*---------------------------------------------------------------------------------------------------------*/
/* The interrupt services routine of smartcard port 0                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void SC012_IRQHandler(void)
{
    // Print SCUART received data to UART port
    // Data length here is short, so we're not care about UART FIFO over flow.
    UART_WRITE(UART0, SCUART_READ(SC0));

    // RDA is the only interrupt enabled in this sample, this status bit
    // automatically cleared after Rx FIFO empty. So no need to clear interrupt
    // status here.

    return;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;

    /* Enable external XTAL 12MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for external XTAL clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLK_S_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;

    /* Enable SC0 module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_SC0_EN_Msk ;

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /* Select SC0 module clock source */
    CLK->CLKSEL3 &= CLK_CLKSEL3_SC0_S_Msk ;
    CLK->CLKSEL3 |= CLK_CLKSEL3_SC0_S_HXT ;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Set GPA multi-function pins for SC UART mode */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA2_Msk | SYS_GPA_MFP_PA3_Msk);
    SYS->GPA_MFP |= SYS_GPA_MFP_PA2_SC0_CLK | SYS_GPA_MFP_PA3_SC0_DAT;
    SYS->ALT_MFP1 &= ~(SYS_ALT_MFP1_PA2_Msk | SYS_ALT_MFP1_PA3_Msk);
    SYS->ALT_MFP1 |= SYS_ALT_MFP1_PA2_SC0_CLK | SYS_ALT_MFP1_PA3_SC0_DAT;

}

/*---------------------------------------------------------------------------------------------------------*/
/* This function use to enable UART function and set baud-rate.                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART_Open(UART_T* uart, uint32_t u32baudrate)
{
    uint8_t u8UartClkSrcSel;
    uint32_t u32ClkTbl[4] = {__HXT, 0, 0, __HIRC};
    uint32_t u32Baud_Div = 0;

    /* Get UART clock source selection */
    u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART_S_Msk) >> CLK_CLKSEL1_UART_S_Pos;

    /* Select UART function */
    uart->FUN_SEL = UART_FUNC_SEL_UART;

    /* Set UART line configuration */
    uart->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    /* Set UART Rx and RTS trigger level */
    uart->FCR &= ~(UART_FCR_RFITL_Msk | UART_FCR_RTS_TRI_LEV_Msk);

    /* Get PLL clock frequency if UART clock source selection is PLL */
    if(u8UartClkSrcSel == 1)
        u32ClkTbl[u8UartClkSrcSel] = CLK_GetPLLClockFreq();

    /* Set UART baud rate */
    if(u32baudrate != 0)
    {
        u32Baud_Div = UART_BAUD_MODE2_DIVIDER(u32ClkTbl[u8UartClkSrcSel], u32baudrate);

        if(u32Baud_Div > 0xFFFF)
            uart->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(u32ClkTbl[u8UartClkSrcSel], u32baudrate));
        else
            uart->BAUD = (UART_BAUD_MODE2 | u32Baud_Div);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* This function use to enable smartcard module UART mode and set baudrate.                                */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t SCUART_Open(SC_T* sc, uint32_t u32baudrate)
{
    uint32_t u32ClkSrc, u32Clk, u32Div;

    if(sc == SC0)
        u32ClkSrc = (CLK->CLKSEL3 & CLK_CLKSEL3_SC0_S_Msk) >> CLK_CLKSEL3_SC0_S_Pos;
    else if(sc == SC1)
        u32ClkSrc = (CLK->CLKSEL3 & CLK_CLKSEL3_SC1_S_Msk) >> CLK_CLKSEL3_SC1_S_Pos;
    else // SC2
        u32ClkSrc = (CLK->CLKSEL3 & CLK_CLKSEL3_SC2_S_Msk) >> CLK_CLKSEL3_SC2_S_Pos;

    // Get smartcard module clock
    if(u32ClkSrc == 0)
        u32Clk = __HXT;
    else if(u32ClkSrc == 1)
        u32Clk = CLK_GetPLLClockFreq();
    else
        u32Clk = __HIRC;


    if(sc == SC0)
        u32Clk /= (CLK->CLKDIV1 & CLK_CLKDIV1_SC0_N_Msk);
    else if(sc == SC1)
        u32Clk /= ((CLK->CLKDIV1 & CLK_CLKDIV1_SC1_N_Msk) >> CLK_CLKDIV1_SC1_N_Pos);
    else // SC2
        u32Clk /= ((CLK->CLKDIV1 & CLK_CLKDIV1_SC2_N_Msk) >> CLK_CLKDIV1_SC2_N_Pos);

    // Calculate divider for target baudrate
    u32Div = (u32Clk + (u32baudrate >> 1) - 1) / u32baudrate - 1;

    sc->CTL = SC_CTL_SC_CEN_Msk | SC_CTL_SLEN_Msk;  // Enable smartcard interface and stop bit = 1
    sc->UACTL = SCUART_CHAR_LEN_8 | SCUART_PARITY_NONE | SC_UACTL_UA_MODE_EN_Msk; // Enable UART mode, disable parity and 8 bit per character
    sc->ETUCR = u32Div;

    return(u32Clk / u32Div);
}

/*---------------------------------------------------------------------------------------------------------*/
/* This function is to write data into transmit FIFO to send data out.                                     */
/*---------------------------------------------------------------------------------------------------------*/
void SCUART_Write(SC_T* sc, uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    uint32_t u32Count;

    for(u32Count = 0; u32Count != u32WriteBytes; u32Count++)
    {
        while(SCUART_GET_TX_FULL(sc));  // Wait 'til FIFO not full
        sc->THR = pu8TxBuf[u32Count];    // Write 1 byte to FIFO
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("This sample code demos smartcard interface UART mode\n");
    printf("Please connect SC0 CLK pin(PA.2) with SC0 I/O pin(PA.3)\n");
    printf("Hit any key to continue\n");
    getchar();

    // Open smartcard interface 0 in UART mode.
    SCUART_Open(SC0, 115200);
    // Enable receive interrupt
    SCUART_ENABLE_INT(SC0, SC_IER_RDA_IE_Msk);
    NVIC_EnableIRQ(SC012_IRQn);

    SCUART_Write(SC0, au8TxBuf, sizeof(au8TxBuf));

    while(1);
}
