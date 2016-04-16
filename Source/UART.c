/****************************************************************************
 Module
    UART.c

 Revision           Revised by:
    1.0.0           Vikram
    1.1.0           Eric
    1.1.1           Denny

 Description
    Initialize the Tiva UART modules

 Edits:
    1.0.0 - Modified for UART 2 for use in the PIC
    1.1.0 - Changed from a service into a module, added Xbee UART
            initialization.
    1.1.1 - General Clean-up.
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/

// Module headers
#include "UART.h"

/*----------------------------- Module Defines ----------------------------*/
#define BitsPerNibble           4

/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service.*/
void InitUART(void);
static void InitUART1(void);
static void InitUART2(void);
static void InitUART5(void);

/*---------------------------- Module Variables ---------------------------*/


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    InitUART

 Description
    Initialize the Tiva UART modules 1, 2, and 5
****************************************************************************/
void InitUART(void) {
    InitUART1();
    InitUART2();
    InitUART5();
}

/****************************************************************************
 Function
    InitUART1

 Description
    Initialize UART module 1 for Xbee
****************************************************************************/
static void InitUART1( void ) {
    /**************** Initialize UART module 1 *****************/
    /*
    
    MAKE SURE INTRRUPT HANDLER IS DECLARED IN THE STARTUP FILE
    
    Steps to initialize Tiva UART (Register)
    1.  Enable the clock to the UART module (RCGCUART)
    2.  Enable the clock to the appropriate GPIO module (RCGCGPIO)
    3.  Wait for the UART to be ready PRUART (Skipped since GPIO takes longer)
    4.  Wait for the GPIO module to be ready
    5.  Configure GPIO pins for in/out/drive-level/drivetype
    6.  Select the alternate function for the UART pins (AFSEL)
    7.  Configure the PMCn fields in the GPIOPCTL register to assign the UART pins
    8.  Disable the UART by clearing the UARTEN bit in the UARTCTL register
    9.  Write the integer portion of the BRD to the UARTIBRD register
    10. Write the fractional portion of the BRD to the UARTBRD register
    11. Write the desired serial parameters to the UARTLCRH register
    12. Configure the UART operation using the UARTCTL register
    13. Locally enable interrupts for the UART receive interruypt mask (RXIM in UARTIM)
    14. Enable UART1 interrupt vector in NVIC (Tiva p.104 for vector table)
    15. Enable the UART by setting the UARTEN bit in the (UARTCTL) register
    
    UART Module 1:
    U1RX - Receive (PC4, 2 Mux)
    U1TX - Transmit (PC5, 2 Mux)
    
    */
    
    // 1. Enable the clock to the UART module
    HWREG(SYSCTL_RCGCUART) |= SYSCTL_RCGCUART_R1;
    
    // 2. Enable clock to the GPIO port C
    HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R2;
    
    // 4. Wait for GPIO port C to be ready
    while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R2 ) != SYSCTL_PRGPIO_R2 )
        ;
    
    // 5. Configure GPIO pins for in/out/drive-level/drivetype
    // Set GPIO PC4 and PC5 as digital
    HWREG(GPIO_PORTC_BASE + GPIO_O_DEN) |= (BIT4HI | BIT5HI);
    
    // Set GPIO PC4 as input (receive) and PC5 as output (transfer)
    HWREG(GPIO_PORTC_BASE + GPIO_O_DIR) |= BIT5HI;
    HWREG(GPIO_PORTC_BASE + GPIO_O_DIR) &= ~BIT4HI;
    
    // Configure the transfer line (PC5) as open drain, see TIVA p.676
    //HWREG(GPIO_PORTC_BASE + GPIO_O_ODR) |= BIT5HI;
    
    // 6. Program pins C4, C5 as alternate functions on the GPIO to use UART (Set High) 
    HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= (BIT4HI | BIT5HI);
    
    // 7. Set mux position on GPIOPCTL to select UART alternate function on C4, C5 (Tiva p.1351)
    // Mux value = 2 (p.895) offset mask to clear nibbles 4 and 5 on the port controll
    HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) =                                                                                                                                          
    (HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) & ~0x00ff0000) + (2 << ( 4 * BitsPerNibble)) +
      (2 << ( 5 * BitsPerNibble));
    
    // 8. Disable UART module 1
    HWREG(UART1_BASE + UART_O_CTL) &= ~UART_CTL_UARTEN;
    
    // Baud Rate = 9600, Integer = 260, Fraction = 27
    // 9. Write the integer portion of the BRD (260)
    HWREG(UART1_BASE + UART_O_IBRD) = 0x0104;
    
    // 10. Write the fractional portion of the BRD (27)
    HWREG(UART1_BASE + UART_O_FBRD) = 0x1B;
    
    // 11. Write desired serial parameters to the UART line control
    // Set 8-bit frame length and clear everything else 
    HWREG(UART1_BASE + UART_O_LCRH) = UART_LCRH_WLEN_8; 

    // 12. Configure UART operation (Enable Receive and Transmit)
    HWREG(UART1_BASE + UART_O_CTL) |= (UART_CTL_RXE | UART_CTL_TXE | UART_CTL_EOT);
    
    // 13. Locally enable interupts for UART receive and transmit interrupt mask (RXIM/TXIM)
    HWREG(UART1_BASE + UART_O_IM) |= (UART_IM_RXIM | UART_IM_TXIM);

    // 14. Set NVIC enable for UART1 (see Tiva p.104)
    HWREG(NVIC_EN0) |= BIT6HI;
    
    // Make sure interrupts are enabled globally
    __enable_irq( );

    // 15. Enable UART module 1
    HWREG(UART1_BASE + UART_O_CTL) |= UART_CTL_UARTEN;

    // Print to console if successful initialization
    printf("UART 1 Initialized\n\r");
}

/****************************************************************************
 Function
    InitUART2

 Description
    Initialize UART module 2 for Wind
****************************************************************************/
static void InitUART2(void) {
    /**************** Initialize UART module 2 *****************/
    /*
    
    UART Module 2:
    U2RX - Receive (PD6, 1 Mux)
    U2TX - Transmit (PD7, 1 Mux)
    
    */
    
    // Enable the clock to the UART module
    HWREG(SYSCTL_RCGCUART) |= SYSCTL_RCGCUART_R2;
    
    // Enable clock to the GPIO port D
    HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R3;
    
    // Wait for GPIO port D to be ready
    while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R3 ) != SYSCTL_PRGPIO_R3 )
        ;
    
    // Configure GPIO pins for in/out/drive-level/drivetype
    // Set GPIO PD6/7 as digital
    HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= (BIT6HI | BIT7HI);
    
    // Set GPIO PD6 as input (receive) and PD7 as output (transfer)
    HWREG(GPIO_PORTD_BASE + GPIO_O_DIR) |= BIT7HI;
    HWREG(GPIO_PORTD_BASE + GPIO_O_DIR) &= ~BIT6HI;

    // Program pins D6, D7 as alternate functions on the GPIO to use UART (Set High) 
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) |= (BIT6HI | BIT7HI);
    
    // Set mux position on GPIOPCTL to select UART alternate function on E4, E5 (Tiva p.1351)
    // Mux value = 2 (p.895) offset mask to clear nibbles 7 and 7
    HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) =                                                                                                                                          
    (HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) & ~0xFF000000) + (1 << ( 6 * BitsPerNibble)) +
      (1 << ( 7 * BitsPerNibble));

    // Disable UART module 2
    HWREG(UART2_BASE + UART_O_CTL) &= ~UART_CTL_UARTEN;

    // Baud Rate = 4800, Integer = 520, Fraction = 54
    // Write the integer portion of the BRD (260)
    HWREG(UART2_BASE + UART_O_IBRD) = 0x0208;

    // Write the fractional portion of the BRD (54)
    HWREG(UART2_BASE + UART_O_FBRD) = 0x36;

    // Write desired serial parameters to the UART line control
    // Set 8-bit frame length and clear everything else
    HWREG(UART2_BASE + UART_O_LCRH) = UART_LCRH_WLEN_8; 

    // Configure UART operation (Enable Receive and Transmit)
    HWREG(UART2_BASE + UART_O_CTL) |= (UART_CTL_RXE | UART_CTL_TXE|UART_CTL_EOT);

    // Locally enable interupts for UART receive and transmit interrupt mask (RXIM and TXIM)
    HWREG(UART2_BASE + UART_O_IM) |= (UART_IM_RXIM|UART_IM_TXIM);

    // Set NVIC enable for UART2 (see Tiva p.104) 33, EN1 bit 1 (use go to definition)
    HWREG(NVIC_EN1) |= BIT1HI;

    // Make sure interrupts are enabled globally
    __enable_irq( );

    // Enable UART module 2
    HWREG(UART2_BASE + UART_O_CTL) |= UART_CTL_UARTEN;

    // Print to console if successful initialization
    printf("UART 2 Initialized\n\r");
}

/****************************************************************************
 Function
    InitUART5

 Description
    Initialize UART module 5 for GPS
****************************************************************************/
static void InitUART5(void) {
    /**************** Initialize UART module 5 *****************/
    /*
    
    UART Module 5:
    U2RX - Receive (PE4, 1 Mux)
    U2TX - Transmit (PE5, 1 mux)
    
    */
    
    // Enable the clock to the UART module
    HWREG(SYSCTL_RCGCUART) |= SYSCTL_RCGCUART_R5;
    // Enable the clock to the GPIO module
    HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R4;
    
    // Wait for GPIO port E to be ready
    while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R4 ) != SYSCTL_PRGPIO_R4 )
        ;
    
    // Configure GPIO pins for in/out/drive-level/drivetype
    // Set GPIO PE4/5 as digital
    HWREG(GPIO_PORTE_BASE + GPIO_O_DEN) |= (BIT4HI | BIT5HI);
    
    // Set GPIO PE4 as input (receive) and PE5 as output (transfer)
    HWREG(GPIO_PORTE_BASE + GPIO_O_DIR) |= BIT5HI;
    HWREG(GPIO_PORTE_BASE + GPIO_O_DIR) &= ~BIT4HI;

    // Program pins E4, E5 as alternate functions on the GPIO to use UART (Set High) 
    HWREG(GPIO_PORTE_BASE + GPIO_O_AFSEL) |= (BIT4HI | BIT5HI);
    
    // Set mux position on GPIOPCTL to select UART alternate function on E4, E5 (Tiva p.1351)
    // Mux value = 2 (p.895) offset mask to clear nibbles 4 and 5
    HWREG(GPIO_PORTE_BASE + GPIO_O_PCTL) =                                                                                                                                          
    (HWREG(GPIO_PORTE_BASE + GPIO_O_PCTL) & ~0x00FF0000) + (1 << ( 4 * BitsPerNibble)) +
      (1 << ( 5 * BitsPerNibble));

    // Disable UART module 5
    HWREG(UART5_BASE + UART_O_CTL) &= ~UART_CTL_UARTEN;

    // Baud Rate = 4800, Integer = 520, Fraction = 54
    // Write the integer portion of the BRD (260)
    HWREG(UART5_BASE + UART_O_IBRD) = 0x0104;

    // Write the fractional portion of the BRD (27)
    HWREG(UART5_BASE + UART_O_FBRD) = 0x1B;

    // Write desired serial parameters to the UART line control
    // Set 8-bit frame length and clear everything else 
    HWREG(UART5_BASE + UART_O_LCRH) = UART_LCRH_WLEN_8; 

    // Configure UART operation (Enable Receive and Transmit)
    HWREG(UART5_BASE + UART_O_CTL) |= (UART_CTL_RXE | UART_CTL_TXE | UART_CTL_EOT);

    // Locally enable interupts for UART receive interrupt mask (RXIM)
    HWREG(UART5_BASE + UART_O_IM) |= (UART_IM_RXIM|UART_IM_TXIM);

    // Set NVIC enable for UART5 (see Tiva p.104) 33, EN1 bit 29 (use go to definitionn)
    HWREG(NVIC_EN1) |= BIT29HI;

    // Make sure interrupts are enabled globally
    __enable_irq( );

    // Enable UART module 5
    HWREG(UART5_BASE + UART_O_CTL) |= UART_CTL_UARTEN;

    // Print to console if successful initialization
    printf("UART 5 Initialized\n\r");
}
