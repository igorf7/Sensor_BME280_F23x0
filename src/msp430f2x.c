#include  "msp430f2x.h"

extern const uint16_t TaInterval;

/**
 * USCIA0_ToSpiMode
*/
void USCIA0_ToSpiMode(uint8_t Mode, uint8_t ClkDiv)
{    
    P3SEL |= 0x31;  // USCI_A0 option select
    BIT_SET(UCA0CTL1, UCSWRST); //disable USCI Module
    BIT_SET(UCA0CTL0, Mode + UCCKPH + UCMSB + UCSYNC);
    BIT_SET(UCA0CTL1, UCSSEL_2);    // SMCLK
    UCA0BR0 = ClkDiv;
    UCA0BR1 = 0;
    UCA0MCTL = 0;
    BIT_CLEAR(IFG2, UCA0RXIFG);
    BIT_CLEAR(IFG2, UCA0TXIFG);
    BIT_CLEAR(UCA0CTL1, UCSWRST);   //enable USCI Module
}

/**
 * USCIB0_ToSpiMode(init USCIB0 in SPI mode)
 */
void USCIB0_ToSpiMode(uint8_t Mode, uint8_t ClkDiv)
{        
    P3SEL |= 0x0E;  // USCI_B0 option select
    BIT_SET(UCB0CTL1, UCSWRST); //disable USCI Module
    BIT_SET(UCB0CTL0, Mode + UCCKPH + UCMSB + UCSYNC);
    BIT_SET(UCB0CTL1, UCSSEL_2);    // SMCLK
    UCB0BR0 = ClkDiv;
    UCB0BR1 = 0;
    BIT_CLEAR(IFG2, UCB0RXIFG);  
    BIT_CLEAR(IFG2, UCB0TXIFG);
    BIT_CLEAR(UCB0CTL1, UCSWRST); //enable USCI Module
}

/**
 * Reed Write_SPI_A0
*/
uint8_t SPI_A0_ReedWrite(uint8_t SndByte)
{
	UCA0TXBUF = SndByte;
	while(UCA0STAT & UCBUSY)
    {
	}
	return UCA0RXBUF;
}

/**
 * Reed Write_SPI_B0
 */
uint8_t SPI_B0_ReedWrite(uint8_t SndByte)
{    
    UCB0TXBUF = SndByte;
    while (UCB0STAT & UCBUSY)
    {
    }
    return UCB0RXBUF;
}

/**
 * Sending one byte via UART
 */
void SendViaUart(uint8_t value)
{    
    while(!(UC0IFG & UCA0TXIFG));
    UCA0TXBUF = value;
}

/**
 * UART byte array forwarding
 */
void SendArrayViaUart(uint8_t *arr, uint16_t len)
{
    while (len--) {
        SendViaUart(*arr++);
    }
}

/**
 * USCIA0_ToUartMode(init USCIA0 in UART mode)
 */
void USCIA0_ToUartMode(void)
{    
    BIT_SET(UCA0CTL1, UCSSEL_2);    // CLK = SMCLK
    UCA0BR0 = 68;                   // 8MHz/115.200
    UCA0BR1 = 0x00;                 //
    UCA0MCTL = UCBRS1 + UCBRS0;     // Modulation UCBRSx = 3
    BIT_SET(P3SEL, (BIT4 + BIT5));  //
    BIT_CLEAR(IFG2, UCA0RXIFG);     //
    BIT_CLEAR(UCA0CTL1, UCSWRST);   // Initialize USCI state machine
    BIT_SET(IE2, UCA0RXIE);         // Enable USCI_A0 RX interrupt
}

/**
 * Wait
 */
void Wait_us(uint16_t value)
{        
    if (value > 8000) {
        value = 8000;
    }
    
    value <<= 3;	// multiply by 8 if DCO == 8MHz
    
    TBCTL &= ~MC_1;
    TBCTL &= ~TBIE;
    TBCCTL0 &= ~CCIE;
    TBCCTL0 &= ~CCIFG;
    TBCTL = TBCLR + TBSSEL_2;
    TBCCR0 = value;
    TBCTL |= MC_1;
    while(!(TBCCTL0 & CCIFG)){
        ;
    }
    TBCTL &= ~(MC_1 + TBSSEL_2);
    TBCCR0 = 0;
}

/**
 * InitTimerA
 */
void InitTimerA(void)
{    
    TACCR0_DISABLE;
    TACCR1_DISABLE;
    TACCR2_DISABLE;
    TACTL &= ~MC_1;
    TAR = 0;
    //TACTL = MC_1 + TASSEL_1 + ID_0; // режим UP, ACLK, делитель на 1
    TACTL = MC_1 + TASSEL_1 + ID_3; // режим UP, ACLK, делитель на 8
    BIT_CLEAR(TACCTL0, CCIFG);
    BIT_CLEAR(TACCTL1, CCIFG);
    BIT_CLEAR(TACCTL2, CCIFG);
}

/**
 * StartCCR1Timeout
 */
void StartCCR1Timeout(uint16_t offset)
{
    offset += TAR;
    if (offset <= TaInterval) {
        TACCR1_ENABLE(offset);
    }
    else {
        TACCR1_ENABLE(offset-TaInterval);
    }
}

/**
 * StopCCR1Timeout
 */
void StopCCR1Timeout(void)
{
    TACCR1_DISABLE;
}

/**
 * StartCCR2Timeout
 */
void StartCCR2Timeout(uint16_t offset)
{
    offset += TAR;
    if (offset <= TaInterval) {
        TACCR2_ENABLE(offset);
    }
    else {
        TACCR2_ENABLE(offset-TaInterval);
    }
}

/**
 * StopCCR2Timeout
 */
void StopCCR2Timeout(void)
{
    TACCR2_DISABLE;
}
// eof
