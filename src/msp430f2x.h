#ifndef __msp430f2x
#define __msp430f2x

#include <msp430x23x0.h>

#include "globalls.h"

#define	OSC_TIMEOUT     190         		// start delay LFXT1 > 50 µs
#define	CONFSIZE        15          		// configuration package size
#define	PASPSIZE        8           		// device passport size
#define	HEADSIZE        1           		// configuration packet header size
#define	SEG_SIZE      	64					// segment size
#define	MCU_RESET     	(WDTCTL = 0x01) 	// system reset
#define	MASTER			UCMST				// Master Mode SPI
#define	SLAVE			0					// Slave Mode SPI
#define	MEANDER			(0xAA)

#define NSS_PORT		P4OUT				//
#define NSS_CONF		BIT0				// Px.0
#define NSS_DATA		BIT1				// Px.1
#define IRQ_IFG			P2IFG				//
#define IRQ_IE			P2IE				//
#define IRQ_0_PIN       BIT2				// Px.2	(IRQ_0)
#define IRQ_1_PIN       BIT1				// Px.1	(IRQ_1)
#define PLL_L_PIN       BIT0				// Px.0	(IRQ_PLL)

/*!
*/
void InitMCU(void);

/*!
*/
void InitPort1(void);

/*!
*/
void InitPort2(void);

/*!
*/
void InitPort3(void);

/*!
*/
void InitPort4(void);

/*!
*/
void SetupPorts(void);

/*!
*/
void InitTimerA(void);

/*!
*/
void InitTimerB(void);

/*!
*/
//void Delay(uInt16 ms);

/*!
*/
void USCIA0_ToSpiMode(uint8_t Mode, uint8_t ClkDiv);

/*!
*/
void USCIB0_ToSpiMode(uint8_t Mode, uint8_t ClkDiv);

/*!
*/
void USCIA0_ToUartMode(void);

/*!
*/
uint8_t SPI_A0_ReedWrite(uint8_t SndByte);

/*!
*/
uint8_t SPI_B0_ReedWrite(uint8_t SndByte);

/*!
*/
void SendMassBySpi(uint8_t UsartNumber, uint8_t* mass, uint16_t nbyte);

/*!
*/
void Delay(uint16_t cntVal);

/*!
*/
void Wait_us(uint16_t value);

void StartCCR1Timeout(uint16_t offset);
void StartCCR2Timeout(uint16_t offset);
void StopCCR1Timeout(void);
void StopCCR2Timeout(void);
void SendViaUart(uint8_t value);
void SendArrayViaUart(uint8_t *arr, uint16_t len);
#endif /* msp430f2x.h */

