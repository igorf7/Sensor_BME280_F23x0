/************************************************************************************************
*                                       globalls.h			                                    *
*				©2011 by closed (joint-stock) company "SENS". All rights reserved.  			*
*									Written by I.Filippov										*
************************************************************************************************/
#ifndef __globalls
#define __globalls

#include "mytypes.h"
#include "msp430f2x.h"
#include "packets.h"
#include "sx1211driver.h"

/* ѕорты */
//#define LED_PORT	P4OUT	//порт светодиода
//#define	BLUE		BIT4	//
#if defined(__msp430x23x0)
	#define LED_PORT	P4OUT	//порт светодиода
	#define	LED_PIN		BIT5	//пин светодиода
#elif defined(__msp430x22x2)
	#define LED_PORT	P1OUT	//порт светодиода
	#define	LED_PIN     BIT1	//пин светодиода
#endif

/* ћакросы */
#define	LED_ON(Led)			(LED_PORT &= ~Led)	//зажечь светодиод
#define	LED_OFF(Led)  		(LED_PORT |=  Led)	//погасить светодиод
#define	TOGGLE_LED(Led)     (LED_PORT ^=  Led)	//xor светодиод

#define	SWAPB(Word)				    ((unsigned short)((Word) << 8) | ((Word) >> 8))
#define	TACCR0_ENABLE(TaInterval)	{TACCTL0 &= ~CCIFG; TACCR0 = TaInterval; TACCTL0 |= CCIE;}
#define	TACCR0_DISABLE			    {TACCTL0 &= ~CCIE; TACCR0 = 0;}
#define	TACCR1_ENABLE(TaInterval)	{TACCTL1 &= ~CCIFG; TACCR1 = TaInterval; TACCTL1 |= CCIE;}
#define	TACCR1_DISABLE			    {TACCTL1 &= ~CCIE;}
#define	TACCR2_ENABLE(TaInterval)	{TACCTL2 &= ~CCIFG; TACCR2 = TaInterval; TACCTL2 |= CCIE;}
#define	TACCR2_DISABLE			    {TACCTL2 &= ~CCIE;}

#define	BIT_SET(Reg, Bit)		Reg |= (Bit)
#define	BIT_CLEAR(Reg, Bit)		Reg &= ~(Bit)
#define	BIT_CHECK(Reg, Bit)		(Reg & Bit) ? 1 : 0
#define	TOGGLE_BIT(Reg, Bit)	Reg ^= (Bit)

#endif	/* globalls */
