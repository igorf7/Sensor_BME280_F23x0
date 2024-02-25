#include "main.h"
#include "scheduler.h"

#define TIME_TO_SEND (SENDER_ID * 410)

const uint16_t TaInterval = 0xF000; // 15 sec. with ACLK/8
//const uint16_t TaInterval = 0x8000; // 1 sec. with ACLK/1

static RadioEvents_t RadioEvents = {
    NULL,
    NULL,
    NULL,
    NULL
};

Parameter_t Parameter;
//float altitude;

static struct {
    uint8_t intervals,
            minutes;
} Timer;

RF_Packet_t txPacket;

static uint8_t txPacketSize = 0;

void main(void)
{
    WDTCTL = WDTPW + WDTHOLD; //стоп WDT
    __disable_interrupt();
    
    // Setup Clock Module
    BCSCTL1 = CALBC1_8MHZ; // Set DCO to 8MHz
    DCOCTL = CALDCO_8MHZ;
    BIT_SET(BCSCTL3, XCAP_3 );

    /* Порт P1 */
    P1IE  = 0x00; // запрет всех прерываний порта P1
    P1SEL = 0x00; // ввод/вывод
    P1DIR = 0x00;
    P1REN = 0xFF;
    P1OUT = 0x00;
    P1IES = 0x00;
    P1IFG = 0x00;
    /* Порт P2 */
    P2IE  = 0x00; // запрет прерываний
    P2SEL = 0xC0; // P2.6, 7 - часовой кварц
    P2DIR = 0x00; //
    P2REN = 0x3F; /// 0x39; ///
    P2OUT = 0x00;
    P2IES = 0x00; // прерывания по фронту
    P2IFG = 0x00; // очистить регистр флагов
    /* Порт P3 */
    P3DIR = 0x00;
    P3REN = 0xC0;
    P3OUT = 0x00;
    /* Порт P4 */
    P4SEL = 0x00; // ввод/вывод
    P4REN = 0xEC;
    P4DIR = 0x00;
    P4OUT = 0x00;
    
    /* Radio callback-functions initialize */
	RadioEvents.TxDone = OnRfTxDone;   // data transfer is complete
	RadioEvents.TxError = OnRfTxError; // data transfer error
    InitRFChip(&RadioEvents);
    SetRFMode(RF_SLEEP);
    
    BME280_Init();
    BME280_SetMode(BME280_MODE_SLEEP);
    
    /* Initialize the task queue */
    initTaskQueue(&BackgroundTask);
    
    Timer.intervals = 0;
    Timer.minutes = 0;
    //Diagnostic.uptime = 0;
    
    InitTimerA();
    TAR = 0xD000;
    TACCR0_ENABLE(TaInterval);
    
    __enable_interrupt();
    
    /* Mainloop */
    while (1)
    {
        WDTCTL = WDT_ARST_1000; // Start WDT for 1000 ms
        runTaskSheduler();      // Run Task Sheduler
    }
}

/**
 * BackgroundTask
 */
void BackgroundTask(void)
{
    if (GetRfMode() != RF_SLEEP) SetRfSleep();
    WDTCTL = WDTPW + WDTHOLD; // Stop WDT
    _BIS_SR(LPM3_bits + GIE); // Enter in LPM_3 and wait for interrupts
    _NOP(); // Workaround (see MSP430F2xxx Device Erratasheet)
}

/**
 * Task handler TACCR0
 */
void TimerA0Task(void *prm)
{
    switch (Timer.intervals)
    {
    case 0: // Starts measurement 2 times per minute
    case 2:
        BME280_SetMode(BME280_MODE_FORCED);
        createRfPacket(BME280_PACKET, &txPacket);
        StartCCR2Timeout(TIME_TO_SEND);
        break;
//    case 1: // Send diagnostic data 1 time per hour
//        if (Timer.minutes == 0) {
//            createRfPacket(DIA_PACKET, &txPacket);
//            StartCCR2Timeout(TIME_TO_SEND);
//        }
//        break;
    default:
        break;
    }
//    Diagnostic.uptime += 15;
    if (++Timer.intervals > 3) {
        Timer.intervals = 0;
//        Timer.minutes++;
//        if (Timer.minutes > 59) {
//            Timer.minutes = 0;
//        }
    }
}

/**
 * Task Send RF packet
 */
void SendPacketTask(void *prm)
{
    switch (txPacket.header.pack_type)
    {
    case BME280_PACKET:
        Parameter.temperature = BME280_ReadTemperature();
        Parameter.pressure = BME280_ReadPressure();
        Parameter.humidity = BME280_ReadHumidity();
        txPacketSize = sizeof(Parameter);
        memcpy(txPacket.data, &Parameter, txPacketSize);        
        break;
//    case DIA_PACKET:
//        //Diagnostic.voltage_sl = measureBattary();
//        Diagnostic.voltage_sl = 3.3f;
//        Diagnostic.voltage_tx = 3.3f;
//        txPacketSize = sizeof(Diagnostic);
//        memcpy(txPacket.data, &Diagnostic, txPacketSize);
//        break;
    default:
        return;
    }
    txPacketSize += sizeof(txPacket.header);
    SendRfFrame((uint8_t*)&txPacket, txPacketSize); // Send packet
}

/**
 * Task Set Rf Sleep
 */
void SetRfSleepTask(void *prm)
{
    SetRFMode(RF_SLEEP);
}

/**
 * OnTxDone callback function
 */
void OnRfTxDone(void)
{
    putEvent(SetRfSleepTask, NULL);
}

/**
 * OnTxError callback function
 */
void OnRfTxError(void)
{
    putEvent(SetRfSleepTask, NULL);
}

/**
 * Timer A0 interrupt service routine
 */
#pragma vector = TIMERA0_VECTOR
__interrupt void Timer_A0 (void)
{
    BIT_CLEAR(TACCTL0, CCIFG);   // clear interrupt flag
    putEvent(TimerA0Task, NULL);
    _BIC_SR_IRQ(LPM3_bits + GIE);
}

/**
 * Timer A1 interrupt service routine
 */
#pragma vector = TIMERA1_VECTOR
__interrupt void Timer_A1(void)
{
    switch (__even_in_range(TAIV, 10))
    {
        case 2: // Not used
            break;
        case 4: // Send Packet Task
            StopCCR2Timeout();
            putEvent(SendPacketTask, NULL);
            break;
        case 10: // Not used
            break;
    }
    _BIC_SR_IRQ(LPM3_bits + GIE);
}

/* eof */
