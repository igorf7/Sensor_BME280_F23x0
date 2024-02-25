/************************************************************************
*   File sx1211driver.c - sx1211 transceiver driver for the MSP430 MCU  *
*                           Packet Mode                                 *
************************************************************************/
#include "sx1211driver.h"
#include "string.h"

volatile uint8_t ModeNow = RF_STANDBY; //Previous chip operating mode

static RadioEvents_t *RadioEvents;

static uint16_t byteCounter = 0, // RF frame byte counter
                whileTimeout = 0;

static uint8_t rfRxBuffer[RF_BUFFER_SIZE];

uint16_t RegistersCfg[] = { 
    // SX1211 configuration registers values
    DEF_MCPARAM1 | RF_MC1_STANDBY | RF_MC1_BAND_868 | RF_MC1_VCO_TRIM_00 | RF_MC1_RPS_SELECT_1,
    DEF_MCPARAM2 | RF_MC2_MODULATION_FSK | RF_MC2_DATA_MODE_PACKET | RF_MC2_OOK_THRESH_TYPE_PEAK | RF_MC2_GAIN_IF_01,
    DEF_FDEV | RF_FDEV_57,///RF_FDEV_100,
    DEF_BITRATE | RF_BITRATE_50000,
    DEF_OOKFLOORTHRESH | RF_OOKFLOORTHRESH_VALUE,
    DEF_MCPARAM6 | RF_MC6_FIFO_SIZE_64 | RF_MC6_FIFO_THRESH_VALUE,
    DEF_R1 | RF_R1_VALUE,
    DEF_P1 | RF_P1_VALUE,
    DEF_S1 | RF_S1_VALUE,
    DEF_R2 | RF_R2_VALUE,
    DEF_P2 | RF_P2_VALUE,
    DEF_S2 | RF_S2_VALUE,
    DEF_PARAMP | RF_PARAMP_11,
		
    DEF_IRQPARAM1 | RF_IRQ0_RX_STDBY_PAYLOADREADY | RF_IRQ1_RX_STDBY_CRCOK | RF_IRQ1_TX_TXDONE,
    DEF_IRQPARAM2 | RF_IRQ0_TX_FIFOEMPTY_START_FIFONOTEMPTY | RF_IRQ2_PLL_LOCK_PIN_ON,
    DEF_RSSIIRQTHRESH | RF_RSSIIRQTHRESH_VALUE,
		
    DEF_RXPARAM1 | RF_RX1_PASSIVEFILT_321 | RF_RX1_FC_FOPLUS100,
    DEF_RXPARAM2 | RF_RX2_FO_100, 
    DEF_RXPARAM3 | RF_RX3_POLYPFILT_OFF | RF_RX3_SYNC_SIZE_32 | RF_RX3_SYNC_TOL_0,
    DEF_RES19,
    //RSSI Value (Read only)
    DEF_RXPARAM6 | RF_RX6_OOK_THRESH_DECSTEP_000 | RF_RX6_OOK_THRESH_DECPERIOD_000 | RF_RX6_OOK_THRESH_AVERAGING_00,
		
    DEF_SYNCBYTE1 | 0xC1, // 1st byte of Sync word,
    DEF_SYNCBYTE2 | 0x94, // 2nd byte of Sync word,
    DEF_SYNCBYTE3 | 0xC1, // 3rd byte of Sync word,
    DEF_SYNCBYTE4 | 0x94, // 4th byte of Sync word,
		
    DEF_TXPARAM | RF_TX_FC_250/*RF_TX_FC_200*/ | RF_TX_POWER_PLUS4,
		
    DEF_OSCPARAM | RF_OSC_CLKOUT_OFF | RF_OSC_CLKOUT_427,

    DEF_PKTPARAM1 | RF_PKT1_MANCHESTER_OFF | 64,                  
    DEF_NODEADRS  | RF_NODEADRS_VALUE,                 
    DEF_PKTPARAM3 | RF_PKT3_FORMAT_VARIABLE | RF_PKT3_PREAMBLE_SIZE_24 | RF_PKT3_WHITENING_ON | RF_PKT3_CRC_ON | RF_PKT3_ADRSFILT_00,                    
    DEF_PKTPARAM4 | RF_PKT4_AUTOCLEAR_ON | RF_PKT4_FIFO_STANDBY_WRITE
};

static inline void select_config_line()
{
    P4DIR |= (NSS_CONF | NSS_DATA);
    BIT_SET(NSS_PORT, NSS_DATA);
    BIT_CLEAR(NSS_PORT, NSS_CONF);
}

static inline void deselect_config_line()
{
    BIT_SET(NSS_PORT, NSS_CONF);
    P4DIR &= ~(NSS_CONF | NSS_DATA);
}

static inline void select_data_line()
{
    P4DIR |= (NSS_CONF | NSS_DATA);
    BIT_SET(NSS_PORT, NSS_CONF);
    BIT_CLEAR(NSS_PORT, NSS_DATA);
}

static inline void deselect_data_line()
{
    BIT_SET(NSS_PORT, NSS_DATA);
    P4DIR &= ~(NSS_CONF | NSS_DATA);
}

/**
*/
static void WriteRegister(uint8_t address, uint8_t value)
{
    select_config_line();
    address = ((address << 1) & 0x3E);
    SPI_B0_ReedWrite(address);
    SPI_B0_ReedWrite(value);
    deselect_config_line();
}

/**
*/
static uint8_t ReadRegister(uint8_t address)
{
    volatile uint8_t rcv_byte = 0;
    
    select_config_line();
    address = ((address << 1) & 0x7E) | 0x40;
    rcv_byte = SPI_B0_ReedWrite(address);
    rcv_byte = SPI_B0_ReedWrite(0);
    deselect_config_line();
    
    return rcv_byte;
}

/**
 */
static void SendByte(uint8_t data)
{
    select_data_line();
	SPI_B0_ReedWrite(data);
    deselect_data_line();
}

/**
*/
static uint8_t ReceiveByte(void)
{    
    uint8_t rcv_byte = 0;

    select_data_line();
	rcv_byte = SPI_B0_ReedWrite(0x55);
    deselect_data_line();

    return rcv_byte;
}

/**
 * Initialization SX1211 Registers
*/
void InitRFChip(RadioEvents_t *events)
{
    uint16_t i;

    RadioEvents = events;

    USCIB0_ToSpiMode(MASTER, 8);  // SPI B0 for SX1211
    
    for (i = 0; (i + 1) <= REG_PKTPARAM4; i++) {
        if (i < REG_RSSIVALUE) {
            WriteRegister(i, RegistersCfg[i]);
        }
        else {
            WriteRegister(i + 1, RegistersCfg[i]);
        }
    }
}

/**
*/
void SetRFMode(uint8_t mode)
{	
    BIT_CLEAR(IRQ_IE, IRQ_1_PIN); // Desable interrupt IRQ_1
    BIT_CLEAR(IRQ_IFG, IRQ_1_PIN);	// Clear flag IRQ_1
//    if (mode != PreMode) {
        if (mode == RF_TRANSMITTER) {
            if (ModeNow == RF_SLEEP) {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_STANDBY);        		
                Wait_us(TS_OS);        		
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
                Wait_us(TS_FS);         		
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_TRANSMITTER);
                Wait_us(TS_TR);
            }
            else if (ModeNow == RF_STANDBY) {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
                Wait_us(TS_FS);         		
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_TRANSMITTER);
                Wait_us(TS_TR);
            }
            else if (ModeNow == RF_SYNTHESIZER) {
        		   WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_TRANSMITTER);
        		   Wait_us(TS_TR);
        		}
        		else if (ModeNow == RF_RECEIVER) {
                    WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_TRANSMITTER);
                    Wait_us(TS_TR);
        		}
                ModeNow = RF_TRANSMITTER;
        }
        else if (mode == RF_RECEIVER) {
            if (ModeNow == RF_SLEEP) {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_STANDBY);        		
                Wait_us(TS_OS);        		
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
                Wait_us(TS_FS); 
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_RECEIVER);
                Wait_us(TS_RE);
            }
            else if (ModeNow == RF_STANDBY) {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
                Wait_us(TS_FS); 
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_RECEIVER);
                Wait_us(TS_RE);
            }
            else if (ModeNow == RF_SYNTHESIZER) {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_RECEIVER);
        		   Wait_us(TS_RE);     		
        		}

        		else if (ModeNow == RF_TRANSMITTER) {	
        		   WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_RECEIVER);
        		   Wait_us(TS_RE);
        		}
                BIT_CLEAR(IRQ_IFG, IRQ_1_PIN);	// Clear flag IRQ_1
                BIT_SET(IRQ_IE, IRQ_1_PIN); // Enable interrupt IRQ_1 (CRCOK)
                ModeNow = RF_RECEIVER;
        }
        else if (mode == RF_SYNTHESIZER) {
            if (ModeNow == RF_SLEEP) {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_STANDBY);        		
                Wait_us(TS_OS);        		
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
                Wait_us(TS_FS); 
            }
            else if (ModeNow == RF_STANDBY) {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
                Wait_us(TS_FS); 
            }
            else {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
            }
            ModeNow = RF_SYNTHESIZER;
        }
        else if (mode == RF_STANDBY) {
            if (ModeNow == RF_SLEEP) {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_STANDBY);        		
                Wait_us(TS_OS);
            }
            else {
                WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_STANDBY);       		
            }
            ModeNow = RF_STANDBY;
        }
        else if (mode == RF_SLEEP) {
            WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SLEEP);
            ModeNow = RF_SLEEP;
        }
//    }
}

/**

static uint16_t ReadRssi(void)
{	
	return ReadRegister(REG_RSSIVALUE);
}*/

/**
 * Sends data packet
*/
void SendRfFrame(uint8_t *buffer, uint16_t size)
{
    if ((size+1) > (((RegistersCfg[REG_MCPARAM6])>>6)+1)*16) {  // If (size + length byte) > FIFO size
		if (RadioEvents->TxError != NULL) {
			RadioEvents->TxError(); // notify application about TxError event
		}
        return;
    }
    
    SetRFMode(RF_STANDBY);
	
    WriteRegister(REG_PKTPARAM4, (RegistersCfg[REG_PKTPARAM4-1] & 0xBF) | RF_PKT4_FIFO_STANDBY_WRITE);
    
    SendByte(size);

	for (byteCounter = 0; byteCounter < size;) {
        SendByte(buffer[byteCounter++]);
    }
    SetRFMode(RF_TRANSMITTER); // RF_IRQ0_TX_FIFOEMPTY_START_FIFONOTEMPTY
    
    while (!(BIT_CHECK(IRQ_IFG, IRQ_1_PIN))) // Wait for TX done
    {
        if (++whileTimeout > 3000) {
            break;
        }
    }
    
    BIT_CLEAR(IRQ_IFG, IRQ_1_PIN);
    whileTimeout = 0;
    Wait_us(500); // Wait for last bit to be sent (worst case bitrate)
    
    SetRFMode(RF_STANDBY);
    
	if (RadioEvents->TxDone != NULL) {
		RadioEvents->TxDone(); // Notify application about TxDone event
	}
}

/**
*/
void ReadDataPacket(void *prm)
{
	int16_t rssi;	// RSSI value
    uint8_t size;

	if (ModeNow == RF_RECEIVER) {
		rssi = ReadRegister(REG_RSSIVALUE); // RSSI
		SetRFMode(RF_STANDBY);
		WriteRegister(REG_PKTPARAM4, (RegistersCfg[REG_PKTPARAM4-1] & 0xBF) | RF_PKT4_FIFO_STANDBY_READ);
		
		size = ReceiveByte(); //
		
		if ((!size) || (size > RF_BUFFER_SIZE)) {
		    if (RadioEvents->RxError != NULL) {
				RadioEvents->RxError(); // Notify application about RxError event
			}
			return;
		}
		for(byteCounter = 0; byteCounter < size;) {
			rfRxBuffer[byteCounter++] = ReceiveByte();
		}
		if (RadioEvents->RxDone != NULL ) {
			rssi = (int16_t)((rssi - 40) * 0.6f - 100.0f); // Compute RSSI
			RadioEvents->RxDone(rfRxBuffer, size, rssi); // Notify application about RxDone event
		}
	}
}

/**
 */
uint8_t GetRfMode(void)
{
	return ModeNow;
}

/**
 * In  : timeout_ms - 
 * 0...65535; 0 = 
 * Out : -
 */
void SetRfReceiver(uint16_t timeout_ticks)
{	
	if (timeout_ticks != 0) {
	    StartCCR1Timeout(timeout_ticks);
	}
	SetRFMode(RF_RECEIVER);
}

/**
 * In  : -
 * Out : -
 */
void SetRfSleep(void)
{
	SetRFMode(RF_STANDBY);
	SetRFMode(RF_SLEEP);
}

/**
 * In  : -
 * Out : -
 */
void SetRfStandby(void)
{
	SetRFMode(RF_STANDBY);
}

/**
 * In  : -
 * Out : -
 */
void SetRfSynthesier(void)
{
	SetRFMode(RF_SYNTHESIZER);
}

/**
 * In  : timeout_ms -
 * 0...65535; 0 = 
 * Out : -
 */
void SetRfTransmiter(uint16_t timeout_ticks)
{
//	if ( timeout_ms != 0) {
//		while( RfTimeout.isRunning )
//		{
//		}
//		RfTimeout.value = timeout_ms;
//		RfTimeout.callback = OnTxTimeout;
//		SetTimeout(&RfTimeout);
//	}
	SetRFMode(RF_TRANSMITTER);
}

#pragma vector=PORT2_VECTOR
__interrupt void PortIRQ_Interrupt(void){
	
	if( (IRQ_IE & IRQ_1_PIN) && (ModeNow == RF_RECEIVER) ){
		BIT_CLEAR(IRQ_IFG, IRQ_1_PIN);
		ReadDataPacket(NULL);
	}
    _BIC_SR_IRQ(LPM3_bits+GIE);
}
//eof
