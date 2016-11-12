#include <MSP430fr5994.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <i2c.h>

// Internal state
static uint16_t *i2c_sequence;
static uint16_t i2c_sequence_length;
static uint8_t *i2c_receive_buffer;
static uint16_t i2c_wakeup_sr_bits;

static inline void i2c_prepare_data_xmit_recv();
static void i2c_send_sequence(uint16_t *sequence, uint16_t sequence_length, uint8_t *received_data, uint16_t wakeup_sr_bits);

// Call this with one of the USIDIV_* constants as a usi_clock_divider parameter, which will set the clock divider used
// for USI I2C communications. The usi_clock_source parameter should be set to one of the USISSEL* constants. Example:
// i2c_init(USIDIV_5, USISSEL_2) uses SMCLK/16.
void i2c_init(uint16_t baudrate) {
	_disable_interrupts();				//will this have to be replaced as well?

	//USICTL0 = USIPE6|USIPE7|USIMST|USISWRST;  // Port & USI mode setup						          /* USI  Control Register 1 */ = /* USI  Port Enable Px.6 */ | /* USI  Port Enable Px.7 */ | /* USI  Master Select  0:Slave / 1:Master */ | /* USI  Software Reset */
	P1OUT &= ~BIT0; // Clear P1.0 output latch <--- This may have to go in the state machine
	P1SEL0 &= ~(BIT4 | BIT5);               // Configure I2C pins
	P1SEL1 |= BIT4 | BIT5;
	//UCB0CTLW0 |= UCMODE_3 | UCMST | UCSYNC; // I2C mode, Master mode, sync

	// Disable the GPIO power-on default high-impedance mode to activate
	// previously configured port settings
	PM5CTL0 &= ~LOCKLPM5;
	//        ^
	//        | This looks important.

	//USICTL1 = USII2C|USIIE;                   // Enable I2C mode & USI interrupt 				    /* USI  Control Register 1 */ = /* USI  I2C Mode */ | /* USI  Counter Interrupt enable */
	//   TODO          ^
	//                 |      Account for inturrupt?

	UCB0BRW = baudrate;                       // baudrate = SMCLK / 8 =0x0008
	//USICKCTL = usi_clock_divider | usi_clock_source | USICKPL; //Convert to baudrate HOW? 	/* USI  Clock Control Register */ = usi_clock_divider | usi_clock_source | USICKPL

	//USICNT |= USIIFGCC;                       // Disable automatic clear control 				    /* USI  Bit Counter Register */ |= /* USI  Interrupt Flag Clear Control */
	// TODO

	//USICTL0 &= ~USISWRST;                     // Enable USI 									              /* USI  Control Register 0 */ &= ~/* USI  Software Reset */
	UCB0CTLW0 |= UCSWRST;

	//USICTL1 &= ~USIIFG;                       // Clear pending Flag 							          /* USI  Control Register 1 */ &= ~/* USI  Counter Interrupt Flag */
	//TODO

	//UCB0IE |= UCRXIE | UCNACKIE | UCBCNTIE | UCTXIE0 | UCSTPIE;

	_enable_interrupts();				//will this have to be replaced as well?
}

uint8_t *i2cRead(uint16_t address, uint16_t registery, uint16_t numOfBytes) {

	UCB0TBCNT = numOfBytes;                 // number of bytes to be received

	uint16_t sequence_length = (uint16_t) (4 + numOfBytes);
	uint16_t * sequence;
	sequence = malloc(sizeof(uint16_t)*(int)(sequence_length));
	sequence[0] = (address << 1);
	sequence[1] = registery;
	sequence[2] = I2C_RESTART;
	sequence[3] = (address << 1) | 1;
	uint16_t i;
	for (i = 0x00; i < numOfBytes; i++) {
		sequence[i] = I2C_READ;
	}

	uint8_t * RXData;
	RXData = malloc((int)(numOfBytes) * sizeof(uint8_t));

	i2c_send_sequence(sequence, sequence_length, RXData, LPM0_bits);

	free(sequence);
	return RXData;
}

void i2cWrite(uint16_t address, uint16_t registery, uint16_t data) {
	UCB0CTLW0 |= UCMODE_3 | UCSYNC;         // I2C mode, sync mode
	UCB0IE |= UCTXIE0 | UCSTPIE;            // transmit,stop interrupt enable

	uint16_t sequence_length = 0x03;
	uint16_t * sequence;
	sequence = malloc(sizeof(uint16_t)*(4));
	sequence[0] = (address << 1);
	sequence[1] = registery;
	sequence[2] = data;
	sequence[3] = (address << 1) | 1;

	printf("%u \n", sequence[0]);
	printf("%u \n", sequence[1]);
	printf("%u \n", sequence[2]);
	printf("%u \n", sequence[3]);

	uint8_t * RXData;
	RXData = malloc(sizeof(uint8_t));

	i2c_send_sequence(sequence, sequence_length, RXData, LPM0_bits);

	free(RXData);
	free(sequence);
}

// Sends a command/data sequence that can include restarts, writes and reads. Every transmission begins with a START,
// and ends with a STOP so you do not have to specify that. Will busy-spin if another transmission is in progress. Note
// that this is interrupt-driven asynchronous code: you can't just call i2c_send_sequence from an interrupt handler and
// expect it to work: you risk a deadlock if another transaction is in progress and nothing will happen before the
// interrupt handler is done anyway. So the proper way to use this is in normal code. This should not be a problem, as
// performing such lengthy tasks as I2C communication inside an interrupt handler is a bad idea anyway.  wakeup_sr_bits
// should be a bit mask of bits to clear in the SR register when the transmission is completed (to exit LPM0: LPM0_bits
// (CPUOFF), for LPM3: LPM3_bits (SCG1+SCG0+CPUOFF))
static void i2c_send_sequence(uint16_t *sequence, uint16_t sequence_length, uint8_t *received_data, uint16_t wakeup_sr_bits) {
	while (UCB0CTL1 & UCTXSTP);         // Ensure stop condition got sent
	i2c_sequence = sequence;
	i2c_sequence_length = sequence_length;
	i2c_receive_buffer = received_data;
	i2c_wakeup_sr_bits = wakeup_sr_bits;
	UCB0CTL1 |= UCTXSTT;
	__bis_SR_register(i2c_wakeup_sr_bits | GIE); // Enter LPM0 w/ interrupt
}

static inline void i2c_prepare_data_xmit_recv() {
	if (i2c_sequence_length == 0) {
		UCB0CTL1 |= UCTXSTP;         // nothing more to do, prepare to send STOP
	} else {
		if (*i2c_sequence == I2C_RESTART) {
			//USICTL0 |= USIOE;         // SDA = output 											                    /* USI  Control Register 0 */ |= /* USI  Output Enable */
			//USISRL = 0xff;            // prepare and send a dummy bit, so that SDA is high		  /* USI  Low Byte Shift Register */ = 0xff
			//USICNT = (USICNT & 0xE0) | 1;	//														                        /* USI  Bit Counter Register */ = (/* USI  Bit Counter Register */ & 0xE0) | 1
			//i2c_state = I2C_START;
			UCB0CTL1 |= UCTXSTT;

		} else if (*i2c_sequence == I2C_READ) {
			//USICTL0 &= ~USIOE;               // SDA = input 										                /* USI  Control Register 0 */ &= ~/* USI  Output Enable */
			//USICNT = (USICNT & 0xE0) | 8;    // Bit counter = 8, RX data 							          /* USI  Bit Counter Register */ = (/* USI  Bit Counter Register */ & 0xE0) | 8
			//i2c_state = I2C_RECEIVED_DATA;   // next state: Test data and ACK/NACK
			*i2c_receive_buffer = UCB0RXBUF;             // Get RX data
			i2c_receive_buffer++;
		} else {                           // a write
			// at this point we should have a pure data byte, not a command, so (*i2c_sequence >> 8) == 0
			//USICTL0 |= USIOE;                // SDA = output 										                /* USI  Control Register 0 */ |= /* USI  Output Enable */

			UCB0TXBUF = (char) (*i2c_sequence);
			//USISRL = (char)(*i2c_sequence);  // Load data byte 									                /* USI  Low Byte Shift Register */ = byte to be sent
			//USICNT = (USICNT & 0xE0) | 8;    // Bit counter = 8, start TX 						          /* USI  Bit Counter Register */ = (/* USI  Bit Counter Register */ & 0xE0) | 8

			//i2c_state = I2C_PREPARE_ACKNACK; // next state: prepare to receive data ACK/NACK
		}
		i2c_sequence++;
		i2c_sequence_length--;
	}
}

#pragma vector = EUSCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)      //will this have to be replaced as well?
{
	switch (__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG)) {
	case USCI_NONE:
		break;     // Vector 0: No interrupts
	case USCI_I2C_UCALIFG:
		break;     // Vector 2: ALIFG
	case USCI_I2C_UCNACKIFG:            // Vector 4: NACKIFG
		UCB0CTL1 |= UCTXSTP;
		__bic_SR_register_on_exit(i2c_wakeup_sr_bits);
		break;

	case USCI_I2C_UCSTTIFG:
		break;     // Vector 6: STTIFG
	case USCI_I2C_UCSTPIFG:             // Vector 8: STPIFG
		UCB0CTL1 |= UCTXSTP;
		__bic_SR_register_on_exit(i2c_wakeup_sr_bits);
		break;

	case USCI_I2C_UCRXIFG3:
		break;     // Vector 10: RXIFG3
	case USCI_I2C_UCTXIFG3:
		break;     // Vector 12: TXIFG3
	case USCI_I2C_UCRXIFG2:
		break;     // Vector 14: RXIFG2
	case USCI_I2C_UCTXIFG2:
		break;     // Vector 16: TXIFG2
	case USCI_I2C_UCRXIFG1:
		break;     // Vector 18: RXIFG1
	case USCI_I2C_UCTXIFG1:
		break;     // Vector 20: TXIFG1
	case USCI_I2C_UCRXIFG0:             // Vector 22: RXIFG0
		i2c_prepare_data_xmit_recv();
		break;

	case USCI_I2C_UCTXIFG0:             // Vector 24: TXIFG0
		i2c_prepare_data_xmit_recv();
		break;

	case USCI_I2C_UCBCNTIFG:            // Vector 26: BCNTIFG
		UCB0CTL1 |= UCTXSTP;
		__bic_SR_register_on_exit(i2c_wakeup_sr_bits);
		break;

	case USCI_I2C_UCCLTOIFG:
		break;     // Vector 28: clock low timeout
	case USCI_I2C_UCBIT9IFG:
		break;     // Vector 30: 9th bit
	default:
		break;
	}
}
