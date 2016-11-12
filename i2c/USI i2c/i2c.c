#include <MSP430fr5994.h>
#include <stdint.h>
#include "i2c.h"

// Internal state
static uint16_t *i2c_sequence;
static uint16_t i2c_sequence_length;
static uint8_t *i2c_receive_buffer;
static uint16_t i2c_wakeup_sr_bits;
i2c_state_type i2c_state = I2C_IDLE;

void i2cInitialize() {
  i2c_init(/*usi_clock_divider*/, /*usi_clock_source*/);									                 //Convert to baudrate eUSCI HOW? 
}

uint16_t* i2cRead(uint16_t address, uint16_t registery, int numOfBytes) {
  uint16_t sequence_length = (uint16_t) (4 + numOfBytes);
  uint16_t sequence[] = malloc((sequence_length)*sizeof(uint16_t));
  sequence[0] = (address<<1);
  sequence[1] = registery;
  sequence[2] = I2C_RESTART;
  sequence[3] = (address<<1)|1;
  for(int i = 0; i < numOfBytes; i++) {
    sequence[i] = I2C_READ;
  }

  uint16_t data[] = malloc((numOfBytes)*sizeof(uint16_t));

  i2c_send_sequence(sequence, (uint16_t)sequence_length, &data, LPM0_BITS);

  free(sequence);
  return &data;
}

void i2cWrite(uint16_t address, uint16_t registery, uint16_t data) {
  uint16_t sequence_length = 0x03;
  uint16_t sequence[] = {(address<<1), registery, data};

  uint16_t data;

  i2c_send_sequence(sequence, sequence_length, &data, LPM0_BITS);

  free(sequence);
}

static inline void i2c_prepare_stop();
static inline void i2c_prepare_data_xmit_recv();
static void i2c_send_sequence(uint16_t *sequence, uint16_t sequence_length, uint8_t *received_data, uint16_t wakeup_sr_bits);

// Sends a command/data sequence that can include restarts, writes and reads. Every transmission begins with a START,
// and ends with a STOP so you do not have to specify that. Will busy-spin if another transmission is in progress. Note
// that this is interrupt-driven asynchronous code: you can't just call i2c_send_sequence from an interrupt handler and
// expect it to work: you risk a deadlock if another transaction is in progress and nothing will happen before the
// interrupt handler is done anyway. So the proper way to use this is in normal code. This should not be a problem, as
// performing such lengthy tasks as I2C communication inside an interrupt handler is a bad idea anyway.  wakeup_sr_bits
// should be a bit mask of bits to clear in the SR register when the transmission is completed (to exit LPM0: LPM0_BITS
// (CPUOFF), for LPM3: LPM3_bits (SCG1+SCG0+CPUOFF))
static void i2c_send_sequence(uint16_t *sequence, uint16_t sequence_length, uint8_t *received_data, uint16_t wakeup_sr_bits) {
  while(i2c_state != I2C_IDLE); // we can't start another sequence until the current one is done
  i2c_sequence = sequence;
  i2c_sequence_length = sequence_length;
  i2c_receive_buffer = received_data;
  i2c_wakeup_sr_bits = wakeup_sr_bits;
  i2c_state = I2C_START;
  USICTL1 |= USIIFG;            // actually start communication,  							          /* USI  Control Register 1 */ |= /* USI  Counter Interrupt Flag */
}

static inline void i2c_prepare_stop() {
  USICTL0 |= USIOE;             // SDA = output, 						   					                  /* USI  Control Register 0 */ |= /* USI  Output Enable */
  USISRL = 0x00;				        //															                          /* USI  Low Byte Shift Register */ = 0x00
  USICNT |=  0x01;              // Bit counter= 1, SCL high, SDA low,						          /* USI  Bit Counter Register */ |= 0x01
  i2c_state = I2C_STOP;
}

static inline void i2c_prepare_data_xmit_recv() {
  if(i2c_sequence_length == 0) {
    i2c_prepare_stop();         // nothing more to do, prepare to send STOP
  } else {
    if(*i2c_sequence == I2C_RESTART) {
      USICTL0 |= USIOE;         // SDA = output 											                    /* USI  Control Register 0 */ |= /* USI  Output Enable */
      USISRL = 0xff;            // prepare and send a dummy bit, so that SDA is high		  /* USI  Low Byte Shift Register */ = 0xff
      USICNT = (USICNT & 0xE0) | 1;	//														                        /* USI  Bit Counter Register */ = (/* USI  Bit Counter Register */ & 0xE0) | 1
      i2c_state = I2C_START;
    }
    else if(*i2c_sequence == I2C_READ) {
      USICTL0 &= ~USIOE;               // SDA = input 										                /* USI  Control Register 0 */ &= ~/* USI  Output Enable */
      USICNT = (USICNT & 0xE0) | 8;    // Bit counter = 8, RX data 							          /* USI  Bit Counter Register */ = (/* USI  Bit Counter Register */ & 0xE0) | 8
      i2c_state = I2C_RECEIVED_DATA;   // next state: Test data and ACK/NACK
    } else {                           // a write
      // at this point we should have a pure data byte, not a command, so (*i2c_sequence >> 8) == 0
      USICTL0 |= USIOE;                // SDA = output 										                /* USI  Control Register 0 */ |= /* USI  Output Enable */
      USISRL = (char)(*i2c_sequence);  // Load data byte 									                /* USI  Low Byte Shift Register */ = byte to be sent
      USICNT = (USICNT & 0xE0) | 8;    // Bit counter = 8, start TX 						          /* USI  Bit Counter Register */ = (/* USI  Bit Counter Register */ & 0xE0) | 8
      i2c_state = I2C_PREPARE_ACKNACK; // next state: prepare to receive data ACK/NACK
    }
    i2c_sequence++;
    i2c_sequence_length--;
  }
}

#pragma vector = USI_VECTOR		                                                            //will this have to be replaced as well?
__interrupt void USI_TXRX(void)                                                           //will this have to be replaced as well?
{
  switch(__even_in_range(i2c_state,12)) {                                                 //will this have to be replaced as well?
  case I2C_IDLE:
    break;

  case I2C_START:               // generate start condition
    USISRL = 0x00;				//															                                /* USI  Low Byte Shift Register */ = 0x00
    USICTL0 |= (USIGE|USIOE);	//														                              /* USI  Control Register 0 */ |= (/* USI  General Output Enable Latch */ | /* USI  Output Enable */)
    USICTL0 &= ~USIGE;			//															                              /* USI  Control Register 0 */ &= ~/* USI  General Output Enable Latch */ 
    i2c_prepare_data_xmit_recv();
    break;

  case I2C_PREPARE_ACKNACK:      // prepare to receive ACK/NACK
    USICTL0 &= ~USIOE;           // SDA = input 											                    /* USI  Control Register 0 */ &= ~/* USI  Output Enable */
    USICNT |= 0x01;              // Bit counter=1, receive (N)Ack bit 						        /* USI  Bit Counter Register */ |= 0x01
    i2c_state = I2C_HANDLE_RXTX; // Go to next state: check ACK/NACK and continue xmitting/receiving if necessary
    break;

  case I2C_HANDLE_RXTX:         // Process Address Ack/Nack & handle data TX
    if((USISRL & BIT0) != 0) {  // did we get a NACK? 										                (/* USI  Low Byte Shift Register */ & BIT0) != 0
      i2c_prepare_stop();
    } else {
      i2c_prepare_data_xmit_recv();
    }
    break;

  case I2C_RECEIVED_DATA:       // received data, send ACK/NACK
    *i2c_receive_buffer = USISRL; //														                          *dataToReceive = /* USI  Low Byte Shift Register */;
    i2c_receive_buffer++;																	
    USICTL0 |= USIOE;           // SDA = Output 											                    /* USI  Control Register 0 */ |= /* USI  Output Enable */
    if(i2c_sequence_length > 0) {
      // If this is not the last byte
      USISRL = 0x00;                // ACK 					    								                  /* USI  Low Byte Shift Register */ = 0x00
      i2c_state = I2C_HANDLE_RXTX;  // Go to next state: data/rcv again
    } else {                        // last byte: send NACK
      USISRL = 0xff;                // NACK 												                      /* USI  Low Byte Shift Register */ = 0xff
      i2c_state = I2C_PREPARE_STOP; // stop condition is next
    }
    USICNT |= 0x01;             // Bit counter = 1, send ACK/NACK bit 						        /* USI  Bit Counter Register */ |= 0x01
    break;

  case I2C_PREPARE_STOP:        // prepare stop condition
    i2c_prepare_stop();         // prepare stop, go to state 14 next
    break;

  case I2C_STOP:                // Generate Stop Condition
    USISRL = 0x0FF;             // USISRL = 1 to release SDA 								              /* USI  Low Byte Shift Register */ = 0xff
    USICTL0 |= USIGE;           // Transparent latch enabled 								              /* USI  Control Register 0 */ |= /* USI  General Output Enable Latch */
    USICTL0 &= ~(USIGE|USIOE);  // Latch/SDA output disabled								              /* USI  Control Register 0 */ &= ~(/* USI  General Output Enable Latch */ | /* USI  Output Enable */)
    i2c_state = I2C_IDLE;       // Reset state machine for next xmt
    if(i2c_wakeup_sr_bits) {
      _bic_SR_register_on_exit(i2c_wakeup_sr_bits); // exit active if prompted to 	      //will this have to be replaced as well?
    }
    break;
  }
  USICTL1 &= ~USIIFG;           // Clear pending Flag 										                /* USI  Control Register 1 */ &= ~/* USI  Counter Interrupt Flag */
}

// Call this with one of the USIDIV_* constants as a usi_clock_divider parameter, which will set the clock divider used
// for USI I2C communications. The usi_clock_source parameter should be set to one of the USISSEL* constants. Example:
// i2c_init(USIDIV_5, USISSEL_2) uses SMCLK/16.
static inline void i2c_init(uint16_t usi_clock_divider, uint16_t usi_clock_source) {
  _disable_interrupts();																	//will this have to be replaced as well?
  USICTL0 = USIPE6|USIPE7|USIMST|USISWRST;  // Port & USI mode setup						          /* USI  Control Register 1 */ = /* USI  Port Enable Px.6 */ | /* USI  Port Enable Px.7 */ | /* USI  Master Select  0:Slave / 1:Master */ | /* USI  Software Reset */
  USICTL1 = USII2C|USIIE;                   // Enable I2C mode & USI interrupt 				    /* USI  Control Register 1 */ = /* USI  I2C Mode */ | /* USI  Counter Interrupt enable */
  USICKCTL = usi_clock_divider | usi_clock_source | USICKPL; //Convert to baudrate HOW? 	/* USI  Clock Control Register */ = usi_clock_divider | usi_clock_source | USICKPL
  USICNT |= USIIFGCC;                       // Disable automatic clear control 				    /* USI  Bit Counter Register */ |= /* USI  Interrupt Flag Clear Control */
  USICTL0 &= ~USISWRST;                     // Enable USI 									              /* USI  Control Register 0 */ &= ~/* USI  Software Reset */
  USICTL1 &= ~USIIFG;                       // Clear pending Flag 							          /* USI  Control Register 1 */ &= ~/* USI  Counter Interrupt Flag */
  _enable_interrupts();																		                                //will this have to be replaced as well?
}