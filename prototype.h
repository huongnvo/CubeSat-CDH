#ifndef PROTOTYPE_H
#define PROTOTYPE_H
2#define CNF1_REG 0x2A
#define CNF2_REG 0x29
#define CNF3_REG 0x28
#define CANCTRL_REG 0x0F
#define TXBNCTRL_REG 0x48

#define SPI_SPEED 14000000

#define WRITE_INSTR 0X02
#define READ_INSTR 0x03
#define RESET_INSTR 0xC0
#define READ_STATUS_INSTR 0xA0
#define RX_STATUS_INSTR 0xB0
#define BIT_MODIFY_INSTR 0x05

#define NORMAL_MODE 0X00
#define SLEEP_MODE 0x01
#define LOOPBACK_MODE 0X02
#define LISTEN_ONLY_MODE 0x03
#define CONFIG_MODE 0x04

#define HIGH 1
#define LOW 0

#define BLAH_PIN 4
#define BLOB_PIN 6

#define EUSCI_B_SPI_MSB_FIRST UCMSB
#define EUSCI_B_SPI_LSB_FIRST 0x00

#define EUSCI_B_SPI_TRANSMIT_INTERRUPT UCTXIE
#define EUSCI_B_SPI_RECEIVE_INTERRUPT UCRXIE
// structName.data = HIGH << BLAH_PIN | HIGH << BLOB_PIN;
typedef struct 
{
 	uint8_t instruction;
 	uint8_t address;

} ReadInstruction_t;

typedef struct 
{
	uint8_t instruction;
	uint8_t address;
	uint8_t data;
} WriteInstruction_t;

typedef struct
{
	uint8_t instruction;
} Instruction_t;

typedef struct 
{
	uint8_t instruction;
	uint8_t data;
} LoadTxBuffer_t;

typedef struct
{
	uint8_t instruction;
	uint8_t address;
	uint8_t mask;
	uint8_t data;
} ModifyBitInstruction_t;

typedef struct EUSCI_B_SPI_initMasterParam
{
    //! Selects Clock source. Refer to device specific datasheet for available
    //! options.
    //! \n Valid values are:
    //! - \b EUSCI_B_SPI_CLOCKSOURCE_ACLK
    //! - \b EUSCI_B_SPI_CLOCKSOURCE_SMCLK
    uint8_t selectClockSource;
    //! Is the frequency of the selected clock source
    uint32_t clockSourceFrequency;
    //! Is the desired clock rate for SPI communication
    uint32_t desiredSpiClock;
    //! Controls the direction of the receive and transmit shift register.
    //! \n Valid values are:
    //! - \b EUSCI_B_SPI_MSB_FIRST
    //! - \b EUSCI_B_SPI_LSB_FIRST [Default]
    uint16_t msbFirst;
    //! Is clock phase select.
    //! \n Valid values are:
    //! - \b EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT [Default]
    //! - \b EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
    uint16_t clockPhase;
    //! Is clock polarity select
    //! \n Valid values are:
    //! - \b EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
    //! - \b EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW [Default]
    uint16_t clockPolarity;
    //! Is SPI mode select
    //! \n Valid values are:
    //! - \b EUSCI_B_SPI_3PIN
    //! - \b EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_HIGH
    //! - \b EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_LOW
    uint16_t spiMode;
} EUSCI_B_SPI_initMasterParam;

typedef struct EUSCI_B_SPI_initSlaveParam
{
    //! Controls the direction of the receive and transmit shift register.
    //! \n Valid values are:
    //! - \b EUSCI_B_SPI_MSB_FIRST
    //! - \b EUSCI_B_SPI_LSB_FIRST [Default]
    uint16_t msbFirst;
    //! Is clock phase select.
    //! \n Valid values are:
    //! - \b EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT [Default]
    //! - \b EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
    uint16_t clockPhase;
    //! Is clock polarity select
    //! \n Valid values are:
    //! - \b EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
    //! - \b EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW [Default]
    uint16_t clockPolarity;
    //! Is SPI mode select
    //! \n Valid values are:
    //! - \b EUSCI_B_SPI_3PIN
    //! - \b EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_HIGH
    //! - \b EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_LOW
    uint16_t spiMode;
} EUSCI_B_SPI_initSlaveParam;


void initialize();

void writeCAN(unsigned char data[8]);

void readCAN( unsigned char data[8]);



#endif // PROTOTYPE_H

