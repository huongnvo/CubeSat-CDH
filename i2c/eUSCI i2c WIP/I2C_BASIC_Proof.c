#include <stdio.h>
#include <msp430.h> 
#include <driverlib.h>

double convertHexToTemp(uint8_t UpperByte, uint8_t LowerByte);

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	
    /*
     * Write a report on the lessons i have learned, hand in a working prototypes
     *
     * Possible Issues:
     * 		Hardware:
     * 			wiring
     * 			rst pins
     * 			power ins to chips
     * 			I2C bus problems
     * 		Software:
 	 *
     *
     */

	printf("Hello World!\n");
	
    WDT_A_hold(WDT_A_BASE);

    //Set DCO frequency to 1MHz
    CS_setDCOFreq(CS_DCORSEL_0,CS_DCOFSEL_0);
    //Set ACLK = VLO with frequency divider of 1
    CS_initClockSignal(CS_ACLK,CS_VLOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);

    // Configure Pins for I2C
    //Set P1.6 and P1.7 as Secondary Module Function Input.
    /*

     * Select Port 1
     * Set Pin 6, 7 to input Secondary Module Function, (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL).
     */

    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P7,
        GPIO_PIN0 + GPIO_PIN1,
        GPIO_SECONDARY_MODULE_FUNCTION
        );


	PMM_unlockLPM5();

	EUSCI_B_I2C_initMasterParam param = {0};
		param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
		param.i2cClk = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
		param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;
	EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);

		EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, 0x18);

	    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

			//Enable I2C Module to start operations
			EUSCI_B_I2C_enable(EUSCI_B0_BASE);

			EUSCI_B_I2C_clearInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT);
			//Enable master Receive interrupt
			EUSCI_B_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT);

		printf("I2C initialized\n");

		EUSCI_B_I2C_masterSendStart(EUSCI_B0_BASE);
		EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, 0x05); // this is probably not hitting the right thing

		EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);

		    //Enable I2C Module to start operations
		    EUSCI_B_I2C_enable(EUSCI_B0_BASE);

		    EUSCI_B_I2C_clearInterrupt(EUSCI_B0_BASE,
		                               EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
		                               EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT +
		                               EUSCI_B_I2C_NAK_INTERRUPT);

		    //Enable master Receive interrupt
		    EUSCI_B_I2C_enableInterrupt(EUSCI_B0_BASE,
		                                EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
		                                EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT +
		                                EUSCI_B_I2C_NAK_INTERRUPT);

		EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);

		uint8_t UByte = EUSCI_B_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE);
		printf("This is the UByte! %u \n" , UByte); // this prints out 0? is it suppossed to do this?

		uint8_t LByte = EUSCI_B_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE); // Stuck
		printf("This is the LByte! %u \n" , LByte); // this prints out 0? is it suppossed to do this?

		EUSCI_B_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE);

		printf("Data transmission complete\n");

		double Temperature = convertHexToTemp(UByte, LByte);

		printf("This is the Temp! %f \n" , Temperature);

		//convert the temp to two bytes of voltage data

		/*
		EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, );
		EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
		EUSCI_B_I2C_enable(EUSCI_B0_BASE);
		EUSCI_B_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_STOP_INTERRUPT);
		*/
		//Send the two bytes to the dac

		//Delay

	}

double convertHexToTemp(uint8_t UpperByte, uint8_t LowerByte) {
	double Temperature;

	//First Check flag bits
	if ((UpperByte & 0x80) == 0x80) { 	/* TA ³ TCRIT  */	}
	if ((UpperByte & 0x40) == 0x40) { 	/* TA > TUPPER */	}
	if ((UpperByte & 0x20) == 0x20) { 	/* TA < TLOWER */	}

	UpperByte = UpperByte & 0x1F; 		//Clear flag bits
	if ((UpperByte & 0x10) == 0x10) { 	//TA < 0°C
		UpperByte = UpperByte & 0x0F;		//Clear SIGN
		Temperature = 256 - (UpperByte * 16 + LowerByte / 16);
	}
	else //TA ³ 0°C
	Temperature = (UpperByte * 16 + LowerByte / 16); //Temperature = Ambient Temperature (°C)

	return Temperature;
}
