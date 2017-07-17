#include <msp430.h>
#include "MSP430F5xx_6xx/driverlib.h"
#include <math.h>

// ----------- CLOCK ----------------------------------------
#define CLOCK_FREQUENCY  24000000        // (hertz)
#define TIMER_COUNTER    480           // Number of clock cycles before every timer interrupt
                                        // Here it also represents the baud rate of li-fi transmission (CLOCK_SPEED / TIMER_COUNTER)
                                        // Can't go under 8000 for now
// ----------------------------------------------------------
// ----------- UART TRANSMISSION ----------------------------
#define UART_BAUD_RATE      115200      // (bit/s) - the communication with the computer
// ----------------------------------------------------------
// ----------- SELECT BUFFER SIZE ---------------------------
#define BUFFER_SIZE    32          // (bytes)
#define PACKET_SIZE    1 + BUFFER_SIZE * 8 + sizeof(crc) * 8 + 1        // (bits)   (Don't change this)
// ----------------------------------------------------------
// ----------- SELECT START/STOP BITS -----------------------
#define START_BIT      1
#define STOP_BIT       0
// ----------------------------------------------------------
// ----------- CRC ------------------------------------------
typedef char crc;                   // Decides of the size of the crc (8 or 16 only)
#define POLYNOMIAL 0x31                 // The generator polynomial
#define WIDTH  (8 * sizeof(crc))        // The crc's width (Don't change this)
#define TOPBIT (1 << (WIDTH - 1))       // Leftmost bit (Don't change this)
// ----------------------------------------------------------


//functions
void acquireData();
void sendPacket();
void crcInit(void);
crc calculateChecksum(char const message[], int nBytes);


//attributes
char temp;
volatile unsigned long i = 0;
crc crcTable[256];
int buffer_pos;
char buffer[BUFFER_SIZE];
char packet[PACKET_SIZE];    //start bit + data bits + crc + stop bit
volatile unsigned int data_received, timer_active;
volatile unsigned int sending;
uint32_t smclk;

//interruption flags
int USCI_A1_INT = 0;

/*
 * 1. Stop the Watchdog Timer
 * 2. Setup the Oscillators and clocks and check for their correct operation using the appropriate handlers
 * 3. Setup The I/O pins
 * 4. Setup the rest of the modules to be used
 * 5. start the application specific code
 */

int main(void) {

    WDTCTL = WDTPW+WDTHOLD;                   // Stop watchdog timer

    // SET CLOCK
    UCS_initClockSignal(UCS_SMCLK, UCS_DCOCLK_SELECT, UCS_CLOCK_DIVIDER_1);    // Select SMCLK to be DCO
    PMM_setVCore(PMM_CORE_LEVEL_3);                                            // Set VCore Up to level 3 to be able to run higher than 20 MHz
    UCS_initFLLSettle(CLOCK_FREQUENCY*2, CLOCK_FREQUENCY / 32768);             // Set clock source to selected frequency

    smclk = UCS_getSMCLK();

    P7DIR |= BIT7;
    P7SEL |= BIT7;                            // To see the clock output on P7.7

    // SET A/D CONVERTER AND REF
    P6DIR &= ~BIT0;                         // Set P6.0 as input
    P6SEL |= BIT0;                          // Activate alternate function
    REFCTL0 &= ~REFMSTR;
    ADC12CTL0 = ADC12ON | ADC12SHT0_4 | ADC12REFON | ADC12REF2_5V;
    ADC12CTL1 = ADC12SHP;
    ADC12MCTL0 = ADC12SREF_1 + ADC12INCH_0; // V+=Vref+  and  V-=AVss,


    for (i = 0; i < 0x30; i++);             // Delay for reference start-up

    ADC12CTL0 |= ADC12ENC;                  // Enable conversions


    // SET TIMER
    TA0CCTL0 = CCIE;                        // CCR0 interrupt enabled
    TA0CCR0 = TIMER_COUNTER;                // Sample every X cycles
    TA0CTL = TASSEL_2 + MC_1 + TACLR;


    // SET UART
    P4SEL |= BIT4 + BIT5;                           // P4.4 = TX  and  P4.5 = RX
    UCA1CTL1 |= UCSWRST;                            // Reset the UART state machine
    UCA1CTL1 |= UCSSEL_2;                           // SMCLK
    double n = CLOCK_FREQUENCY / UART_BAUD_RATE;     // Variable used to calculate the next settings
    UCA1BR0 = (int)(n);
    UCA1BR1 = 0;                                    // Set baud rate
    int modulation = round((n - (int)(n))*8);
    UCA1MCTL |= modulation + UCBRF_0;               // Select the correct modulation
    UCA1CTL1 &= ~UCSWRST;                           // Start the UART state machine
    UCA1IE |= UCRXIE;                               // Enable USCI_A1 RX interrupts


    P2DIR |= BIT0;              //Set output pin (P2.0)
    P2OUT |= BIT0;

    timer_active = 0;
    data_received = 0;
    buffer_pos = 0;
    sending = 0;

    crcInit();
    CRC_setSeed(CRC_BASE, 0x0000);

    __enable_interrupt();


    while(1) {

        __bis_SR_register(LPM0_bits + GIE);       // CPU off, enable interrupts
        __no_operation();                         // For debugger

        sendPacket();

        data_received = 0;
        buffer_pos = 0;
    }

    return 0;
}


// Character received
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    data_received = 1;

    switch(__even_in_range(UCA1IV, 4))
    {
    case 0 : break;                 // Vector 0 - no interrupt
    case 2 :                        // Vector 2 - RXIFG9
        while(!(UCA1IFG & UCTXIFG));

        UCA1TXBUF = UCA1RXBUF;
        buffer[buffer_pos] = UCA1RXBUF;
        CRC_set8BitData(CRC_BASE, buffer[buffer_pos]);        // Calculate CRC for this byte
        buffer_pos++;

        if(buffer_pos == BUFFER_SIZE) {
            UCA1IE &= ~UCRXIE;          // Disable USCI_A1 RX interrupts
            __bic_SR_register_on_exit(LPM0_bits);
        }

        break;
    case 4 : break;                 // Vector 4 - TXIFG
    default : break;
    }
}

// Timer0 A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    /*
    //Data recuperation from the Analog/Digital Converter
    ADC12CTL0 |= ADC12SC;                   // Start conversion
    while(!(ADC12IFG & BIT0));              // Wait for conversion completion
    temp = ADC12MEM0;*/

    if (sending && (__get_SR_register_on_exit() & CPUOFF)) {
        __bic_SR_register_on_exit(LPM0_bits);
    }

}

void acquireData() {
    //Data will eventually come from the sensor
}

void sendPacket() {

    sending = 1;

    // start bit
    __bis_SR_register(LPM0_bits + GIE);       // CPU off, enable interrupts
                                              //always wait for the right time to acquire data
    P2OUT = ~(0xFE | START_BIT);

    // data bits
    for (i = 0; i < BUFFER_SIZE * 8; i++) {
        __bis_SR_register(LPM0_bits + GIE);       // CPU off, enable interrupts
                                                  //always wait for the right time to acquire data
        P2OUT = ~(0xFE | ((buffer[(int)(i/8)] >> i%8) & 1));
    }

    // checksum
    crc checksum = (crc)CRC_getResult(CRC_BASE);
    for (i = 0; i < sizeof(crc) * 8; i++) {
        __bis_SR_register(LPM0_bits + GIE);       // CPU off, enable interrupts
                                                  //always wait for the right time to acquire data
        P2OUT = ~(0xFE | ((checksum >> i) & 1));
    }


    // stop bit
    __bis_SR_register(LPM0_bits + GIE);       // CPU off, enable interrupts
                                              //always wait for the right time to acquire data
    P2OUT = ~(0xFE & STOP_BIT);

    CRC_setSeed(CRC_BASE, 0x0000);            // Reset CRC signature
    UCA1IFG &= ~UCRXIFG;                      // Clear RX interrupt flag
    UCA1IE |= UCRXIE;                         // Enable USCI_A1 RX interrupts

/*
    char test = 0;
    while(1) {
        __bis_SR_register(LPM0_bits + GIE);       // CPU off, enable interrupts
                                                  //always wait for the right time to acquire data
        for (i = 0; i < 8; i++)  {
            int bit = (test >> i % 8) & 1;
            if (bit == 1) {
                P2OUT &= ~BIT0;
            }
            else {
                P2OUT |= BIT0;
            }
        }
        test++;
    }*/

    sending = 0;
}


void crcInit(void)
{
    crc remainder;

    /*
     * Compute the remainder of each possible dividend.
     */
    int dividend;
    for (dividend = 0; dividend < 256; ++dividend)
    {
        /*
         * Start with the dividend followed by zeros.
         */
        remainder = dividend << (WIDTH - 8);

        /*
         * Perform modulo-2 division, a bit at a time.
         */
        int bit;
        for (bit = 8; bit > 0; --bit)
        {
            /*
             * Try to divide the current data bit.
             */
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }

        /*
         * Store the result into the table.
         */
        crcTable[dividend] = remainder;
    }

}   /* crcInit() */



crc calculateChecksum(char const message[], int nBytes)
{
    char data;
    crc remainder = 0;


    /*
     * Divide the message by the polynomial, a byte at a time.
     */
    int byte;
    for (byte = 0; byte < nBytes; ++byte) {
        data = message[byte] ^ (remainder >> (WIDTH - 8));
        remainder = crcTable[data] ^ (remainder << 8);
    }

    /*
     * The final remainder is the CRC.
     */
    return (remainder);

}

