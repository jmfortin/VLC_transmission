#include <msp430.h>
#include "MSP430F5xx_6xx/driverlib.h"

// ----------- CLOCK ----------------------------------------
#define CLOCK_FREQUENCY  8000000       // (hertz)
#define TIMER_COUNTER    1600           // Number of clock cycles before every timer interrupt
                                        // Here it also represents the baud rate of transmission (CLOCK_SPEED / TIMER_COUNTER)
// ----------------------------------------------------------
// ----------- SELECT BUFFER SIZE ---------------------------
#define BUFFER_SIZE    32          // (bytes)
#define PACKET_SIZE    1 + BUFFER_SIZE * 8 + sizeof(crc) * 8 + 1        // (bits)
// ----------------------------------------------------------
// ----------- SELECT START/STOP BITS -----------------------
#define START_BIT      1
#define STOP_BIT       0
// ----------------------------------------------------------
// ----------- PARITY BIT -----------------------------------
#define PARITY         0           // 0 = even, 1 = odd
// ----------------------------------------------------------

typedef char crc;

#define WIDTH  (8 * sizeof(crc))
#define TOPBIT (1 << (WIDTH - 1))
#define POLYNOMIAL 0x31  /* 11011 followed by 0's */

crc crcTable[256];

//functions
void receivePacket();
void retrieveData();
void crcInit();
void verifyData(char const message[]);
void sendToComputer();
void error();

//attributes
volatile unsigned char temp;
volatile unsigned long i = 0;
char buffer[BUFFER_SIZE];
char packet[PACKET_SIZE];    //start bit + data bits + crc + stop bit
crc checksum;
volatile unsigned int receiving, timer_active;
volatile unsigned int packet_error, ready;
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

int main(void)
{
    WDTCTL = WDTPW+WDTHOLD;                   // Stop watchdog timer

    // SET CLOCK
    UCS_initClockSignal(UCS_SMCLK, UCS_DCOCLK_SELECT, UCS_CLOCK_DIVIDER_1);    // Select SMCLK to be DCO
    PMM_setVCore(PMM_CORE_LEVEL_3);                                            // Set VCore Up to level 3 to be able to run higher than 20 MHz
    UCS_initFLLSettle(CLOCK_FREQUENCY*2, CLOCK_FREQUENCY / 32768);             // Set clock source to selected frequency

    smclk = UCS_getSMCLK();

    P7DIR |= BIT7;
    P7SEL |= BIT7;                            // To see the clock output on P7.7


    // SET VARIABLES
    receiving = 0;
    timer_active = 0;
    packet_error = 0;

    crcInit();

    ready = 1;


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
    TA0CCR0 = TIMER_COUNTER;                // Sample 5000 times per second
    TA0CTL = TASSEL_2 + MC_1 + TACLR;


    // SET UART
    P4SEL |= BIT4 + BIT5;               // P4.4 = TX  and  P4.5 = RX
    UCA1CTL1 |= UCSWRST;                // Reset the UART state machine
    UCA1CTL1 |= UCSSEL_2;               // SMCLK
    UCA1BR0 = 69;
    UCA1BR1 = 0;                        // Set baud rate to 115200 (User's guide)
                                        // http://processors.wiki.ti.com/index.php?title=USCI_UART_Baud_Rate_Gen_Mode_Selection&oldid=122242
    UCA1MCTL |= UCBRS_4 + UCBRF_0;      // Select the correct modulation
    UCA1CTL1 &= ~UCSWRST;               // Start the UART state machine
    UCA1IE |= UCRXIE;                   // Enable USCI_A1 RX interrupts


    P2DIR &= ~BIT0;     //input pin
    P2REN |= BIT0;      //set pull-up resistor
    P2OUT |= BIT0;
    P2IE |= BIT0;       //enable interrupts for pin P2.0
    P2IES &= ~BIT0;     //interrupt on rising edge


    __enable_interrupt();

    while(1) {

        __bis_SR_register(LPM0_bits + GIE);       // CPU off, enable interrupts
        __no_operation();                         // For debugger

        receivePacket();
        retrieveData();
        verifyData(buffer);

        if (packet_error == 0) {
            sendToComputer();
        }
        else {
            error();
        }


        packet_error = 0;
        ready = 1;
    }

    return 0;
}

// Port 2 interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    if (ready == 1) {
        TA0R = TIMER_COUNTER / 2;
        ready = 0;
        __bic_SR_register_on_exit(LPM0_bits);
    }

    P2IFG &= (~BIT0); // P2.0 IFG clear
}

// Character received
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch(__even_in_range(UCA1IV, 4))
    {
    case 0 : break;                 // Vector 0 - no interrupt
    case 2 :                        // Vector 2 - RXIFG9
        while(!(UCA1IFG & UCTXIFG));

        break;
    case 4 :
        break;                 // Vector 4 - TXIFG
    default : break;
    }
}

// Timer0 A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    if (receiving && (__get_SR_register_on_exit() & CPUOFF)) {
        __bic_SR_register_on_exit(LPM0_bits);
    }
}

void receivePacket() {

    receiving = 1;

    long pos = 0;
    while (pos < PACKET_SIZE) {
        __bis_SR_register(LPM0_bits + GIE);       // CPU off, enable interrupts
                                                  //always wait for the right time to acquire data
        packet[pos] = P2IN & BIT0;
        pos++;
    }

    receiving = 0;
}


void retrieveData() {

    temp = 0;
    unsigned int pos;
    for (pos = 1; pos < BUFFER_SIZE * 8 + 1; pos++) {

        temp |= packet[pos] << (pos-1) % 8;
        if (pos != 0 && pos % 8 == 0) {
            buffer[((pos-1) / 8)] = temp;
            temp = 0;
        }
    }
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


void verifyData(char const message[])
{

    // Verify start and stop bits
    if (packet[0] != START_BIT || packet[PACKET_SIZE - 1] != STOP_BIT) {

        packet_error = 1;
        return;
    }

    checksum = 0;

    // Find the checksum from the received packet
    for(i = 0; i < sizeof(crc) * 8; i++) {

        checksum |= packet[BUFFER_SIZE*8 + 1 + i] << i % (sizeof(crc) * 8);
    }

    char data;
    crc remainder = 0;

    /*
     * Divide the message by the polynomial, a byte at a time.
     */
    int byte;
    for (byte = 0; byte < BUFFER_SIZE; ++byte) {
        data = message[byte] ^ (remainder >> (WIDTH - 8));
        remainder = crcTable[data] ^ (remainder << 8);
    }

    /*
     * The final remainder is the CRC.
     */
    if (remainder != checksum){
        packet_error = 1;
    }

}


void sendToComputer() {

    for (i = 0; i < BUFFER_SIZE; i++) {
        while(UCA1STAT & UCBUSY);
        UCA1TXBUF = buffer[i];
    }
}

void error() {
    char error[9] = "\n\rERROR\n\r";
    for (i = 0; i < 9; i++) {
            while(UCA1STAT & UCBUSY);
            UCA1TXBUF = error[i];
        }
}


