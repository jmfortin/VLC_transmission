#include <msp430.h>

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
int verifyCRC(char const message[], crc check);
void sendToComputer();

//attributes
volatile unsigned char temp;
volatile unsigned int i = 0;
char buffer[BUFFER_SIZE];
char packet[PACKET_SIZE];    //start bit + data bits + crc + stop bit
crc checksum;
volatile unsigned int receiving, timer_active;
volatile unsigned int packet_error;

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
    UCSCTL3 = SELREF_2;                       // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;                        // Set ACLK = REFO
    UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx

    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
        SFRIFG1 &= ~OFIFG;
    } while(SFRIFG1 & OFIFG);

    __bis_SR_register(SCG0);                  // Disable the FLL control loop

    UCSCTL1 = DCORSEL_5;                      // Select DCO range 16MHz operation
    UCSCTL2 |= 249;                           // Set DCO Multiplier for 8MHz
                                              // (N + 1) * FLLRef = Fdco
                                              // (249 + 1) * 32768 = 8MHz

    __bic_SR_register(SCG0);                  // Enable the FLL control loop

    __delay_cycles(250000);                   // Settling time for the DCO

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
    TA0CCR0 = 1600;                         // Sample 5000 times per second
    TA0CTL = TASSEL_2 + MC_1 + TACLR;


    // SET UART
    P4SEL |= BIT4 + BIT5;               // P4.4 = TX  and  P4.5 = RX
    UCA1CTL1 |= UCSWRST;                // Reset the UART state machine
    UCA1CTL1 |= UCSSEL_2;               // SMCLK
    UCA1BR0 = 69;
    UCA1BR1 = 0;                        // Set baud rate to 115200 (User's guide)
    UCA1MCTL |= UCBRS_4 + UCBRF_0;      // Select the correct modulation
    UCA1CTL1 &= ~UCSWRST;               // Start the UART state machine
    UCA1IE |= UCRXIE;                   // Enable USCI_A1 RX interrupts


    P2DIR &= ~BIT0;     //input pin
    P2REN |= BIT0;      //set pull-up resistor
    P2OUT |= BIT0;
    P2IE |= BIT0;       //enable interrupts for pin P2.0
    P2IES &= ~BIT0;     //interrupt on rising edge

    receiving = 0;
    timer_active = 0;
    packet_error = 0;

    crcInit();

    __enable_interrupt();

    while(1) {
        if((P2IN & BIT0) == 1) {

            receivePacket();
            retrieveData();

            if (packet_error == 0) {
                sendToComputer();
            }

            packet_error = 0;
        }
    }

    return 0;
}

// Port 2 interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    P2IFG &= (~BIT0); // P2.0 IFG clear
    TA0R = 800;
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
    case 4 : break;                 // Vector 4 - TXIFG
    default : break;
    }
}

// Timer0 A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    if (receiving) {
        timer_active = 1;
    }
}

void receivePacket() {

    receiving = 1;

    i = 0;
    while (i < PACKET_SIZE) {
        while(timer_active == 0);       //always wait for the right time to acquire data
        timer_active = 0;
        packet[i] = P2IN & BIT0;
        i++;
    }


    /*
    //Receive start bit
    while(timer_active == 0);           //always wait for the right time to acquire data
    if(P2IN & BIT0 != START_BIT) {
        packet_error = 1;
        return;
    }

    //Receive data
    i = 0;
    while (i < BUFFER_SIZE*8) {
        while(timer_active == 0);       //always wait for the right time to acquire data
        timer_active = 0;
        temp |= (P2IN & BIT0) << i % 8;
        if (i != 0 && i % 8 == 0) {
            buffer[(int)(i / 8)] = temp;
        }
        i++;
    }

    //Receive CRC
    i = 0;
    while (i < sizeof(crc) * 8) {
        while(timer_active == 0);       //always wait for the right time to acquire data
        timer_active = 0;
        checksum |= (P2IN & BIT0) << i % (sizeof(crc) * 8);
        i++;
    }

    if(!verifyCRC(buffer, checksum)){
        packet_error = 1;
        return;
    }

    //Receive stop bit
    while(timer_active == 0);       //always wait for the right time to acquire data
    if(P2IN & BIT0 != STOP_BIT) {
        packet_error = 1;
        return;
    }*/

    receiving = 0;
}


void retrieveData(char *packet) {
    temp = 0;
    temp += packet[3];
    temp += packet[5] << 1;
    temp += packet[6] << 2;
    temp += packet[7] << 3;
    temp += packet[9] << 4;
    temp += packet[10] << 5;
    temp += packet[11] << 6;
    temp += packet[12] << 7;
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



int verifyCRC(char const message[], crc check)
{
    char data;
    crc remainder = sizeof(crc);


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
    return (remainder == check);

}   /* crcFast() */


void sendToComputer() {

    for (i = 0; i < BUFFER_SIZE; i++) {
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = buffer[i];
    }
}
