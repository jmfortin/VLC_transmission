#include <msp430.h>

// ----------- SELECT BUFFER SIZE --------------------------
#define BUFFER_SIZE    32          // (bytes)
// ----------------------------------------------------------
// ----------- SELECT START/STOP BITS -----------------------
#define START_BIT      1
#define STOP_BIT       0
// ----------------------------------------------------------
// ----------- PARITY BIT -----------------------------------
#define PARITY         0           // 0 = even, 1 = odd
// ----------------------------------------------------------


//functions
void acquireData();
void createPacket(char data);
int calculateParity(char *packet);
void sendPacket();
void waitForConfirmation();
void hammingEncode(char data);

//attributes
char temp;
volatile unsigned int i = 0;
char buffer[BUFFER_SIZE];
char packet[14];    //start bit + data bits + parity + stop bit(s)
volatile unsigned int p1, p2, p3, p4;
volatile unsigned int data_received, timer_active;

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


    P2DIR |= BIT0;
    P2OUT |= BIT0;

    timer_active = 0;
    data_received = 0;


    while(1) {

        __bis_SR_register(LPM0_bits + GIE);       // CPU off, enable interrupts
        __no_operation();                         // For debugger

        createPacket(UCA1RXBUF);
        sendPacket(packet);
        waitForConfirmation();

        data_received = 0;
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

    if (data_received == 1) {
        timer_active = 1;
        __bic_SR_register_on_exit(LPM0_bits);
    }
}

void acquireData() {
    //Data will eventually come from the sensor
}

void createPacket(char data) {

    hammingEncode(data);

    packet[0] = START_BIT;
    packet[1] = p1;
    packet[2] = p2;
    packet[3] = data & 1;
    packet[4] = p3;
    packet[5] = (data >> 1) & 1;
    packet[6] = (data >> 2) & 1;
    packet[7] = (data >> 3) & 1;
    packet[8] = p4;
    packet[9] = (data >> 4) & 1;
    packet[10] = (data >> 5) & 1;
    packet[11] = (data >> 6) & 1;
    packet[12] = (data >> 7) & 1;
    packet[13] = STOP_BIT;
}

int calculateParity(char *packet) {

    int sum = 0;
    for (i = sizeof(packet)-2; i > 0; i--) {
        sum += packet[i-1];
    }
    if (sum % 2 == 0) {
        return PARITY;
    }
    else {
        return PARITY ^ 1;
    }
}

void sendPacket() {

    for (i = 0; i < sizeof(packet); i++) {
        while(timer_active == 0);       //wait for timer
        timer_active = 0;
        if (packet[i] == 1) {
            P2OUT &= ~(0xFE | packet[i]);
        }
        else {
            P2OUT |= ~(0xFE | packet[i]);
        }
    }
}

void waitForConfirmation() {

}

void hammingEncode(char data) {

    //calculate parity bits
    p1 = (data & 1) ^ ((data >> 1) & 1) ^ ((data >> 3) & 1)
            ^ ((data >> 4) & 1) ^ ((data >> 6) & 1);
    p2 = (data & 1) ^ ((data >> 2) & 1) ^ ((data >> 3) & 1)
            ^ ((data >> 5) & 1) ^ ((data >> 6) & 1);
    p3 = ((data >> 1) & 1) ^ ((data >> 2) & 1)
            ^ ((data >> 3) & 1) ^ ((data >> 7) & 1);
    p4 = ((data >> 4) & 1) ^ ((data >> 5) & 1)
            ^ ((data >> 6) & 1) ^ ((data >> 7) & 1);
}

