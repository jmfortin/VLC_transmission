#include <msp430.h>

// ----------- SELECT BUFFER SIZE ---------------------------
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
void receivePacket();
void verifyParity(char *packet);
void transmissionEnded(char *packet);
void retrieveData(char *packet);
void hammingDecode(char *packet);

//attributes
char temp;
volatile unsigned int i = 0;
char buffer[BUFFER_SIZE];
char packet[14];    //start bit + data bits + parity bits + stop bit(s)
char data[8];
volatile unsigned int error_p1, error_p2, error_p3, error_p4;
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
    P2REN |= BIT0;      //set pull-down resistor
    P2OUT &= ~BIT0;
    P2IE |= BIT0;       //enable interrupts for pin P2.0
    P2IES &= ~BIT0;     //interrupt on rising edge

    receiving = 0;
    timer_active = 0;
    packet_error = 0;

    error_p1 = 0;
    error_p2 = 0;
    error_p3 = 0;
    error_p4 = 0;

    __enable_interrupt();

    while(1) {
        if((P2IN & BIT0) == 1) {
            i = 0;
            receivePacket();

            if (packet_error == 0) {
                retrieveData(packet);
                UCA1TXBUF = temp;
            }
            else {
                __no_operation();
            }
            packet_error = 0;
            receiving = 0;
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
    while (i < sizeof(packet)) {
        while(timer_active == 0);
        timer_active = 0;
        packet[i] = (P2IN & BIT0);
        i++;
    }

    hammingDecode(packet);
    transmissionEnded(packet);
}

void verifyParity(char *packet) {
    int sum = 0;
    for (i = 1; i < 9; i++) {
        sum += packet[i];
    }
    if (((sum % 2) + PARITY) % 2 != packet[9]) {
        packet_error = 1;
    }
}

void transmissionEnded(char *packet) {
    if (packet[sizeof(packet)-1] != 0) {
        packet_error = 1;
    }
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

void hammingDecode(char *packet) {

    //Simpler and more efficient to hardcode
    error_p1 = packet[1] ^ packet[3] ^ packet[5] ^ packet[7] ^ packet[9] ^ packet[11];
    error_p2 = packet[2] ^ packet[3] ^ packet[6] ^ packet[7] ^ packet[10] ^ packet[11];
    error_p3 = packet[4] ^ packet[5] ^ packet[6] ^ packet[7] ^ packet[12];
    error_p4 = packet[8] ^ packet[9] ^ packet[10] ^ packet[11] ^ packet[12];

    unsigned int wrong_bit = error_p1 * 1 + error_p2 * 2 + error_p3 * 4 + error_p4 * 8;

    if (wrong_bit != 0) {
        packet[wrong_bit] ^= 1;
    }
}
