/**
 * @file main.c
 * @brief Main file to run all code.
 */

#include <msp430fr2310.h>
#include <stdint.h>
#include <stdbool.h>

#define I2C_ADDR 0x48

uint8_t old_pattern = 0b00000000; // To check against new pattern
uint8_t new_pattern = 0b00000000; // For receiving new pattern

unsigned int time_since_active = 3;

/**
 * Initializes all GPIO ports.
 */
void initGPIO(void)
{
    // Set ports 1.0, 1.1, 1.4-1.7, 2.0, 2.6, 2.7 as outputs
    P1DIR |= BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7;
    P2DIR |= BIT0 | BIT6 | BIT7;

    // Set GPIO outputs to zero
    P1OUT &= ~(BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7);
    P2OUT &= ~(BIT0 | BIT6 | BIT7);

    // I2C pins
    P1SEL0 |= BIT2 | BIT3;
    P1SEL1 &= ~(BIT2 | BIT3);

    // Disable the GPIO power-on default high-impedance mdoe to activate
    // previously configure port settings
    PM5CTL0 &= ~LOCKLPM5;
}

/**
 * Initializes all timers.
 */
void initTimer(void)
{
    // ACLK, continuous mode, clear TBR, divide by 8, length 12-bit
    TB1CTL = TBSSEL__ACLK | MC_2 | TBCLR | ID__8 | CNTL_1;
    TB1CTL &= ~TBIFG; // Clear CCR0 Flag
    TB1CTL |= TBIE; // Enable TB1 Overflow IRQ
}

/**
 * Sets all I2C parameters.
 */
void initI2C(void)
{
    UCB0CTLW0 = UCSWRST; // Software reset enabled
    UCB0CTLW0 |= UCMODE_3 | UCSYNC; // I2C mode, sync mode
    UCB0I2COA0 = I2C_ADDR | UCOAEN; // Own Address and enable
    UCB0CTLW0 &= ~UCSWRST; // clear reset register
    UCB0IE |= UCRXIE; // Enable I2C read interrupt
}

/**
 * Main function.
 *
 * A longer description, with more discussion of the function
 * that might be useful to those using or modifying it.
 */
int main(void)
{
    const uint8_t PATTERNS[] = { { 0b00000000 }, { 0b10000000 }, { 0b00000001 } };

    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    // Initialize ports and other subsystems
    initGPIO();
    initTimer();
    initI2C();

    __enable_interrupt(); // Enable Maskable IRQs

    while (true)
    {
        if (new_pattern != old_pattern)
        {
            // Set LED bar outputs
            P1OUT = (P1OUT & 0b11111100) | (new_pattern & 0b00000011);
            P1OUT = (P1OUT & 0b00001111) | ((new_pattern & 0b00111100) << 2);
            P2OUT = (P2OUT & 0b00111111) | (new_pattern & 0b11000000);

            old_pattern = new_pattern; // Update old pattern
        }
    }
}

/**
 * Timer B1 Overflow Interrupt.
 *
 * Runs every second. Starts flashing status LED
 * 3 seconds after receiving anything over I2C.
 */
#pragma vector = TIMER1_B1_VECTOR
__interrupt void ISR_TB1_OVERFLOW(void)
{
    if (time_since_active >= 3)
    {
        P2OUT ^= BIT0;
    }
    time_since_active++;

    TB1CTL &= ~TBIFG; // Clear CCR0 Flag
}

/**
 * I2C RX Interrupt.
 *
 * Stores value received over I2C in global var "key".
 */
#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void)
{
    new_pattern = UCB0RXBUF;
    P2OUT |= BIT0;
    time_since_active = 0;
}
