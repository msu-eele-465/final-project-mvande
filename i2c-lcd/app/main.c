/**
 * @file main.c
 * @brief Main file to run all code.
 */

#include <msp430fr2310.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "src/lcd_driver.h"

#define I2C_ADDR 0x49

#define RS_HIGH P2OUT |= BIT6
#define RS_LOW P2OUT &= ~BIT6
#define E_HIGH P2OUT |= BIT7
#define E_LOW P2OUT &= ~BIT7

#define LCD_DATA P1OUT
#define DB4 BIT4
#define DB5 BIT5
#define DB6 BIT6
#define DB7 BIT7

#define CLEAR_LCD 0x01

char key = '\0';

unsigned int time_since_active = 3;

uint8_t buffer[3] = { 0 };
unsigned int index;

char new_x_coord[] = "000";
char new_y_coord[] = "000";
char new_z_coord[] = "000";

char old_x_coord[] = "000";
char old_y_coord[] = "000";
char old_z_coord[] = "000";

/**
 * Initializes all GPIO ports.
 */
void init_gpio(void)
{
    // Set ports 1.4-1.7, 2.0, 2.6, 2.7 as outputs
    P1DIR |= BIT4 | BIT5 | BIT6 | BIT7;
    P2DIR |= BIT0 | BIT6 | BIT7;

    // Set GPIO outputs to zero
    P1OUT &= ~(BIT4 | BIT5 | BIT6 | BIT7);
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
void init_timer(void)
{
    TB1CTL = TBSSEL__ACLK | MC_2 | TBCLR | ID__8 | CNTL_1; // ACLK, continuous mode, clear TBR, divide by 8, length
                                                           // 12-bit
    TB1CTL &= ~TBIFG; // Clear CCR0 Flag
    TB1CTL |= TBIE; // Enable TB1 Overflow IRQ
}

/**
 * Sets all I2C parameters.
 */
void init_i2c(void)
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
 * Starts by initializing subsystems and ports. Handles
 * I2C data after being received and outputs to LCD display.
 */
int main(void)
{
    // uint8_t cursor = 0b00001100;
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    // Initialize ports and other subsystems
    init_gpio();
    init_timer();
    init_i2c();

    init_lcd();
    send_cmd(CLEAR_LCD);
    __delay_cycles(10000); // Wait ~1.6 ms

    // Set initial display
    send_string("X=");
    send_string(old_x_coord);

    send_cmd(0x88); // Send cursor to halfway of top row
    send_string("Y=");
    send_string(old_y_coord);

    send_cmd(0xC0); // Send cursor to start of bottom row
    send_string("Z=");
    send_string(old_z_coord);

    __enable_interrupt(); // Enable Maskable IRQs

    while (true)
    {
        if (strcmp(new_x_coord, old_x_coord))
        {
            send_cmd(0x80); // Send cursor to start of top row
            send_string("X=");
            send_string(new_x_coord);
            strcpy(old_x_coord, new_x_coord);
        }
        if (strcmp(new_y_coord, old_y_coord))
        {
            send_cmd(0x88); // Send cursor to halfway of top row
            send_string("Y=");
            send_string(new_y_coord);
            strcpy(old_y_coord, new_y_coord);
        }
        if (strcmp(new_z_coord, old_z_coord))
        {
            send_cmd(0xC0); // Send cursor to start of bottom row
            send_string("Z=");
            send_string(new_z_coord);
            strcpy(old_z_coord, new_z_coord);
        }
    }
}

/**
 * Timer B1 Overflow Interrupt.
 *
 * Runs every second. Starts flashing status LED
 * 3 seconds after receiving something over I2C.
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
uint8_t data_in;
#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void)
{
    data_in = UCB0RXBUF;
    if (data_in > 0x39)
    {
        key = (char)data_in;
    }
    else 
    {
        buffer[index++] = data_in;
    }

    if(index >= 3 && key > 0x39)
    {
        if (key == 'X')
        {
            new_x_coord[0] = (char)buffer[0];
            new_x_coord[1] = (char)buffer[1];
            new_x_coord[2] = (char)buffer[2];
        }
        else if (key == 'Y')
        {
            new_y_coord[0] = (char)buffer[0];
            new_y_coord[1] = (char)buffer[1];
            new_y_coord[2] = (char)buffer[2];
        }
        else if (key == 'Z')
        {
            new_z_coord[0] = (char)buffer[0];
            new_z_coord[1] = (char)buffer[1];
            new_z_coord[2] = (char)buffer[2];
        }
        key = '\0';
        index = 0;
        buffer[0] = 0;
        buffer[1] = 0;
        buffer[2] = 0;
    }
    P2OUT |= BIT0;
    time_since_active = 0;
}
