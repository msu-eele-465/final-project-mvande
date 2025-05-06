/**
 * @file main.c
 * @brief Main file to run all code.
 */

#include "msp430fr2355.h"
#include <msp430.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// I2C control pins
#define SDA_PIN BIT2
#define SCL_PIN BIT3

// I2C Addresses
#define LED_PERIPHERAL_ADDR 0x48
#define LCD_PERIPHERAL_ADDR 0x49
#define RTC_PERIPHERAL_ADDR 0x68

// I2C TX and RX Buffers
volatile uint8_t i2c_tx_data[4];
volatile unsigned int i2c_tx_index = 0;
volatile uint8_t i2c_rx_data[2];
volatile unsigned int i2c_rx_index = 0;

void i2c_init(void);
void i2c_write_byte(uint8_t addr, uint8_t byte);
void i2c_write_string(uint8_t addr, const uint8_t data[], unsigned int data_len);
void i2c_read(uint8_t addr, uint8_t reg, unsigned int bytes);
void send_to_lcd(uint8_t byte);
void send_to_led(uint8_t pattern);

void gpio_init(void);

void timer_init(void);

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog

    i2c_init(); // Initialize I2C with int
    gpio_init(); // Initialize Keypad
    timer_init(); // Initialize any timers

    __enable_interrupt(); // Enable global interrupts

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configure port settings
    PM5CTL0 &= ~LOCKLPM5;

    while (1)
    {
        
    }
}

void timer_init(void) {
    TB1CTL = TBSSEL__ACLK | MC_2 | TBCLR | ID__8 | CNTL_1; // ACLK, continuous mode, clear TBR, divide by 8, length
                                                           // 12-bit
    TB1CTL &= ~TBIFG; // Clear CCR0 Flag
    TB1CTL |= TBIE; // Enable TB1 Overflow IRQ
}

void i2c_init(void)
{
    P1SEL1 &= ~(SDA_PIN | SCL_PIN);
    P1SEL0 |= SDA_PIN | SCL_PIN; // Set SDA and SCL pins

    UCB0CTLW0 = UCSWRST; // Put eUSCI_B0 in reset mode

    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSSEL_3 | UCTR; // I2C master mode
    UCB0BRW = 10; // Set I2C clock prescaler
    UCB0CTLW1 |= UCASTP_2;      // Auto STOP when UCB0TBCNT reached
    UCB0TBCNT = 0x01;

    UCB0CTLW0 &= ~UCSWRST; // Release eUSCI_B0 from reset

    UCB0IE |= UCTXIE0 | UCRXIE0; // Enable TX and RX interrupts
}

void gpio_init(void)
{
    
}

void send_to_lcd(uint8_t byte)
{
    i2c_write_byte(LCD_PERIPHERAL_ADDR, byte); // Use interrupt-based I2C
}

void send_to_led(uint8_t pattern)
{
    i2c_write_byte(LED_PERIPHERAL_ADDR, pattern);
}

void i2c_write_byte(uint8_t addr, uint8_t byte)
{
    i2c_tx_data[0] = byte;
    i2c_tx_index = 0;

    UCB0I2CSA = addr; // Set I2C slave address
    UCB0TBCNT = 0x01;
    UCB0CTLW0 |= UCTR | UCTXSTT; // Set to transmit mode and send start condition

    while((UCB0IFG & UCSTPIFG) == 0){}  // Wait for STOP
    UCB0IFG &= ~UCSTPIFG;
}

void i2c_write_string(uint8_t addr, const uint8_t data[], unsigned int data_len)
{
    if (data_len > 4)
    {
        data_len = 4;
    }

    unsigned int i;
    for (i = 0; i < data_len; i++)
    {
        i2c_tx_data[i] = data[i];
    }
    i2c_tx_index = 0;

    UCB0I2CSA = addr; // Set I2C slave address
    UCB0TBCNT = data_len;
    UCB0CTLW0 |= UCTR | UCTXSTT; // Set to transmit mode and send start condition

    while((UCB0IFG & UCSTPIFG) == 0){}  // Wait for STOP
    UCB0IFG &= ~UCSTPIFG;
}

void i2c_read(uint8_t addr, uint8_t reg, unsigned int bytes)
{
    i2c_rx_index = 0;
    i2c_write_byte(addr, reg);

    UCB0CTLW0 &= ~UCTR;
    UCB0TBCNT = bytes;
    UCB0CTLW0 |= UCTXSTT; // Set to read mode and send start condition

    while((UCB0IFG & UCSTPIFG) == 0){}  // Wait for STOP
    UCB0IFG &= ~UCSTPIFG;
}

#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void)
{
    switch (__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
    {
        case USCI_NONE:
            break;
        case USCI_I2C_UCTXIFG0:
            UCB0TXBUF = i2c_tx_data[i2c_tx_index++]; // Load TX buffer
            break;

        case USCI_I2C_UCRXIFG0:
            i2c_rx_data[i2c_rx_index++] = UCB0RXBUF; // Read received byte
            break;

        default:
            break;
    }
}

#pragma vector = TIMER1_B1_VECTOR
__interrupt void ISR_TB1_OVERFLOW(void)
{
    TB1CTL &= ~TBIFG;
}
