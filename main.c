/*
* EE 615 Embedded Systems Lab
* Final Project : Ultrasonic Sensor-based Parking Assist
* Rishabh Pomaje                - 210020036
* Ganesh Panduranga Karamsetty  - 210020009
*/

/*
 * Temporary Notes :
 * R = 0x02
 * B = 0x04
 * G = 0x08
 * Y = 0x0A
 */

#define STCTRL *((volatile long *) 0xE000E010)                                  // Control and Status
#define STRELOAD *((volatile long *) 0xE000E014)                                // Reload value
#define STCURRENT *((volatile long *) 0xE000E018)                               // Current value

#define COUNT_FLAG  (1 << 16)                                                   // CSR[16] = Count Flag
#define ENABLE      (1 << 0)                                                    // CSR[1] = enable the timer
#define CLKINT      (1 << 2)                                                    // CSR[2] = clock source
#define CLOCK_HZ    16000000                                                    // System Timer Clock Frequency
#define CLOCK_MHz   16
#define MAX_RELOAD  16777215                                                    // Systick Timer counter max out value = 2**24 - 1
#define safeDist    75
#define cautionDist 20

#define REAR        0
#define FRONT       1

int state = 0 ;

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"

// Initialisation fncs
void CLK_enable( void ) ;
void PORTA_init( void );
void PORTE_init( void ) ;
void PORTF_init( void ) ;
void delay(float seconds) ;
// Sensor-specific fncs
void trigUS( void ) ;
void readEcho( void ) ;
// Comms fncs
void UART_setup( void ) ;
void UART_Tx( char data );
char UART_Rx( void );
void UART_sendFloat(float value) ;

int main(void)
{
    // Initializations:
    CLK_enable() ;
    PORTE_init() ;
    PORTF_init() ;
    PORTA_init() ;
    UART_setup() ;

    while(1) {
        if (~(GPIO_PORTF_DATA_R) & 0x01){
            state = FRONT ;
        }
        else if (~(GPIO_PORTF_DATA_R) & 0x10){
            state = REAR ;
        }
        trigUS() ;
        delay(0.05) ;                                                           // Sample the distance every 0.05 seconds
    }
}

void CLK_enable( void )
{
    SYSCTL_RCGCUART_R |= (1 << 0) ;
    SYSCTL_RCGCWTIMER_R = 0x01 ;                                                // Provide clock to timer 0
    SYSCTL_RCGCGPIO_R |= 0x00000001;
    SYSCTL_RCGCGPIO_R |= 0x00000020;                                            // PORTF
    SYSCTL_RCGCGPIO_R |= 0x00000010;                                            // Enable clock to PORT_E
}

void UART_Tx( char data )
{
    while((UART0_FR_R & (1 << 3)) != 0){
        ;
    }
    UART0_DR_R = data ;
}

char UART_Rx( void )
{
    if ((UART0_FR_R & 0x40) != 0){
        char rxData = UART0_DR_R ;
        return rxData ;
    }
    else{
        return 0x00 ;
    }
}

void UART_setup( void )
{
    UART0_CTL_R = 0x00 ;                                                        // Disable the UART

    // Calculations for the Baud Rate Divisor
    int UARTSysClk = CLOCK_HZ ;                                                 // Using system clock for UART module
    int clk_div = 16 ;
    int baud_rate = 9600 ;

    float BRD = (1.0 * UARTSysClk) / (clk_div * baud_rate) ;
    int BRDI = BRD ;
    BRD = BRD - BRDI ;
    int BRDF = 64 * BRD + 0.5 ;

    // Configuring the UART
    UART0_IBRD_R = BRDI ;
    UART0_FBRD_R = BRDF ;
    UART0_LCRH_R |= (1 << 6) | (1 << 5) | (1 << 1) ;
    UART0_CC_R = 0x00 ;
    UART0_ECR_R = 0xFF ;
    UART0_CTL_R |= (1 << 9) | (1 << 8) | (1 << 0) ;
}

void delay(float seconds)
{
    // Set up the GPTM for required delay
    WTIMER0_CTL_R = 0x00 ;                                                          // Disable before configuring
    WTIMER0_CFG_R = 0x04 ;                                                          // Select 32-bit individual mode
    WTIMER0_TAMR_R = 0x01 ;                                                         // Timer and mode register
    WTIMER0_TAILR_R = seconds * CLOCK_HZ ;                                          // Interval Load register
    WTIMER0_CTL_R |= 0x01 ;                                                         // Enable the timer
    while((WTIMER0_RIS_R & 0x01) == 0);                                             // Wait for timer to count down
}

void readEcho( void )
{
    /*
    Interrupt Subroutine that gets called when a rising edge is detected on Echo Pin
    NOTE: Echo Pin is connected to PORT E Pins 1, 3.
    */
    GPIO_PORTE_ICR_R = 0x0A ;                           // Clear the interrupt
    // Use Systick to measure the duration of the pulse:
    STRELOAD = MAX_RELOAD ;                             // Set reload value
    STCURRENT = 0 ;                                     // Writing a dummy value to clear the count register and the count flag.
    STCTRL |= (CLKINT | ENABLE);                        // Set internal clock, enable the timer
    float time_us ;
    
    if (state == 0){
        while (GPIO_PORTE_DATA_R & 0x02);                   // Wait until flag is set
        STCTRL = 0 ;                                        // Stop the timer
        time_us = 1.0 * (MAX_RELOAD - STCURRENT) / CLOCK_MHz ; // Time in microseconds
    }
    else{
        while (GPIO_PORTE_DATA_R & 0x08);                   // Wait until flag is set
        STCTRL = 0 ;                                        // Stop the timer
        time_us = 1.0 * (MAX_RELOAD - STCURRENT) / CLOCK_MHz ; // Time in microseconds
    }

    // GPIO_PORTF_DATA = |...|SW1|G|B|R|SW2|
    float estDist = 1.0 * time_us / 58 ;                      // Estimate the distance
    if  (estDist >= safeDist){
        GPIO_PORTF_DATA_R = 0x08 ;                      // Green LED On :: Distance > threshold
    }
    else if (cautionDist <= estDist && estDist < safeDist){
        GPIO_PORTF_DATA_R = 0x0A ;
    }
    else{
        GPIO_PORTF_DATA_R = 0x02 ;                      // Red LED on :: Distance < threshold
    }
    STCURRENT = 0 ;                                     // Writing a dummy value to clear the count register and the count flag.

    // Send the distance via UART
    UART_sendFloat(estDist) ;
}

void trigUS( void )
{
    /*
    Function to send a single active high pulse of duration ~10 us.
    This is pulse is required to Trigger the Ultrasonic sensor.
    */
    float trigPulseDuration_s = 10.0 / 1000000.0 ;      // Duration of 'Trig' Pulse
    if (state == 0){
        GPIO_PORTE_DATA_R |= 0x01 ;                         // Pulse high
        delay(trigPulseDuration_s);                         // Wait for Trig duration
        GPIO_PORTE_DATA_R &= 0xFE ;                         // Pulse Low
    }
    else{
        GPIO_PORTE_DATA_R |= 0x04 ;                         // Pulse high
        delay(trigPulseDuration_s);                         // Wait for Trig duration
        GPIO_PORTE_DATA_R &= 0xFB ;                         // Pulse Low
    }
}

void PORTA_init( void )
{
    GPIO_PORTA_LOCK_R = 0x4C4F434B ;
    GPIO_PORTA_CR_R = 0xF1 ;
    GPIO_PORTA_DEN_R = 0x03 ;
    GPIO_PORTA_DIR_R = 0x02 ;
    GPIO_PORTA_PUR_R = 0x02 ;
    GPIO_PORTA_AFSEL_R = 0x03 ;
    GPIO_PORTA_PCTL_R = 0x11 ;
}

void PORTF_init( void )
{
    /*
    Function to initialize Port F as an I/O block.
    LEDs as Outputs and Switches as Inputs
    */
    // GPIO_PORTF_DATA = |...|SW1|G|B|R|SW2|
    GPIO_PORTF_LOCK_R = 0x4C4F434B ;                    // Unlock commit register
    GPIO_PORTF_CR_R = 0xF1 ;                            // Make PORT-F configurable
    GPIO_PORTF_DEN_R = 0x1F ;                           // Set PORT-F pins as digital pins
    GPIO_PORTF_DIR_R = 0x0E ;                           // Set PORT-F pin directions
    GPIO_PORTF_PUR_R = 0x11 ;                           // Pull-Up-Resistor Register
    GPIO_PORTF_DATA_R = 0x00 ;                          // Clearing previous data
}

void PORTE_init( void )
{
    /*
    Function to initialize Port E for digital I/O.
    PE[0, 2] = Outputs --> Trig
    PE[1, 3] = Inputs  <-- Echo
    */
    GPIO_PORTE_LOCK_R = 0x4C4F434B;                     // Unlock commit register
    GPIO_PORTE_CR_R = 0x01;                             // Make PORT_E0 configurable
    GPIO_PORTE_DEN_R = 0x0F;                            // 1 = digital; 0 = analog
    GPIO_PORTE_DIR_R = 0x05;                            // 1 = output ; 0 = input
    GPIO_PORTE_IS_R = 0x00 ;                            // 1 = level ; 0 = edge
    GPIO_PORTE_IEV_R = 0x0A ;                           // 1 = Rising/ High; 0 = Falling/Low
    GPIO_PORTE_IM_R = 0x0A ;                            // 1 = Send interrupt; 0 = Do not send.
    GPIO_PORTE_ICR_R = 0xFF ;                           // 1 = Clear interrupt.
    NVIC_EN0_R |=  (1 << 4) ;                           // Enable interrupt for GPIO Port E
}

void UART_sendFloat(float value) {
    // Assuming the value has only 2 decimal places
    int intPart = (int)value;                       // Integer part of the float
    int decPart = (int)((value - intPart) * 100);   // Decimal part (multiplied by 100)

    char buffer[10];
    int i = 0 ;
    int j = 0 ;

    // Convert the integer part to string
    if (intPart == 0) {
        buffer[i++] = '0';  // Handle zero case
    } else {
        while (intPart > 0) {
            buffer[i++] = '0' + (intPart % 10);   // Convert last digit to character
            intPart /= 10;                        // Reduce the number
        }

        // Reverse the integer part
        for (j = 0; j < i / 2; j++) {
            char temp = buffer[j];
            buffer[j] = buffer[i - j - 1];
            buffer[i - j - 1] = temp;
        }
    }

    // Add decimal point
    buffer[i++] = '.';

    // Convert the decimal part to string
    buffer[i++] = '0' + (decPart / 10);  // Tens place of the decimal part
    buffer[i++] = '0' + (decPart % 10);  // Ones place of the decimal part

    // Null-terminate the string
    buffer[i++] = ' ';
    buffer[i++] = 'c';
    buffer[i++] = 'm';

    for (i = 0 ; i < 20 ; i++){
        UART_Tx(buffer[i]) ;
        delay(0.001) ;
    }
}
