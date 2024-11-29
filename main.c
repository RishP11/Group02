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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "ssd1306_driver.h"

#define STCTRL *((volatile long *) 0xE000E010)                     // Control and Status
#define STRELOAD *((volatile long *) 0xE000E014)                   // Reload value
#define STCURRENT *((volatile long *) 0xE000E018)                  // Current value

#define COUNT_FLAG  (1 << 16)                                      // CSR[16] = Count Flag
#define ENABLE      (1 << 0)                                       // CSR[1] = enable the timer
#define CLKINT      (1 << 2)                                       // CSR[2] = clock source
#define SYSCLK_HZ   16000000                                       // System Timer Clock Frequency
#define CLOCK_MHz   16
#define MAX_RELOAD  16777215                                       // Systick Timer counter max out value = 2**24 - 1
#define safeDist    75
#define cautionDist 20

#define REAR        0
#define FRONT       1

int state = 0 ;

// Initialisation functions
void CLK_enable( void ) ;
void PORTA_init( void ) ; 
void PORTE_init( void ) ;
void PORTF_init( void ) ;
void delay( float seconds ) ;

// Sensor-specific functions
void trigUS( void ) ;
void readEcho( void ) ;

// UART functions
void UART_setup( void ) ;
void UART_Tx( char data );
char UART_Rx( void );
void UART_sendFloat( float value ) ;

int main( void )
{
    // Initializations:
    CLK_enable() ;
    I2C_enable() ;
    I2C3_setup() ;
    PORTD_init() ;
    PORTE_init() ;
    PORTF_init() ;
    PORTA_init() ;
    UART_setup() ;

    oled_init() ;
    oledClear() ;

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
    /*
    Function to enable system clock to all the GPIO_Ports/ modules, etc.
    */
    SYSCTL_RCGCUART_R |= (1 << 0) ;                     // UART Module 0
    SYSCTL_RCGCWTIMER_R = 0x01 ;                        // Wide timer 0
    SYSCTL_RCGCGPIO_R |= 0x00000001;                    // PORT A
    SYSCTL_RCGCGPIO_R |= 0x00000020;                    // PORT F
    SYSCTL_RCGCGPIO_R |= 0x00000010;                    // PORT E
}

void UART_setup( void )
{
    /*
    Function to setup UART0 to communicate with the PC for serial monitor
    */
    UART0_CTL_R = 0x00 ;                                // Disable the UART

    // Calculations for the Baud Rate Divisor
    int UARTSysClk = SYSCLK_HZ ;                        // Using system clock for UART module
    int clk_div = 16 ;
    int baud_rate = 9600 ;

    float BRD = (1.0 * UARTSysClk) / (clk_div * baud_rate) ;
    int BRDI = (int) BRD ;                              // Integer part of BRD
    BRD = BRD - BRDI ;
    int BRDF = 64 * BRD + 0.5 ;                         // Fractional part of BRD

    // Configuring the UART
    UART0_IBRD_R = BRDI ;
    UART0_FBRD_R = BRDF ;
    UART0_LCRH_R |= (1 << 6) | (1 << 5) | (1 << 1) ;    // ..|SPS|..WLEN..|FEN|STP2|EPS|PEN|BRK|
    UART0_CC_R = 0x00 ;                                 // Clock Configuration Register
    UART0_ECR_R = 0xFF ;                                // Error Clear Register
    UART0_CTL_R |= (1 << 9) | (1 << 8) | (1 << 0) ;     // Control Register :: Enable the module for Tx/Rx
}


void UART_Tx( char data )
{
    /*
    Function to transmit a byte of data using UART 
    */
    while((UART0_FR_R & (1 << 3)) != 0){
        ;                                               // Polling the flag register to check if the module is busy
    }
    UART0_DR_R = data ;                                 // Place the data in the data register
}

char UART_Rx( void )
{
    /*
    Function to receive a byte of data using UART
    */
    if ((UART0_FR_R & 0x40) != 0){
        char rxData = UART0_DR_R ;                      // Receive FIFO not empty => Read the RX FIFO/ BUFFER
        return rxData ;
    }
    else{
        return 0x00 ;                               
    }
    }

    void UART_sendFloat(float value) {
    int intPart = ( int ) value ;                       // Integer part of the float
    int decPart = ( int )( (value - intPart) * 100) ;   // Decimal part (multiplied by 100)

    char buffer[10];
    int i = 0 ;
    int j = 0 ;

    // Convert the integer part to string
    if (intPart == 0) {
        buffer[i++] = '0'; 
    } else {
        while (intPart > 0) {
            buffer[i++] = '0' + (intPart % 10);        // Convert last digit to character
            intPart /= 10;                             // Reduce the number
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
    buffer[i++] = '0' + (decPart / 10);             // Tens place of the decimal part
    buffer[i++] = '0' + (decPart % 10);             // Ones place of the decimal part

    // Null-terminate the string
    buffer[i++] = ' ';
    buffer[i++] = 'c';
    buffer[i++] = 'm';

    for (i = 0 ; i < 20 ; i++){
        UART_Tx(buffer[i]) ;
        delay(0.001) ;
    }
}

void trigUS( void )
{
    /*
    Function to send a single active high pulse of duration ~10 us.
    This is pulse is required to Trigger the Ultrasonic sensor.
    */
    float trigPulseDuration_s = 10.0 / 1000000.0 ;          // Duration of 'Trig' Pulse
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

void readEcho( void )
{
    /*
    Interrupt Subroutine that gets called when a rising edge is detected on Echo Pin
    NOTE: Echo Pin is connected to PORT E Pins 1, 3.
    */
    GPIO_PORTE_ICR_R = 0x0A ;                                   // Clear the interrupt
    // Use Systick to measure the duration of the pulse:
    STRELOAD = MAX_RELOAD ;                                     // Set reload value
    STCURRENT = 0 ;                                             // Writing a dummy value to clear the count register and the count flag.
    STCTRL |= (CLKINT | ENABLE);                                // Set internal clock, enable the timer

    float time_us ;
    if (state == 0){
        while (GPIO_PORTE_DATA_R & 0x02);                       // Wait until flag is set
        STCTRL = 0 ;                                            // Stop the timer
        time_us = 1.0 * (MAX_RELOAD - STCURRENT) / CLOCK_MHz ;  // Time in microseconds
    }
    else{
        while (GPIO_PORTE_DATA_R & 0x08);                       // Wait until flag is set
        STCTRL = 0 ;                                            // Stop the timer
        time_us = 1.0 * (MAX_RELOAD - STCURRENT) / CLOCK_MHz ;  // Time in microseconds
    }

    // GPIO_PORTF_DATA = |...|SW1|G|B|R|SW2|
    float estDist = 1.0 * time_us / 58 ;                        // Estimate the distance
    if  (estDist >= safeDist){
        GPIO_PORTF_DATA_R = 0x08 ;                              // Green LED On :: Safe distance
    }
    else if (cautionDist <= estDist && estDist < safeDist){
        GPIO_PORTF_DATA_R = 0x0A ;                              // Yellow LED On ::  Caution distance
    }
    else{
        GPIO_PORTF_DATA_R = 0x02 ;                              // Red LED on :: Dangerous distance
    }
    STCURRENT = 0 ;                                             // Writing a dummy value to clear the count register and the count flag.

    // Send the distance via I2C to OLED
    I2C3_Tx(OLED_COMMAND, 0xB6) ;
    if (state)
        oledPrintStr("Front :") ;
    else{
        oledPrintStr("Rear : ") ;
    }
    I2C3_Tx(OLED_COMMAND, 0xB7) ;

    // Send the distance via UART to serial monitor (PC)
    UART_sendFloat(estDist) ;
    char dist[10] ;
    num2str(estDist, dist, 2) ;
    oledPrintStr(dist) ;
}

void PORTA_init( void )
{
    /*
    Function to initialize Port A for UART0 communications to PC
    */
    GPIO_PORTA_LOCK_R = 0x4C4F434B ;                        // Unlock commit register
    GPIO_PORTA_CR_R = 0xF1 ;                                // Uncommit port A
    GPIO_PORTA_DEN_R = 0x03 ;                               // Digital Enable 
    GPIO_PORTA_DIR_R = 0x02 ;                               // Pin Direction 0 = input, 1 = output
    GPIO_PORTA_PUR_R = 0x02 ;                               // Pull-ups
    GPIO_PORTA_AFSEL_R = 0x03 ;                             // Alternate Function Select
    GPIO_PORTA_PCTL_R = 0x11 ;                              // Port Control :: Select driver peripheral for AFSEL pins
}

void PORTF_init( void )
{
    /*
    Function to initialize Port F as an I/O block.
    LEDs as Outputs and Switches as Inputs
    */
    // GPIO_PORTF_DATA = |...|SW1|G|B|R|SW2|
    GPIO_PORTF_LOCK_R = 0x4C4F434B ;                        // Unlock commit register
    GPIO_PORTF_CR_R = 0xF1 ;                                // Make PORT-F configurable
    GPIO_PORTF_DEN_R = 0x1F ;                               // Set PORT-F pins as digital pins
    GPIO_PORTF_DIR_R = 0x0E ;                               // Set PORT-F pin directions
    GPIO_PORTF_PUR_R = 0x11 ;                               // Pull-Up-Resistor Register
    GPIO_PORTF_DATA_R = 0x00 ;                              // Clearing previous data
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

void delay(float seconds)
{
    /*
    Function to generate a delay using the wide 32/64-bit GPTM Module
    */
    WTIMER0_CTL_R = 0x00 ;                              // Disable before configuring
    WTIMER0_CFG_R = 0x04 ;                              // Select 32-bit individual mode
    WTIMER0_TAMR_R = 0x01 ;                             // Timer and mode register :: Single shot mode
    WTIMER0_TAILR_R = seconds * SYSCLK_HZ ;             // Interval Load register  :: Pulse Duration
    WTIMER0_CTL_R |= 0x01 ;                             // Enable the timer
    while((WTIMER0_RIS_R & 0x01) == 0);                 // Wait for timer to count down
}
