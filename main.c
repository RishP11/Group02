/*
* EE 615 Embedded Systems Lab
* Final Project : Ultrasonic Sensor-based Parking Assist
* Rishabh Pomaje                - 210020036
* Ganesh Panduranga Karamsetty  - 210020009
*/

#define STCTRL *((volatile long *) 0xE000E010)          // Control and Status
#define STRELOAD *((volatile long *) 0xE000E014)        // Reload value
#define STCURRENT *((volatile long *) 0xE000E018)       // Current value

#define COUNT_FLAG  (1 << 16)                           // CSR[16] = Count Flag
#define ENABLE      (1 << 0)                            // CSR[1] = enable the timer
#define CLKINT      (1 << 2)                            // CSR[2] = clock source
#define CLOCK_HZ    16000000                            // System Timer Clock Frequency
#define CLOCK_MHz   16
#define MAX_RELOAD  16777215                            // Systick Timer counter max out value = 2**24 - 1

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

void Delay(float seconds);
void trigUS( void ) ;
void readEcho( void ) ;
void PORTE_init( void ) ;
void PORTF_init( void ) ;
void delay(float seconds) ;

int main(void)
{
    // Initializations:
    PORTE_init() ;
    PORTF_init() ;
    while(1) {
//        trigUS() ;
////        Delay(0.05) ;       // Sample the distance every 0.05 seconds
//        int count = 0 ;
//        while(count < 400000){
//            count += 1 ;
        GPIO_PORTF_DATA_R = 0x05 ;
        delay(0.5) ;
        GPIO_PORTF_DATA_R = 0x02 ;
        delay(0.5) ;
    }
}

void delay(float seconds)
{
    // Set up the GPTM for required delay
    SYSCTL_RCGCWTIMER_R = 0x01 ;                      // Provide clock to timer 0
    WTIMER0_CTL_R = 0x00 ;                           // Disable before configuring
    WTIMER0_CFG_R = 0x04 ;                      // Select 32-bit individual mode
    WTIMER0_TAMR_R = 0x01 ;                         // Timer and mode register
    WTIMER0_TAILR_R = seconds * CLOCK_HZ ;                         // Interval Load register
    WTIMER0_CTL_R |= 0x01 ;                            // Enable the timer
//    WTIMER0_TAMATCHR_R = 0x00 ;
    while((WTIMER0_RIS_R & 0x01) == 0);                  // Wait for timer to count down
    WTIMER0_ICR_R = 0x01 ;
}

void readEcho( void )
{
    /*
    Interrupt Subroutine that gets called when a rising edge is detected on Echo Pin
    NOTE: Echo Pin is connected to PORT E Pins 1, 3.
    */
    GPIO_PORTE_ICR_R = 0x0A ;                               // Clear the interrupt
    // Using Systick to measure the duration of the pulse:
    STRELOAD = MAX_RELOAD ;                              // Set reload value
    STCURRENT = 0 ;                                     // Writing a dummy value to clear the count register and the count flag.
    STCTRL |= (CLKINT | ENABLE);                        // Set internal clock, enable the timer
    while (GPIO_PORTE_DATA_R & 0x02);                   // Wait until flag is set
    STCTRL = 0 ;                                         // Stop the timer
    float time_us = 1.0 * (MAX_RELOAD - STCURRENT) / CLOCK_MHz ; // Time in microseconds

    // GPIO_PORTF_DATA = |...|SW1|G|B|R|SW2|
    float estDist = time_us / 58 ;             // Estimate the distance
    if  (estDist > 10){
        GPIO_PORTF_DATA_R = 0x08 ;                          // Green LED On :: Distance > threshold
    }
    else{
        GPIO_PORTF_DATA_R = 0x02 ;                          // Red LED on :: Distance < threshold
    }
    STCURRENT = 0 ;                                     // Writing a dummy value to clear the count register and the count flag.
}

void trigUS( void )
{
    /*
    Function to send a single active high pulse of duration ~10 us.
    This is pulse is required to Trigger the Ultrasonic sensor.
    */
    float trigPulseDuration_s = 10.0 / 1000000.0 ;          // Duration of 'Trig' Pulse
    GPIO_PORTE_DATA_R |= 0x01 ;                             // Pulse high
    Delay(trigPulseDuration_s);                             // Wait for Trig duration
    GPIO_PORTE_DATA_R &= 0xFE ;                             //
}

void Delay(float seconds)
{
    /*
    A simple Systick-Timer based delay function. Maximum delay this function can generate is
    1.04 secs. Input the required delay in seconds.
    */
    unsigned long int count_top = CLOCK_HZ * seconds ;
    STRELOAD = count_top ;                              // Set reload value
    STCURRENT = 0 ;                                     // Write dummy value to clear the count register and count flag
    STCTRL |= (CLKINT | ENABLE);                        // Set internal clock, enable the timer
    while ((STCTRL & COUNT_FLAG) == 0) {                // Wait until flag is set
        ;
    }
    STCTRL = 0;                                         // Stop the timer
}

void PORTF_init( void )
{
    /*
    Function to initialize Port F as an I/O block.
    LEDs as Outputs and Switches as Inputs
    */
    // GPIO_PORTF_DATA = |...|SW1|G|B|R|SW2|
    SYSCTL_RCGCGPIO_R |= 0x00000020;
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
    SYSCTL_RCGCGPIO_R |= 0x00000010;                    // Enable clock to PORT_E
    GPIO_PORTE_LOCK_R = 0x4C4F434B;                     // Unlock commit register
    GPIO_PORTE_CR_R = 0x01;                             // Make PORT_E0 configurable
    GPIO_PORTE_DEN_R = 0x0F;                            // 1 = digital; 0 = analog
    GPIO_PORTE_DIR_R = 0x05;                            // 1 = output ; 0 = input
    GPIO_PORTE_IS_R = 0x00 ;                            // 1 = level ; 0 = edge
    GPIO_PORTE_IEV_R = 0x0A ;                           // 1 = Rising/ High; 0 = Falling/Low
    GPIO_PORTE_IM_R = 0x0A ;                            // 1 = Send interrupt; 0 = Do not send.
    GPIO_PORTE_ICR_R = 0x0A ;                           // 1 = Clear interrupt.
    NVIC_EN0_R |=  (1 << 4) ;                           // Enable interrupt for GPIO Port E
}
