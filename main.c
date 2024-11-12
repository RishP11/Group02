#define STCTRL *((volatile long *) 0xE000E010)          // control and status
#define STRELOAD *((volatile long *) 0xE000E014)        // reload value
#define STCURRENT *((volatile long *) 0xE000E018)       // current value

#define COUNT_FLAG  (1 << 16)                           // bit 16 of CSR automatically set to 1
#define ENABLE      (1 << 0)                            // bit 0 of CSR to enable the timer
#define CLKINT      (1 << 2)                            // bit 2 of CSR to specify CPU clock
#define CLOCK_HZ    16000000                            // Timer clock frequency
#define MAX_RELOAD  16777215                            // Maximum value that can be put into the Systick Timer counter

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

void Delay(float seconds);
void trigUS( void ) ;
void readEcho( void ) ;
void PORTE_init( void ) ;
void PORTF_init( void ) ;

int main(void)
 {
    // Initializations:
    PORTE_init() ;
    PORTF_init() ;
    while(1) {
        trigUS() ;
        Delay(0.05) ;
    }
}

void readEcho( void )
{
    GPIO_PORTE_ICR_R = 0x0A ;
//    STRELOAD = MAX_RELOAD ;
//    GPIO_PORTF_DATA_R |= (GPIO_PORTE_DATA_R & 0x02) << 1 ;
//    STCTRL |= (CLKINT | ENABLE);
    // Now that the echo pin has gotten high, we now start a timer to get duration of the echo pulse.
//    uint32_t start = MAX_RELOAD ;
    int count = 0 ;
    while(GPIO_PORTE_DATA_R & 0x02){
        count += 1;
    }
//    STCTRL = 0 ;
//    uint32_t stop = STCURRENT ;
//    STCURRENT = 0 ;
    float Distance = (1.0 * count) / (16 * 58);
        if  (Distance > 10){
            GPIO_PORTF_DATA_R = 0x08 ;
        }
        else{
            GPIO_PORTF_DATA_R = 0x02 ;
        }
}

void trigUS( void )
{
    float trigPulseDuration_s = 10.0 / 1000000.0 ;                          // Pulse duration of the trigger in microseconds.
    //Pulse high
    GPIO_PORTE_DATA_R |= 0x01 ;
    // timer for 10 us
    Delay(trigPulseDuration_s);
//    Delay(0.001) ;
    GPIO_PORTE_DATA_R &= 0xFE ;
}

void Delay(float seconds)
{
    unsigned long int count_top = CLOCK_HZ * seconds ;
    STRELOAD = count_top ;                                 // Set reload value
    STCTRL |= (CLKINT | ENABLE);                        // Set internal clock, enable the timer
        while ((STCTRL & COUNT_FLAG) == 0) {            // Wait until flag is set
            STRELOAD = 0;// do nothing
        }
        // Stop the timer
    STCTRL = 0;
}

void PORTF_init( void )
{
    SYSCTL_RCGCGPIO_R |= 0x00000020;
    GPIO_PORTF_LOCK_R = 0x4C4F434B ;                            // Unlock commit register
    GPIO_PORTF_CR_R = 0xF1 ;                                    // Make PORT-F configurable
    GPIO_PORTF_DEN_R = 0x1F ;                                   // Set PORT-F pins as digital pins
    GPIO_PORTF_DIR_R = 0x0E ;                                   // Set PORT-F pin directions
    GPIO_PORTF_PUR_R = 0x11 ;                                   // Pull-Up-Resistor Register
    GPIO_PORTF_DATA_R = 0x00 ;                                  // Clearing previous data
}

void PORTE_init( void )
{
    // Configure port E for US txns
    SYSCTL_RCGCGPIO_R |= 0x00000010;      // Enable clock to PORT_E
    GPIO_PORTE_LOCK_R = 0x4C4F434B;    // Unlock commit register
    GPIO_PORTE_CR_R = 0x01;            // Make PORT_E0 configurable
    GPIO_PORTE_DEN_R = 0x0F;           // Set PE[0-3] pin as digital
    GPIO_PORTE_DIR_R = 0x05;           // Set PE[0, 2] pin as output, PE[1, 3].
    GPIO_PORTE_IS_R = 0x00 ;
    GPIO_PORTE_IEV_R = 0x0A ;
    GPIO_PORTE_IM_R = 0x0A ;
    GPIO_PORTE_ICR_R = 0x0A ;
    NVIC_EN0_R |=  (1 << 4) ; // Enable interrupt for GPIO Port E
}
