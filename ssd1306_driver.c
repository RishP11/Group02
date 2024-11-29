#include "ssd1306_driver.h"
#include "tm4c123gh6pm.h"
#include "ascii.h"
#include <string.h>

void I2C_enable( void )
{
    /*
    Function to Enable clocks to GPIO, modules, etc.
    */
    SYSCTL_RCGCI2C_R |= 0x8 ;                               // I2C module 3
    SYSCTL_RCGCGPIO_R |= 0x00008 ;                          // GPIO Port D
}

void PORTD_init( void )
{
    GPIO_PORTD_LOCK_R = 0x4C4F434B ;                        // Unlock commit register
    GPIO_PORTD_CR_R |= 0xFF ;                               // Make PORT-D configurable
    GPIO_PORTD_DEN_R  = 0x03 ;                              // Set PORT-D pins as digital pins
    GPIO_PORTD_AFSEL_R = 0x03 ;                             // Selecting the alternate function
    GPIO_PORTD_PCTL_R = 0x33 ;                              // Selecting the peripheral for the driving AFSEL
    GPIO_PORTD_ODR_R = 0x02 ;                               // Open Drain Register
    GPIO_PORTD_PUR_R = 0x03 ;                               // Pull-Up Register
}

void I2C3_setup( void )
{
    /*
    Function to setup the I2C3 module to interface with the oled
    */
    // Time period register value calculation
    int SCL_HP = 6 ;                  
    int SCL_LP = 4 ;
    int SCL_CLK = 400000 ;
    int TPR = (1.0 * SYSCLK_HZ / (2 * (SCL_HP + SCL_LP) * SCL_CLK)) - 1 ;
    I2C3_MCR_R = 0x10 ;                                     // |GFE|SFE|MFE|.|.|.|LPBK| - Configuration Register
    I2C3_MTPR_R = TPR ;                                     // Time Period Register
    I2C3_MSA_R = 0x78 ;                                     // |.|.|.|.|.|.|.|.|R/W'|
}

void I2C3_Tx( uint8_t controlByte, uint8_t dataByte )
{
    /*
    Function to transmit data frames to the oled
    */
    I2C3_MDR_R = controlByte ;                              // Place the data
    I2C3_MCS_R = 0x03 ;                                     // Start and Run
    while(I2C3_MCS_R & 0x01) ;                              // Wait until busy
    I2C3_MDR_R = dataByte ;                                 // Data Content
    I2C3_MCS_R = 0x05 ;                                     // Run and Stop
    while(I2C3_MCS_R & 0x01) ;                              // Wait until busy
}

void oled_init( void )
{
    /*
    Function to initialize the display
    */
    I2C3_Tx(OLED_COMMAND, OLED_DISPLAY_OFF);                // Display OFF
    I2C3_Tx(OLED_COMMAND, OLED_MEMORY_MODE);                // Set Memory Addressing Mode
    I2C3_Tx(OLED_COMMAND, 0x02);                            // Page Addressing Mode
    I2C3_Tx(OLED_COMMAND, 0xB0);                            // Set Page Start Address
    I2C3_Tx(OLED_COMMAND, OLED_COM_SCAN_DIR);               // COM Output Scan Direction
    I2C3_Tx(OLED_COMMAND, OLED_SET_LOW_COLUMN);             // Set Low Column Address
    I2C3_Tx(OLED_COMMAND, OLED_SET_HIGH_COLUMN);            // Set High Column Address
    I2C3_Tx(OLED_COMMAND, OLED_SET_START_LINE);             // Set Start Line Address
    I2C3_Tx(OLED_COMMAND, OLED_SET_CONTRAST);               // Set Contrast Control Register
    I2C3_Tx(OLED_COMMAND, 0xFF);                            // Set Contrast Value
    I2C3_Tx(OLED_COMMAND, OLED_SEG_REMAP);                  // Set Segment Re-map
    I2C3_Tx(OLED_COMMAND, OLED_NORMAL_DISPLAY);             // Set Normal Display
    I2C3_Tx(OLED_COMMAND, OLED_SET_MULTIPLEX);              // Set Multiplex Ratio
    I2C3_Tx(OLED_COMMAND, OLED_MULTIPLEX_RATIO);            // Multiplex Ratio Value
    I2C3_Tx(OLED_COMMAND, OLED_RAM2DISPLAY);                // Output RAM to Display
    I2C3_Tx(OLED_COMMAND, OLED_SET_OFFSET);                 // Set Display Offset
    I2C3_Tx(OLED_COMMAND, 0x00);                            // Offset Value
    I2C3_Tx(OLED_COMMAND, OLED_SET_CLOCK_DIV);              // Set Display Clock Divide Ratio
    I2C3_Tx(OLED_COMMAND, OLED_CLOCK_RATIO);                // Clock Divide Ratio Value
    I2C3_Tx(OLED_COMMAND, OLED_SET_PRECHARGE);              // Set Pre-charge Period
    I2C3_Tx(OLED_COMMAND, OLED_PRECHARGE_VALUE);            // Pre-charge Value
    I2C3_Tx(OLED_COMMAND, OLED_SET_COM_PINS);               // Set COM Pins Hardware Configuration
    I2C3_Tx(OLED_COMMAND, OLED_COM_PINS_VALUE);             // Hardware Configuration Value
    I2C3_Tx(OLED_COMMAND, OLED_SET_VCOMH);                  // Set VCOMH
    I2C3_Tx(OLED_COMMAND, OLED_VCOMH_VALUE);                // VCOMH Value
    I2C3_Tx(OLED_COMMAND, OLED_CHARGE_PUMP);                // Enable Charge Pump Regulator
    I2C3_Tx(OLED_COMMAND, OLED_CHARGE_PUMP_ENABLE);         // Charge Pump Regulator Value
    I2C3_Tx(OLED_COMMAND, OLED_DISPLAY_ON);                 // Display ON
}

void oledClear( void )
{
    /*
    Function that clears the complete screen
    */
    int i ;
    // Page 0 :
    I2C3_Tx(OLED_COMMAND, 0xB0);      // Set Page Start Address
    for(i = 0; i < 128 ; i++){
        I2C3_Tx(OLED_DATA, 0x00) ;
    }
    // Page 1 :
    I2C3_Tx(OLED_COMMAND, 0xB1);      // Set Page Start Address
    for(i = 0; i < 128 ; i++){
        I2C3_Tx(OLED_DATA, 0x00) ;
    }
    // Page 2 :
    I2C3_Tx(OLED_COMMAND, 0xB2);      // Set Page Start Address
    for(i = 0; i < 128 ; i++){
        I2C3_Tx(OLED_DATA, 0x00) ;
    }
    // Page 3 :
    I2C3_Tx(OLED_COMMAND, 0xB3);      // Set Page Start Address
    for(i = 0; i < 128 ; i++){
        I2C3_Tx(OLED_DATA, 0x00) ;
    }
    // Page 4 :
    I2C3_Tx(OLED_COMMAND, 0xB4);      // Set Page Start Address
    for(i = 0; i < 128 ; i++){
        I2C3_Tx(OLED_DATA, 0x00) ;
    }
    // Page 5 :
    I2C3_Tx(OLED_COMMAND, 0xB5);      // Set Page Start Address
    for(i = 0; i < 128 ; i++){
        I2C3_Tx(OLED_DATA, 0x00) ;
    }
    // Page 6 :
    I2C3_Tx(OLED_COMMAND, 0xB6);      // Set Page Start Address
    for(i = 0; i < 128 ; i++){
        I2C3_Tx(OLED_DATA, 0x00) ;
    }
    // Page 7 :
    I2C3_Tx(OLED_COMMAND, 0xB7);      // Set Page Start Address
    for(i = 0; i < 128 ; i++){
        I2C3_Tx(OLED_DATA, 0x00) ;
    }
}

void oledSendData( void )
{
    /*
    Function to test patterns and oled functionality and check the behavior 
    !!! Do not use unless you know what to expect !!!
    */
    int diff = (int) 'B' - 34 ;
    int i = 0 ;
    int start = (int) 'A' ;
    int num_char = 0 ;
    while(num_char < 20){
        for (i=0; i < 5; i++){
            I2C3_Tx(OLED_DATA, ASCII[start - diff][i]) ;
        }
        I2C3_Tx(OLED_DATA, ASCII[(int) ' ' - diff][i]) ;
        start += 1 ;
        num_char += 1 ;
    }
}

void oledPixel(int x, int y)
{
    /*
    Function to light up an individual pixel :: Under Development
    */
    uint8_t page_id = y / 8 ;
    uint8_t data_byte = y - page_id * 8 ;
    int x_shift = 0 ;
    // I2C3_Tx(OLED_COMMAND, 0xB0 + page_id);      // Set Page Start Address
    while(x_shift < x - 1){
        I2C3_Tx(OLED_DATA, 0x00) ;
        x_shift += 1 ;
    }
    I2C3_Tx(OLED_DATA, data_byte) ;
}

void oledPrintStr(char charArray[])
{
    /*
    Function to print any string on the oled display
    */
    int diff = (int) 'B' - 34 ;
    int inputStrLen = strlen(charArray);

    int num_char = 0 ;
    while(num_char < inputStrLen){
        uint8_t i = 0 ;
        for (i=0; i < 5; i++){
            I2C3_Tx(OLED_DATA, ASCII[(int) charArray[num_char] - diff][i]) ;
        }
        I2C3_Tx(OLED_DATA, ASCII[(int) ' ' - diff][i]) ;
        num_char += 1 ;
    } 
    // Fill blanks on rest of the row
    int blank_col_fills = 128 -  (6 * inputStrLen) ;              // 5+1 = Character width + Space
    while(blank_col_fills > 0){
        I2C3_Tx(OLED_DATA, 0x00) ;                                // Blank data 
        blank_col_fills -= 1 ;
    }
}

void num2str(float num, char *str, int precision) {
    /*
    Function to convert floating pt number to string 
    */
    // Handle negative numbers
    if (num < 0) {
        *str++ = '-';
        num = -num;                                           
    }

    // Get the integer part of the float
    int integerPart = (int) num;
    num -= integerPart;                                      // Remove the integer part, leaving the fractional part

    // Convert the integer part to string
    char temp[50];                                          // Temporary array for the integer part
    int i = 0;
    int j ;
    if (integerPart == 0){
        temp[i++] = '0';                                    // Special case for zero
    } 
    else{
        while (integerPart > 0) {
            temp[i++] = (integerPart % 10) + '0';           // Convert digit to character
            integerPart /= 10;
        }
    }

    // Reverse the integer part string and append to result
    for (j = i - 1; j >= 0; j--) {
        *str++ = temp[j];
    }

    // If precision is greater than 0, add the decimal point and fractional part
    if (precision > 0) {
        *str++ = '.';  
        num *= pow(10, precision);                     // Shift the fractional part
        int fracPart = ( int ) num;                    // Extract integer part of the fractional part

    // Convert the fractional part to string
    for (j = 0; j < precision; j++) {
        *str++ = ( fracPart % 10 ) + '0';              // Get the digit and convert to character
        fracPart /= 10;
    }
    }

    *str = '\0';  // Null-terminate the string
}