#ifndef SSD1306_DRIVER_H
#define SSD1306_DRIVER_H

#include <stdint.h>

// Tiva System Clock
#define SYSCLK_HZ               16000000

// Oled SSD1306 definitions
#define OLED_DISPLAY_OFF        0xAE
#define OLED_DISPLAY_ON         0xAF

#define OLED_SET_CONTRAST       0x81

#define OLED_NORMAL_DISPLAY     0xA6
#define OLED_INVERTED_DISPLAY   0xA7

#define OLED_MEMORY_MODE        0x20
#define OLED_PAGE_ADDR_MODE     0x02
#define OLED_PAGE_START         0xB3
#define OLED_SET_LOW_COLUMN     0x00
#define OLED_SET_HIGH_COLUMN    0x10
#define OLED_SET_START_LINE     0x40

#define OLED_SEG_REMAP          0xA1
#define OLED_SET_MULTIPLEX      0xA8
#define OLED_MULTIPLEX_RATIO    0x3F
#define OLED_COM_SCAN_DIR       0xC8

#define OLED_RAM2DISPLAY        0xA4
#define OLED_OnlyDISPLAY        0xA5

#define OLED_SET_OFFSET         0xD3
#define OLED_OFFSET_VALUE       0x00

#define OLED_SET_CLOCK_DIV      0xD5
#define OLED_CLOCK_RATIO        0xF0
#define OLED_SET_PRECHARGE      0xD9
#define OLED_PRECHARGE_VALUE    0x22
#define OLED_SET_COM_PINS       0xDA
#define OLED_COM_PINS_VALUE     0x12
#define OLED_SET_VCOMH          0xDB
#define OLED_VCOMH_VALUE        0x20
#define OLED_CHARGE_PUMP        0x8D
#define OLED_CHARGE_PUMP_ENABLE 0x14

#define OLED_DATA               0x40
#define OLED_COMMAND            0x00
#define OLED_WIDTH              128
#define OLED_HEIGHT             64

void I2C_enable( void );
void PORTD_init( void );

void I2C3_setup( void );
void I2C3_Tx( uint8_t controlByte, uint8_t dataByte ) ;

void oled_init( void );
void oledSendData( void );
void oledPixel(int x, int y) ;
void oledPrintStr(char charArray[]) ;
void num2str(float num, char *str, int precision) ;
void oledClear( void );

#endif
