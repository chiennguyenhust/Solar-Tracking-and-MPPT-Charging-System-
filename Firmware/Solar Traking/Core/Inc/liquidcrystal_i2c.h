#ifndef LIQUIDCRYSTAL_I2C_H_
#define LIQUIDCRYSTAL_I2C_H_

#include "stm32f1xx_hal.h"

/* Command */
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

/* Entry Mode */
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

/* Display On/Off */
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

/* Cursor Shift */
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

/* Function Set */
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

/* Backlight */
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

/* Enable Bit */
#define ENABLE 0x04

/* Read Write Bit */
#define RW 0x0

/* Register Select Bit */
#define RS 0x01

/* Device I2C Address */
#define DEVICE_ADDR     (0x27 << 1)

/**
 * @brief Khởi tạo màn hình LCD 1602.
 *
 * @param rows Số dòng của màn hình LCD (1 hoặc 2).
 */
void HD44780_Init(uint8_t rows);

/**
 * @brief Xóa nội dung trên màn hình LCD.
 */
void HD44780_Clear();

/**
 * @brief Di chuyển con trỏ về vị trí ban đầu (góc trên bên trái) trên màn hình LCD.
 */
void HD44780_Home();

/**
 * @brief Tắt hiển thị trên màn hình LCD.
 */
void HD44780_NoDisplay();

/**
 * @brief Bật hiển thị trên màn hình LCD.
 */
void HD44780_Display();

/**
 * @brief Tắt chớp nháy của con trỏ trên màn hình LCD.
 */
void HD44780_NoBlink();

/**
 * @brief Bật chớp nháy của con trỏ trên màn hình LCD.
 */
void HD44780_Blink();

/**
 * @brief Tắt hiển thị con trỏ trên màn hình LCD.
 */
void HD44780_NoCursor();

/**
 * @brief Bật hiển thị con trỏ trên màn hình LCD.
 */
void HD44780_Cursor();

/**
 * @brief Di chuyển nội dung trên màn hình LCD sang trái.
 */
void HD44780_ScrollDisplayLeft();

/**
 * @brief Di chuyển nội dung trên màn hình LCD sang phải.
 */
void HD44780_ScrollDisplayRight();

/**
 * @brief In các ký tự từ phải sang trái trên màn hình LCD.
 */
void HD44780_PrintLeft();

/**
 * @brief In các ký tự từ trái sang phải trên màn hình LCD.
 */
void HD44780_PrintRight();

/**
 * @brief Thiết lập việc in các ký tự từ trái sang phải trên màn hình LCD.
 */
void HD44780_LeftToRight();

/**
 * @brief Thiết lập việc in các ký tự từ phải sang trái trên màn hình LCD.
 */
void HD44780_RightToLeft();

/**
 * @brief Thiết lập việc dịch con trỏ khi in ký tự trên màn hình LCD.
 */
void HD44780_ShiftIncrement();

/**
 * @brief Thiết lập việc dịch con trỏ khi in ký tự trên màn hình LCD.
 */
void HD44780_ShiftDecrement();

/**
 * @brief Tắt đèn nền của màn hình LCD.
 */
void HD44780_NoBacklight();

/**
 * @brief Bật đèn nền của màn hình LCD.
 */
void HD44780_Backlight();

/**
 * @brief Bật chế độ cuộn tự động trên màn hình LCD.
 */
void HD44780_AutoScroll();

/**
 * @brief Tắt chế độ cuộn tự động trên màn hình LCD.
 */
void HD44780_NoAutoScroll();

/**
 * @brief Tạo một ký tự đặc biệt trên màn hình LCD.
 *
 * @param char_num Vị trí của ký tự đặc biệt (0-7).
 * @param rows Mảng chứa dữ liệu của ký tự đặc biệt (5 byte).
 */
void HD44780_CreateSpecialChar(uint8_t char_num, uint8_t rows[]);

/**
 * @brief In một ký tự đặc biệt trên màn hình LCD.
 *
 * @param char_num Vị trí của ký tự đặc biệt (0-7).
 */
void HD44780_PrintSpecialChar(uint8_t char_num);

/**
 * @brief Thiết lập vị trí con trỏ trên màn hình LCD.
 *
 * @param row Số dòng (0 hoặc 1).
 * @param col Số cột (0-15 trong trường hợp màn hình 2 dòng).
 */
void HD44780_SetCursor(uint8_t row, uint8_t col);

/**
 * @brief Thiết lập cường độ đèn nền của màn hình LCD.
 *
 * @param new_val Giá trị mới của cường độ đèn nền (0-255).
 */
void HD44780_SetBacklight(uint8_t new_val);

 /** @brief Tải một ký tự đặc biệt từ bộ nhớ vào màn hình LCD.
 *
 * @param char_num Vị trí của ký tự đặc biệt (0-7).
 * @param rows Dữ liệu của ký tự đặc biệt (5 byte).
 */
void HD44780_LoadCustomCharacter(uint8_t char_num, uint8_t *rows);

/**
 * @brief In một chuỗi ký tự lên màn hình LCD.
 *
 * @param str Chuỗi ký tự cần in.
 */
void HD44780_PrintStr(const char str[]);

#endif /* LIQUIDCRYSTAL_I2C_H_ */

