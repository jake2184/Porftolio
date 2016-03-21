#!/usr/bin/python
import math
import time
import sys


import Adafruit_CharLCD as LCD

# Raspberry Pi pin configuration:
# The spare GPIO pin numbers
lcd_rs        = 21  
lcd_en        = 20
lcd_d4        = 19
lcd_d5        = 13
lcd_d6        = 6
lcd_d7        = 5
lcd_backlight = 4


# Define LCD column and row size for 16x2 LCD.
lcd_columns = 16
lcd_rows    = 2

lcd = LCD.Adafruit_CharLCD(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7, 
							lcd_columns, lcd_rows, lcd_backlight)


# Script run with an argument on PITS to display a tweet. 
if __name__ == '__main__':

    lcd = LCD.Adafruit_CharLCD(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7, lcd_columns, lcd_rows, lcd_backlight)
	
    message = bytes(sys.argv[1]).decode("unicode_escape")
    if(len(message) > 16):
        lcd.message(message[:16] + "\n" + message[16:])
    else:
        lcd.message(message)