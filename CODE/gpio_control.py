#! /usr/bin/env python
# coding: utf-8

import RPi.GPIO as GPIO
import time
import sys
from threading import Thread
sys.path.append("//home//pi//lcd")
import drivers


#GPIO 제어
led1 = 18
led2=23
btn=17

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(led1, GPIO.OUT)
GPIO.setup(led2, GPIO.OUT)
GPIO.setup(btn, GPIO.IN, pull_up_down=GPIO.PUD_UP)

wait_for_start=1

def use_GPIO():
    global wait_for_start
    while 1:
        #버튼이 눌릴때 
        if GPIO.input(btn):
            GPIO.output(led1, GPIO.HIGH)
            GPIO.output(led2, GPIO.LOW)
        else: #버튼을 안렀을 때
            GPIO.output(led1, GPIO.LOW)
            GPIO.output(led2, GPIO.HIGH)
            time.sleep(0.1)
            GPIO.output(led2, GPIO.LOW)
            time.sleep(0.1)
            wait_for_start=0 #버튼 누름

def GPIO_close():
    GPIO.cleanup()

#LCD

# Load the driver and set it to "display"
# If you use something from the driver library use the "display." prefix first
display = drivers.Lcd()

booting=1

# Main body of code
def LCD_before_start():
    # Remember that your sentences can only be 16 characters long!
    display.lcd_clear()
    while 1:

        if wait_for_start==0: #버튼이 눌러졌을때 
            display.lcd_clear()
            display.lcd_display_string("OH It's time to ", 1)  # Write line of text to first line of display
            display.lcd_display_string("start! - wait 5s", 2)  # Write line of text to second line of display
            time.sleep(5)

        else:
            display.lcd_display_string("HI! I am Robot:)", 1)   # Refresh the first line of display with a different message
            display.lcd_display_string("Press Btn2Start!", 2)


def LCD_clear():
    print("Cleaning up!")
    display.lcd_clear()


if __name__=="__main__":
    th1=Thread(target=use_GPIO, args=())
    th2=Thread(target=LCD_before_start, args=())
    th1.start()
    th2.start()




