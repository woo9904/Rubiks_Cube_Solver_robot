#!/usr/bin/env python
# coding: utf-8

import cube_detect_pi #큐브 인식
import motor_control #모터 제어

import RPi.GPIO as GPIO #GPIO
import time
import sys
from threading import Thread
sys.path.append("//home//pi//lcd")
import drivers

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
            GPIO.cleanup()
            break


#LCD
# Load the driver and set it to "display"
# If you use something from the driver library use the "display." prefix first

def LCD_start():
    if is_detect==1:
        display.lcd_clear()
        display.lcd_display_string("detecting...", 1)
        time.sleep(1)
    if is_solving==1:
        display.lcd_clear()
        display.lcd_display_string("solving...", 1)
    if is_finish==1:
        display.lcd_clear()
        display.lcd_display_string("clear!", 1)

# Main body of code
def LCD_before_start():
    global next_stage
    # Remember that your sentences can only be 16 characters long!
    display.lcd_clear()
    while 1:

        if next_stage==0:
            if wait_for_start==0: #버튼이 눌러졌을때 
                display.lcd_clear()
                display.lcd_display_string("OH It's time to ", 1)  # Write line of text to first line of display
                display.lcd_display_string("start! - wait 5s", 2)  # Write line of text to second line of display
                time.sleep(5)
                next_stage=1
                


            else:
                display.lcd_display_string("HI! I am Robot:)", 1)   # Refresh the first line of display with a different message
                display.lcd_display_string("Press Btn2Start!", 2)
        else:
            LCD_start()


def LCD_clear():
    print("Cleaning up!")
    display.lcd_clear()
#-----------------------------------------------------------------
def move_robot(solution_list):
    motor_control.open_port()
    for i in range(len(solution_list)):
        print("current state : ", solution_list[i])
        motor_control.robot_move_s(solution_list[i])
    print("solution finish!")
    motor_control.close_port()

if __name__=="__main__":
    global is_detect, is_solving, is_finish, wait_for_start
    #GPIO 제어
    led1 = 18
    led2=23
    btn=17

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(led1, GPIO.OUT)
    GPIO.setup(led2, GPIO.OUT)
    GPIO.setup(btn, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    wait_for_start=1 #버튼 누르는지 확인하는 변수

    display = drivers.Lcd()

    next_stage=0
    is_detect=0
    is_solving=0
    is_finish=0

    #1 단계 (초기화 스위치 누를 시)
    th1=Thread(target=use_GPIO, args=())
    th2=Thread(target=LCD_before_start, args=())
    th1.start()
    th2.start()

    th1.join()
    print("Stage 1")
    print("We need initializing. ")
    is_detect=1
    time.sleep(5)

    motor_control.open_port()
    motor_control.motor_initial()
    motor_control.close_port()
    #--------------------------------------------------
    #2 단계 (큐브 알아볼 수 있는 위치로 돌리기)
    #--------------------------------------------------
    #3 단계 (큐브 인식 하기)
    print("Stage 3")
    cube_solution_string=cube_detect_pi.main()
    is_detect=0
    is_solving=1

    #4 단계 큐브 풀기
    print("Stage 4")
    move_robot(cube_solution_string)
    is_solving=0
    is_finish=1
    #5단계 큐브 포트 끝
    print("Stage 5")
    motor_control.close_port()

