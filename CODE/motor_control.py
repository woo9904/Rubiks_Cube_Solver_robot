#!/usr/bin/env python
# -*- coding: utf-8 -*-

#--------------------------
#모터를 제어하기 위해서 만든 함수
#---------------------------
import os
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import time

flag=0
cycle=10
correction=0

#포트 열기
def open_port():
    global packetHandler, portHandler, ADDR_TORQUE_ENABLE, ADDR_GOAL_POSITION,TORQUE_DISABLE, TORQUE_ENABLE, getch, ADDR_PRESENT_POSITION, DXL_MOVING_STATUS_THRESHOLD
    if os.name == 'nt':
        import msvcrt
        def getch():
            return msvcrt.getch().decode()
    else:
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        def getch():
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

    MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 115200

    PROTOCOL_VERSION            = 2.0

    DEVICENAME                  = '/dev/ttyUSB0'
    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold

    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

#모터 동작시키기
def motor_move(motor_id, dir ,times):
    global packetHandler, portHandler
    #1번 모터 동작
    if motor_id==1:
        DXL1_ID = 1                 # Dynamixel#1 ID : 1

        cycle=times #1번 모터가 한바퀴를 넘길때 늘어남
        flag=dir
        dxl_goal_position_1 = [930, 1954, 2978, 4002]

        # Enable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL1_ID)

        
        #write goal position
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, dxl_goal_position_1[flag]+cycle*4096+correction)
        print(flag, cycle, correction)
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))


        while 1:
            dxl1_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

            if not ((abs(dxl_goal_position_1[flag]+cycle*4096+correction - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
                break

        # Disable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    #2번 모터 동작
    if motor_id==2:
        DXL2_ID = 2                 # Dynamixel#2 ID : 2

        cycle_2=times #2번 모터가 한바퀴를 넘길때 늘어남
        dxl_goal_position_2 = 4096

        # Enable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL2_ID)

        # Write goal position
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, dxl_goal_position_2*cycle_2)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        while 1:
            # Syncread present position
            #time.sleep(0.01)
            dxl2_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

            #print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL2_ID, dxl_goal_position_2*cycle_2, dxl2_present_position))

            if not ((abs(dxl_goal_position_2*cycle_2 - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
                #time.sleep(0.1)
                break

        # Disable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            
    #3번 모터 동작
    if motor_id==3:
        DXL3_ID = 3                 # Dynamixel#3 ID : 3

        index_3=dir
        dxl_goal_position_3 = [400, 1200]

        # Enable Dynamixel#3 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL3_ID)

        #write goal position
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_POSITION, dxl_goal_position_3[index_3])

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        while 1:
            dxl3_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

            if not ((abs(dxl_goal_position_3[index_3] - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
                break

        # Disable Dynamixel#3 Torque
        #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        #if dxl_comm_result != COMM_SUCCESS:
            #print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        #elif dxl_error != 0:
            #print("%s" % packetHandler.getRxPacketError(dxl_error))

#모터 초기화 시키기
def motor_initial():
    global flag, cycle, index, cycle_2
    flag=0
    cycle=10
    motor_move(1, flag, cycle)

    index=1
    motor_move(3, index, index)
    
    time.sleep(1)

    cycle_2=5
    motor_move(2, cycle_2, cycle_2)

#1번 모터 동작시키기 wrapper
def motor_1(dir, times):
    global flag, cycle, correction
    if dir=="+":
        flag+=times
    elif dir=="-":
        flag-=times

    if flag>=4:
        flag=flag-4
        cycle+=1
    elif flag<0:
        flag=flag+4
        cycle-=1

    if index==0:
        correction=150
        if dir=="-":
            correction=-150
        
        motor_move(1, flag, cycle) #1번 모터, flag 인덱스만큼, cycle횟수만큼
        correction=0
        motor_move(3, 1, 1) #3 motor Up,
        time.sleep(0.1)
        motor_move(1, flag, cycle) #1번 모터, flag 인덱스만큼, cycle횟수만큼
    else:
        correction=0
        motor_move(1, flag, cycle) #1번 모터, flag 인덱스만큼, cycle횟수만큼

#2번 모터 동작시키기 wrapper
def motor_2(dir, times):
    global cycle_2
    if dir=="+":
        cycle_2+=times
    elif dir=="-":
        cycle_2-=times

    motor_move(2, cycle_2, cycle_2)

#3번 모터 동작시키기 wrapper
def motor_3(dir):
    global index
    if dir=="U":
        if index==1:
            return
        else:
            index=1
    elif dir=="D":
        if index==0:
            return
        else:
            index=0

    motor_move(3, index, index)

#총 모터 동작
def robot_move_s(input_msg):
    global flag, do_dir_opposite
    do_it=input_msg[0]
    num=int(input_msg[1])
    do_dir_opposite=False

    # 3회인 경우 돌리는 방향 반대
    if num==3:
        do_dir_opposite=True
        num=1

    #U는 뒤집으면 반대방향임. 
    if do_it=="U":
        motor_3("U")
        time.sleep(0.1)
        motor_2("+", 2)
        time.sleep(0.1)
        motor_3("D")

        if do_dir_opposite==True: #방향 반대
            input_dir="+"
        else: #방향 그대로
            input_dir="-"
        motor_1(input_dir, num)

        #초기상태 복귀
        motor_3("U")
        time.sleep(0.5)
        motor_2("-", 2)
    elif do_it=="D":
        motor_3("D")
        if do_dir_opposite==True:
            input_dir="+"
        else:
            input_dir="-"
        motor_1(input_dir, num)

        #초기상태 복귀
        motor_3("U")
    elif do_it=="L":
        motor_3("U")
        motor_1("+", 1)
        time.sleep(0.1)
        motor_2("+", 1)
        time.sleep(0.1)
        motor_3("D")
        if do_dir_opposite==True: #방향 반대
            input_dir="+"
        else: #방향 그대로
            input_dir="-"
        motor_1(input_dir, num)

        #초기상태 복귀
        motor_3("U")
        time.sleep(0.5)
        motor_2("-", 3)
        time.sleep(0.1)
        motor_1("-", 1)
    elif do_it=="R":
        motor_3("U")
        motor_1("-", 1)
        motor_2("-", 1)
        motor_3("D")
        if do_dir_opposite==True: #방향 반대
            input_dir="+"
        else: #방향 그대로
            input_dir="-"
        time.sleep(0.1)
        motor_1(input_dir, num)

        #초기상태
        motor_3("U")
        time.sleep(0.5)
        motor_2("+", 3)
        time.sleep(0.1)
        motor_1("+", 1)
    elif do_it=="F":
        motor_3("U")
        time.sleep(0.1)
        motor_2("+", 3)
        time.sleep(0.1)
        motor_3("D")
        if do_dir_opposite==True: #방향 반대
            input_dir="+"
        else: #방향 그대로
            input_dir="-"
        time.sleep(0.1)
        motor_1(input_dir, num)

        #초기
        motor_3("U")
        time.sleep(0.1)
        motor_2("-", 1)
    elif do_it=="B":
        motor_3("U")
        time.sleep(0.1)
        motor_2("+", 1)
        time.sleep(0.1)
        motor_3("D")
        if do_dir_opposite==True: #방향 반대
            input_dir="+"
        else: #방향 그대로
            input_dir="-"
        time.sleep(0.1)
        motor_1(input_dir, num)

        #초기
        motor_3("U")
        time.sleep(0.5)
        motor_2("-", 3)
        time.sleep(0.1)



#포트 종료
def close_port():
    # Close port
    portHandler.closePort()


if __name__=="__main__":
    open_port()
    motor_initial()
    close_port()
