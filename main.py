"""hopefully parts of code will work, mere pe nodeMCU nahi so i didnt test D:"""

import socket
from machine import Pin,PWM

# global variables
errorsum = 0
errordiff = 0

def PIDcontrol(current_coord,dest_coord,motorpin):
    error = dest_coord-current_coord # calculates error
    global errorsum
    global errordiff
    errorsum += error # build errorsum
    errordiff = error - errordiff # builds errordif
    Kp = 50 # subject to change in the tuning phase
    Ki = 0.2 # subject to change in the tuning phase
    Kd = 0 # subject to change in the tuning phase
    pwm = Kp*error + Ki*errorsum + Kd*errordiff # calcs pwn output
    motorpin.duty(pwm)

def parse_direction(data):
    anglediff = data[5] - data[2] # calculate difference of angle of dst and src
    if anglediff == 0:
        # only for forward and reverse
        if data[2] == 0:
            # if pointing towards +ve x axis
            xdiff = data[3] - data[0] # dst - src (required difference is +ve)
            if xdiff > 0:
                return "fw"
            elif xdiff < 0:
                return "rv"
            else:
                return "stop"
        elif data[2] == 180 or data[2] == -180:
            # if pointing towards -ve x axis
            xdiff = data[3] - data[0] # dst - src (required difference is -ve)
            if xdiff < 0:
                return "fw"
            elif xdiff > 0:
                return "rv"
            else:
                return "stop"
        elif data[2] == 90 or data[2] == -270:
            # if pointing towards +ve y axis
            ydiff = data[4] - data[1] # dst - src (required difference is +ve)
            if ydiff > 0:
                return "fw"
            elif ydiff < 0:
                return "rv"
            else:
                return "stop"
        elif data[2] == 270 or data[2] == -90:
            # if pointing towards -ve y axis
            ydiff = data[4] - data[1] # dst - src (required difference is -ve)
            if ydiff < 0:
                return "fw"
            elif ydiff > 0:
                return "rv"
            else:
                return "stop"

    else:
        # for left and right (might not work, i dont have much confidence in this piece of code, cuz no testing D:)
        if anglediff < 180:
            # if angle difference < 180 then if dest angle > current the it will turn left and dest angle < current the it will turn right
            if data[5] > data[2]:
                return "lf"
            elif data[5] < data[2]:
                return "rg"
        elif anglediff > 180:
            # if angle difference > 180 then if dest angle < current the it will turn left and dest angle > current the it will turn right
            if data[5] < data[2]:
                return "lf"
            elif data[5] > data[2]:
                return "rg"

def flip(sm):
    sm.duty(100) # subject to change depending on the angle
    sm.duty(40) # subject to change depending on the angle

def forward(m1p1,m1p2,m2p1,m2p2,current_coord,dest_coord):
    PIDcontrol(current_coord,dest_coord,m1p1)
    m1p2.off()
    PIDcontrol(current_coord,dest_coord,m2p1)
    m2p2.off()

def reverse(m1p1,m1p2,m2p1,m2p2,current_coord,dest_coord):
    m1p1.off()
    PIDcontrol(current_coord,dest_coord,m1p2)
    m2p1.off()
    PIDcontrol(current_coord,dest_coord,m2p2)

def left(m1p1,m1p2,m2p1,m2p2,current_coord,dest_coord):
    """m1p1.off()
    m1p2.on()
    m2p1.on()
    m2p2.off()"""
    # call this function for turning left
    pass

def right(m1p1,m1p2,m2p1,m2p2,current_coord,dest_coord):
    """m1p1.on()
    m1p2.off()
    m2p1.off()
    m2p2.on()"""
    # call this function for turning right
    pass

def stop(m1p1,m1p2,m2p1,m2p2):
    m1p1.off()
    m1p2.off()
    m2p1.off()
    m2p2.off()

def main():
    s =  socket.socket()
    motor1pin1_raw =  Pin(A,Pin.OUT)
    motor1pin1 = (motor1pin1_raw,freq = 30)
    motor1pin2_raw = Pin(B, Pin.OUT)
    motor1pin2 = (motor1pin2_raw, freq = 30)
    motor2pin1_raw = Pin(C, Pin.OUT)
    motor2pin1 = (motor2pin1_raw, freq = 30)
    motor2pin2_raw = Pin(D, Pin.OUT)
    motor2pin2 = (motor2pin2_raw, freq = 30)
    servo_raw = Pin(E, Pin.OUT)
    servo = PWM(servo_raw,freq = 30)
    s.connect("IP")
    while True:
        data = s.recv(1024) # subject to change depending on the amount of data we get from socket
        data = data.decode("ascii") # converting to ascii
        current_coords = data[:3]
        dest_coords = data[3:6]
        direction = parse_direction(data)
        flipbit = data[6]
        """if direction == "lf": # subject to change depending on the input we get from the socket
            left(motor1pin1,motor1pin2,motor2pin1,motor2pin2,current_coords,dest_coords)
        if direction == "rg": # subject to change depending on the input we get from the socket
            right(motor1pin1, motor1pin2, motor2pin1, motor2pin2,current_coords,dest_coords)
        if direction == "fw": # subject to change depending on the input we get from the socket
            forward(motor1pin1, motor1pin2, motor2pin1, motor2pin2,current_coords,dest_coords)
        if direction == "rv": # subject to change depending on the input we get from the socket
            reverse(motor1pin1, motor1pin2, motor2pin1, motor2pin2,current_coords,dest_coords)
        if direction == "stop": # subject to change depending on the input we get from the socket
            stop(motor1pin1, motor1pin2, motor2pin1, motor2pin2)
        if direction == "FLIP":
            flip(servo)""" # very naive approach
        if direction == "fw":
            forward(motor1pin1, motor1pin2, motor2pin1, motor2pin2, current_coords[1], dest_coords[1])
        if direction == "rv":
            reverse(motor1pin1, motor1pin2, motor2pin1, motor2pin2, current_coords[1], dest_coords[1])
        if direction == "stop":
            stop(motor1pin1, motor1pin2, motor2pin1, motor2pin2)
        else:
            # left right nahi smjh aa raha for now :)
            pass
        if flipbit == "1":
            flip(servo)