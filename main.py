import socket
from machine import Pin,PWM

def flip(a):
    a.duty(100)
    a.duty(40)

def forward(a,b,c,d):
    a.on()
    b.off()
    c.on()
    d.off()

def reverse(a,b,c,d):
    a.off()
    b.on()
    c.off()
    d.on()

def left(a,b,c,d):
    a.off()
    b.on()
    c.on()
    d.off()

def right(a,b,c,d):
    a.on()
    b.off()
    c.off()
    d.on()

def main():
    s =  socket.socket()
    motor1pin1 =  Pin(A,Pin.OUT) # subject to change depending on the GPIO pin number
    motor1pin2 = Pin(B, Pin.OUT) # subject to change depending on the GPIO pin number
    motor2pin1 = Pin(C, Pin.OUT) # subject to change depending on the GPIO pin number
    motor2pin2 = Pin(D, Pin.OUT) # subject to change depending on the GPIO pin number
    servopin = Pin(E, Pin.OUT) # subject to change depending on the GPIO pin number
    servo = PWM(servopin,freq = 50)
    s.connect("IP")
    while True:
        data = s.recv(1024) # subject to change depending on the amount of data we get from socket
        if data == "LEFT": # subject to change depending on the input we get from the socket
            left(motor1pin1,motor1pin2,motor2pin1,motor2pin2)
        if data == "RIGHT": # subject to change depending on the input we get from the socket
            right(motor1pin1, motor1pin2, motor2pin1, motor2pin2)
        if data == "FORWARD": # subject to change depending on the input we get from the socket
            forward(motor1pin1, motor1pin2, motor2pin1, motor2pin2)
        if data == "REVERSE": # subject to change depending on the input we get from the socket
            reverse(motor1pin1, motor1pin2, motor2pin1, motor2pin2)
        if data == "FLIP":
            flip(servo)
