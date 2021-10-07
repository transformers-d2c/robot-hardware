import network
import socket
from machine import Pin,PWM
import json

# global variables

motor1pin1_raw = None
motor1pin1 = None
motor1pin2_raw = None
motor1pin2 = None
motor2pin1_raw = None
motor2pin1 = None
motor2pin2_raw = None
motor2pin2 = None
servo = None

def WiFi_Connect():
    sta_if = network.WLAN(network.STA_IF)
    sta_if.active(True)
    ssid = 'dlink'
    pwd = '12345678'
    sta_if.connect(ssid,pwd)
    print('Connecting...')
    while not sta_if.isconnected():
        pass
    print('Connected: ',sta_if.ifconfig())


def w():
    global motor1pin1
    global motor1pin2 
    global motor2pin1
    global motor2pin2
    motor1pin1.on()
    motor1pin2.off()
    motor2pin1.on()
    motor2pin2.off()

def s():
    global motor1pin1
    global motor1pin2 
    global motor2pin1
    global motor2pin2
    motor1pin1.off()
    motor1pin2.on()
    motor2pin1.on()
    motor2pin2.off()

def a():
    global motor1pin1
    global motor1pin2
    global motor2pin1
    global motor2pin2
    motor1pin1.off()
    motor1pin2.on()
    motor2pin1.on()
    motor2pin2.off()

def d():
    global motor1pin1
    global motor1pin2
    global motor2pin1
    global motor2pin2
    motor1pin1.on()
    motor1pin2.off()
    motor2pin1.off()
    motor2pin2.on()

def flip():
    global servo
    servo.duty(125)
    servo.duty(25)
        
def move_linear(pid):
    global motor1pin1
    global motor1pin2 
    global motor2pin1
    global motor2pin2
    pid = int(pid)
    if pid>=0:
        motor1pin1.duty(pid)
        motor1pin2.duty(0)
        motor2pin1.duty(int(pid))
        motor2pin2.duty(0)    
    else:
        pid = 0-pid
        motor1pin1.duty(0)
        motor1pin2.duty(pid)
        motor2pin1.duty(0)
        motor2pin2.duty(int(pid))

def stop():
    """This function will take in motor pins and aupply PID to make it stop"""
    motor1pin1.off()
    motor1pin2.off()
    motor2pin1.off()
    motor2pin2.off()


def main():
    global motor1pin1
    global motor1pin2
    global motor2pin1
    global motor2pin2
    global servo
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    IP = '192.168.0.105'
    port = 5001
    s.connect((IP,port))
    print('Connected to server')
    robot_id = 2
    s.sendall(str(robot_id).encode('ascii'))
    with open("pinConfig.json","r") as f:
        pins = json.loads(f.read())
    motor1pin1 = Pin(int(pins["leftwheel"]["pos"]),Pin.OUT)
    motor1pin2 = Pin(int(pins["leftwheel"]["neg"]), Pin.OUT)
    motor2pin1 = Pin(int(pins["rightwheel"]["pos"]), Pin.OUT)
    motor2pin2 = Pin(int(pins["rightwheel"]["neg"]), Pin.OUT)
    servo_raw = Pin(int(pins["servo"]), Pin.OUT)
    servo = PWM(servo_raw,freq = 50) # port in int
    while True:
        buf = s.recv(64)
        buf = buf.decode("ascii")
        elimiter_index = data.find('|')
        if delimiter_index == -1:
            continue
        else:
            try:
                data = json.loads(data[0:delimiter_index])
            except:
                data1 = data
                data1 = data[delimiter_index+1:]
                data = data1.loads(data[0:delimiter_index])
                continue
        val = data["direction"]
        flip = data["flip"]
        if(val == "w"): # forward
            w()
        elif(val == "s"): # backward
            s()
        elif(val == "a"): # left
            a()
        elif(val == "d"): # right
            d()
        else:
            stop()
        if(val == "True"):
            flip()


if __name__ == "__main__":
    WiFi_Connect()
    main()
