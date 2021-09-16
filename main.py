import network
import socket
from machine import Pin,PWM
import json
import math

# global variables

errorsum_linear = 0.0
errordiff_linear = 0.0
errorsum_rotate = 0.0
errordiff_rotate = 0.0
linear_error = 15.0
rotate_error = 5
motor1pin1_raw = None
motor1pin1 = None
motor1pin2_raw = None
motor1pin2 = None
motor2pin1_raw = None
motor2pin1 = None
motor2pin2_raw = None
motor2pin2 = None
target_angle = 0.0
current_angle = 0.0

def WiFi_Connect():
    sta_if = network.WLAN(network.STA_IF)
    sta_if.active(True)
    ssid = 'grid'
    pwd = 'qwertyuiop'
    sta_if.connect(ssid,pwd)
    print('Connecting...')
    while not sta_if.isconnected():
        pass
    print('Connected: ',sta_if.ifconfig())

def tolerance(param1,param2,error):
    """Used to make a tolerance of ambiguous errors"""
    res = param1-param2
    neverror = 0-error
    if(res <= error and res >= neverror):
        return 0.0
    else:
        return res

def equality(param1,param2,error):
    """Used to make a tolerance of ambiguous errors"""
    return abs(param1-param2)<=error

def euler_distance(x1,y1,x2,y2):
    return ((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))**0.5

def move_rotate(pid):
    global motor1pin1
    global motor1pin2 
    global motor2pin1
    global motor2pin2
    pid = int(pid)
    if pid<0:
        pid = 0-pid
        motor1pin1.duty(pid)
        motor1pin2.duty(0)
        motor2pin1.duty(0)
        motor2pin2.duty(pid)  
    else:
        motor1pin1.duty(0)
        motor1pin2.duty(pid)
        motor2pin1.duty(pid)
        motor2pin2.duty(0)
        
def move_linear(pid):
    global motor1pin1
    global motor1pin2 
    global motor2pin1
    global motor2pin2
    pid = int(pid)
    if pid>=0:
        motor1pin1.duty(pid)
        motor1pin2.duty(0)
        motor2pin1.duty(pid)
        motor2pin2.duty(0)    
    else:
        pid = 0-pid
        motor1pin1.duty(0)
        motor1pin2.duty(pid)
        motor2pin1.duty(0)
        motor2pin2.duty(pid)

def PIDLinear(distance,angle_diff):
    K_p = 5
    K_i = 0.0
    K_d = 0.0
    global errorsum_linear
    global errordiff_linear
    errorsum_linear += distance
    errordiff_linear = distance - errordiff_linear
    pid = K_p*distance + K_i*errorsum_linear + K_d*errordiff_linear
    if equality(angle_diff,180) or equality(angle_diff,-180):
        pid = 0-pid
    move_linear(pid)
    errordiff_linear = distance


def PIDRotate(angle_diff):
    K_p = 5
    K_i = 0.0
    K_d = 0.0
    if angle_diff>=180:
        angle_diff = angle_diff - 360
    elif angle_diff<-180:
        angle_diff = angle_diff + 360
    global errorsum_rotate
    global errordiff_rotate
    errorsum_rotate += angle_diff
    errordiff_rotate = angle_diff - errordiff_rotate
    pid = K_p*angle_diff + K_i*errorsum_rotate + K_d*errordiff_rotate
    errordiff_rotate = angle_diff
    move_rotate(pid)

















def stop(m1p1,m1p2,m2p1,m2p2):
    """This function will take in motor pins and apply PID to make it stop"""
    m1p1.duty(0)
    m1p2.duty(0)
    m2p1.duty(0)
    m2p2.duty(0)


def main():
    global target_angle
    global current_angle
    global motor1pin1_raw
    global motor1pin1
    global motor1pin2_raw
    global motor1pin2
    global motor2pin1_raw 
    global motor2pin1
    global motor2pin2_raw
    global motor2pin2
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    IP = '10.42.0.1'
    port = 5001
    s.connect((IP,port))
    print('Connected to server')
    robot_id = 1
    s.sendall(str(robot_id).encode('ascii'))
    with open("pinConfig.json","r") as f:
        pins = json.loads(f.read())
    motor1pin1_raw = Pin(int(pins["leftwheel"]["pos"]),Pin.OUT)
    motor1pin1 = PWM(motor1pin1_raw)
    motor1pin2_raw = Pin(int(pins["leftwheel"]["neg"]), Pin.OUT)
    motor1pin2 = PWM(motor1pin2_raw)
    motor2pin1_raw = Pin(int(pins["rightwheel"]["pos"]), Pin.OUT)
    motor2pin1 = PWM(motor2pin1_raw)
    motor2pin2_raw = Pin(int(pins["rightwheel"]["neg"]), Pin.OUT)
    motor2pin2 = PWM(motor2pin2_raw)
    servo_raw = Pin(int(pins["servo"]), Pin.OUT)
    servo = PWM(servo_raw,freq = 50) # port in int
    data = ''
    pose = dict()
    #i = 1
    while True:
        buf = s.recv(64)
        data = data + buf.decode('ascii')
        delimiter_index = data.find('|')
        if delimiter_index == -1:
            continue
        else:
            try:
                pose = json.loads(data[0:delimiter_index])
                data1 = data
                data = data[delimiter_index+1:]
                print(str(pose))
            except:
                data1 = data
                data = data[delimiter_index+1:]
                continue
        #i+=1
        #if(i%100==0):
            #print(str(pose))
            #i=1
        try:
            current_angle = pose['pose']['theta']
            target_angle = pose['target']['theta']
            angle_diff = target_angle-current_angle
            if (not equality(angle_diff,0)) and ((not equality(angle_diff,180)) or (not equality(angle_diff,-180))):
                PIDRotate(angle_diff)
            else:
                PIDLinear(euler_distance(pose['pose']['x'],pose['pose']['y'],pose['target']['x'],pose['target']['y']),angle_diff)
            if pose['flip']:
                servo.duty(0)
            else:
                servo.duty(0)
        except:
            continue

if __name__ == "__main__":
    WiFi_Connect()
    main()
