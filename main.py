"""hopefully parts of code will work, mere pe nodeMCU nahi so i didnt test D:"""
import network
import socket
from machine import Pin,PWM
import json

# global variables
errorsum_linear = 0
errordiff_linear = 0
errorsum_rotate = 0
errordiff_rotate = 0

# to be put in something similar to void setup no idea where it belongs to
def WiFi_Connect():
    sta_if = network.WLAN(network.STA_IF)
    sta_if.active(True)
    ssid = input('Enter SSID:')
    pwd = input('Enter password:')
    sta_if.connect(ssid,pwd)
    print('Connecting...')
    while not sta_if.isconnected():
        pass
    print('Connected: ',sta_if.ifconfig())


def PIDcontrollinear(current_coord,dest_coord,motorpin):
    """This function takes coordinates x,y and a motor pin and then applies PID control algo to it"""
    x_current_coord = float(current_coord[0]) # takes current x coord
    y_current_coord = float(current_coord[1]) # takes current y coord
    x_dest_coord = float(dest_coord[0]) # takes dest x coord
    y_dest_coord = float(dest_coord[1]) # takes dest y coord
    error_x = x_dest_coord - x_current_coord # calculates if error in x direction
    error_y = y_dest_coord - y_current_coord # calculates if error in y direction
    # if there is no error in x direction then the final error is error_y, else its error_x
    if error_x == 0:
        error = error_y
    else:
        error = error_x
    # probably the remaning code shouldnt matter on the direction, since PID in itself is applied on 1 dimension
    # but the errorsum and errordiff might need seperate variables for x and y coordinates.
    global errorsum_linear
    global errordiff_linear
    errorsum_linear += error # build errorsum
    errordiff_linear = error - errordiff_linear # builds errordif
    Kp = 50 # subject to change in the tuning phase
    Ki = 0.2 # subject to change in the tuning phase
    Kd = 0 # subject to change in the tuning phase
    pwm = Kp*error + Ki*errorsum_linear + Kd*errordiff_linear # calcs pwn output
    motorpin.duty(int(pwm))


def PIDcontrolrotate(current_coord_dict,dest_coord_dict,motorpin):
    """This function takes coordinates theta and a motor pin and then applies PID control algo to it"""
    current_coord = float(current_coord_dict)
    dest_coord = float(dest_coord_dict)
    error = dest_coord - current_coord
    global errorsum_rotate
    global errordiff_rotate
    errorsum_rotate += error # build errorsum
    errordiff_rotate = error - errordiff_rotate # builds errordif
    Kp = 50 # subject to change in the tuning phase
    Ki = 0.2 # subject to change in the tuning phase
    Kd = 0 # subject to change in the tuning phase
    pwm = Kp*error + Ki*errorsum_rotate + Kd*errordiff_rotate # calcs pwn output
    motorpin.duty(int(pwm))

def parse_direction(data):
    """This function gets data from socket and then returns if one should move fw, rv, clockwise or anti clockwise"""
    anglediff = float(data["target"]["theta"]) - float(data["pose"]["theta"]) # calculate difference of angle of dst and src
    if anglediff == 0:
        # only for forward and reverse
        if data["pose"]["theta"] == 0.0:
            # if pointing towards +ve x axis
            xdiff = float(data["target"]["x"]) - float(data["pose"]["x"]) # dst - src (required difference is +ve)
            if xdiff > 0.0:
                return "fw"
            elif xdiff < 0.0:
                return "rv"
            else:
                return "stop"
        elif data["pose"]["theta"] == 180.0 or data["pose"]["theta"] == -180.0:
            # if pointing towards -ve x axis
            xdiff = float(data["target"]["x"]) - float(data["pose"]["x"]) # dst - src (required difference is -ve)
            if xdiff < 0:
                return "fw"
            elif xdiff > 0:
                return "rv"
            else:
                return "stop"
        elif data["pose"]["theta"] == 90.0:
            # if pointing towards +ve y axis
            ydiff = float(data["target"]["y"]) - float(data["pose"]["y"]) # dst - src (required difference is +ve)
            if ydiff > 0.0:
                return "fw"
            elif ydiff < 0.0:
                return "rv"
            else:
                return "stop"
        elif data["pose"]["theta"] == -90.0:
            # if pointing towards -ve y axis
            ydiff = float(data["target"]["y"]) - float(data["pose"]["y"]) # dst - src (required difference is -ve)
            if ydiff < 0.0:
                return "fw"
            elif ydiff > 0.0:
                return "rv"
            else:
                return "stop"
    else:
        # for left and right (might not work, i dont have much confidence in this piece of code, cuz no testing D:)
        if anglediff < 180.0:
            # if angle difference < 180 then if dest angle > current the it will turn left and dest angle < current the it will turn right
            if data["target"]["theta"] > data["pose"]["theta"]:
                return "antiClock"
            elif data["target"]["theta"] < data["pose"]["theta"]:
                return "Clock"
        elif anglediff > 180.0:
            # if angle difference > 180 then if dest angle < current the it will turn left and dest angle > current the it will turn right
            if data["target"]["theta"] < data["pose"]["theta"]:
                return "antiClock"
            elif data["target"]["theta"] > data["pose"]["theta"]:
                return "Clock"


def flip(sm):
    """This function will be used to flip the load off the robot"""
    sm.duty(100) # subject to change depending on the angle
    sm.duty(40) # subject to change depending on the angle


def forward(m1p1,m1p2,m2p1,m2p2,current_coord,dest_coord):
    """This function will take in motor pins as well as coordinates of current as will as
    destination and apply PID to make it move fw"""
    PIDcontrollinear(current_coord,dest_coord,m1p1)
    m1p2.duty(0)
    PIDcontrollinear(current_coord,dest_coord,m2p1)
    m2p2.duty(0)


def reverse(m1p1,m1p2,m2p1,m2p2,current_coord,dest_coord):
    """This function will take in motor pins as well as coordinates of current as will as
        destination and apply PID to make it move rv"""
    m1p1.duty(0)
    PIDcontrollinear(current_coord,dest_coord,m1p2)
    m2p1.duty(0)
    PIDcontrollinear(current_coord,dest_coord,m2p2)


def antiClockwise(m1p1,m1p2,m2p1,m2p2,current_coord,dest_coord):
    """This function will take in motor pins as well as coordinates of current as will as
        destination and apply PID to make it move anti-clockwise"""
    m1p1.duty(0)
    PIDcontrolrotate(current_coord, dest_coord, m1p2)
    PIDcontrolrotate(current_coord, dest_coord, m2p1)
    m2p2.duty(0)


def Clockwise(m1p1,m1p2,m2p1,m2p2,current_coord,dest_coord):
    """This function will take in motor pins as well as coordinates of current as will as
        destination and apply PID to make it move clockwise"""
    PIDcontrolrotate(current_coord,dest_coord,m1p1)
    m1p2.duty(0)
    m2p1.duty(0)
    PIDcontrolrotate(current_coord,dest_coord,m2p2)


def stop(m1p1,m1p2,m2p1,m2p2):
    """This function will take in motor pins and apply PID to make it stop"""
    m1p1.duty(0)
    m1p2.duty(0)
    m2p1.duty(0)
    m2p2.duty(0)


def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    IP = input('Enter server IP:')
    port = int(input('Enter serve port'))
    s.connect((IP,port))
    print('Connected to server')
    robot_id = int(input('Robot id?'))
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
            pose = json.loads(data[0:delimiter_index])
            data = data[delimiter_index+1:]
        print(str(pose))
        #i+=1
        #if(i%100==0):
            #print(str(pose))
            #i=1
        current_coords = pose["pose"]
        dest_coords = pose["target"]
        direction = parse_direction(pose)
        flipbit = pose["flip"]
        if direction == "fw":
            forward(motor1pin1, motor1pin2, motor2pin1, motor2pin2, [float(current_coords["x"]),float(current_coords["y"])], [float(dest_coords["x"]),float(dest_coords["y"])])
        if direction == "rv":
            reverse(motor1pin1, motor1pin2, motor2pin1, motor2pin2, [float(current_coords["x"]),float(current_coords["y"])], [float(dest_coords["x"]),float(dest_coords["y"])])
        if direction == "stop":
            stop(motor1pin1, motor1pin2, motor2pin1, motor2pin2)
        if direction == "antiClock":
            antiClockwise(motor1pin1,motor1pin2,motor2pin1,motor2pin2,float(current_coords["theta"]),float(dest_coords["theta"]))
        if direction == "Clock":
            Clockwise(motor1pin1,motor1pin2,motor2pin1,motor2pin2,float(current_coords["theta"]),float(dest_coords["theta"]))
        if flipbit == "1":
            flip(servo)

if __name__ == "__main__":
    WiFi_Connect()
    main()