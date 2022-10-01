from machine import Pin,PWM
import json

high = 1023
low = 0

if __name__=='__main__':
    with open('pinConfig.json','r') as finp:
        pins = json.loads(finp.read())
    m1p1 = PWM(Pin(pins['leftwheel']['pos'],Pin.OUT))
    m1p2 = PWM(Pin(pins['leftwheel']['neg'],Pin.OUT))
    m2p1 = PWM(Pin(pins['rightwheel']['pos'],Pin.OUT))
    m2p2 = PWM(Pin(pins['rightwheel']['neg'],Pin.OUT))
    print(m1p1)
    print(m1p2)
    m1p1.duty(high)
    m1p2.duty(low)
    m2p1.duty(high)
    m2p2.duty(low)
