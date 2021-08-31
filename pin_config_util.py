import json

def get_pins():
    with open('pinConfig.json','r') as inp:
        data = json.load(inp)
        print(str(data))


def set_pins(lp,ln,rp,rn,s):
    with open('pinConfig.json','w') as out:
        data = {
            'leftwheel':{
                'pos':lp,
                'neg':ln
            },
            'rightwheel':{
                'pos':rp,
                'neg':rn
            },
            'servo':s
        }
        json.dump(data,out)


def set_pins_input():
    lp = int(input("Left wheel positive:"))
    ln = int(input("Left wheel negative:"))
    rp = int(input("Right wheel positive:"))
    rn = int(input("Right wheel negative:"))
    s = int(input("Servo:"))
    set_pins(lp,ln,rp,rn,s)
