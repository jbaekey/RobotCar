import math
import time
import threading
import RPi.GPIO as GPIO
import asyncio
import evdev
from evdev import InputDevice, categorize, ecodes

# Hard coded input source should be changed to identify any/first bluetooth gamepad
# dev = InputDevice('/dev/input/event2')
# Arbitrary
deadZone = 20

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)
GPIO.setup(4, GPIO.OUT)

topVal=0
middleVal=128
bottomVal=255

smoothCriteria=10
targetSpeed=0
currentSpeed=0

straightAngle=90
steeringAngleFullRange=60
steeringAngleHalfRange=steeringAngleFullRange/2
leftAngleLimit=straightAngle-steeringAngleHalfRange
rightAngleLimit=straightAngle+steeringAngleHalfRange

headlightState = GPIO.LOW
GPIO.output(4,headlightState)
forwardPWM = GPIO.PWM(18, 50 )
backwardPWM = GPIO.PWM(13, 50)
servoPWM =  GPIO.PWM(26, 50)


forwardPWM.start(0)
backwardPWM.start(0)
servoPWM.start(0)

def calcServoAngle(angle):
    duty = angle/18+2
    return duty



def setMotorSpeed(speed):
    if speed > 0:
        backwardPWM.ChangeDutyCycle(0)
        forwardPWM.ChangeDutyCycle(speed)
    elif speed < 0:
        speed = speed * -1
        forwardPWM.ChangeDutyCycle(0)
        backwardPWM.ChangeDutyCycle(speed)
    else:
        forwardPWM.ChangeDutyCycle(0)
        backwardPWM.ChangeDutyCycle(0)

def smooth(current,target,steps):
    diff=target-current
    print("Smoothing:",current,target,math.floor(diff/steps))
    for x in range(current,target,math.floor(diff/steps)):
        print("Setting smooth speed to ",x)
        setMotorSpeed(x)
    return target

async def helper(dev):
    async for ev in dev.async_read_loop():
        global targetSpeed,currentSpeed,headlightState
        if ev.type == ecodes.EV_ABS:
            if ev.code == ecodes.ABS_Y:
                ev.value = ((ev.value - 128)/128)*(math.pi/2)
                targetSpeed = math.floor(math.sin(ev.value)*100)*-1
                print("Setting targetSpeed to ",targetSpeed)
                setMotorSpeed(targetSpeed)
                currentSpeed=targetSpeed
                #smoother.update()
            # Steering servo control
            #  90 degrees of rotation between 45 and 135
            #  Needs servo cutoff after 2 or more seconds or more of inactivity
            elif ev.code == ecodes.ABS_Z:
                print(ev.value," value received")
                if ev.value > middleVal-deadZone and ev.value < middleVal+deadZone:
                    servoPWM.ChangeDutyCycle(calcServoAngle(90))
                    print("Turning degrees: 90\nSteer straight")
                    #time.sleep(servoMovementDelay)
                    #servoPWM.ChangeDutyCycle(0)
                else:
                    if ev.value > middleVal:
                        newAngle=((ev.value-(middleVal+deadZone))/(middleVal-deadZone)*steeringAngleHalfRange)+straightAngle
                        servoPWM.ChangeDutyCycle(calcServoAngle(newAngle))
                        print("Turning degrees: ",newAngle)
                        #time.sleep(servoMovementDelay)
                        #servoPWM.ChangeDutyCycle(0)
                    else:
                        newAngle=((ev.value)/(middleVal-deadZone)*steeringAngleHalfRange)+(straightAngle-steeringAngleHalfRange)
                        servoPWM.ChangeDutyCycle(calcServoAngle(newAngle))
                        print("Turning degrees: ",newAngle)
                        #time.sleep(servoMovementDelay)
                        #servoPWM.ChangeDutyCycle(0)
        elif ev.type == ecodes.EV_KEY:
            if ev.code == ecodes.BTN_B and ev.value == 1:
                if headlightState == GPIO.LOW:
                    print("Turning on headlight")
                    headlightState = GPIO.HIGH
                else:
                    print("Turning off headlight")
                    headlightState = GPIO.LOW
            GPIO.output(4,headlightState)

while True:
    try:
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        dev = None
        for device in devices:
            if ecodes.EV_ABS in device.capabilities():
                dev = InputDevice(device.path)
        if dev is not None:
            print("Controller found!")
            loop = asyncio.get_event_loop()
            loop.run_until_complete(helper(dev))
        else:
            print("No controllers found.  Sleeping for 2 seconds")
            time.sleep(2)
    except OSError:
        dev=None





