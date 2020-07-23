import RPi.GPIO as GPIO
import time
L1 = 35
L2 = 37
R1 = 36
R2 = 38
GPIO.setmode(GPIO.BOARD)
GPIO.setup(L1, GPIO.OUT)
GPIO.setup(L2, GPIO.OUT)
GPIO.setup(R1, GPIO.OUT)
GPIO.setup(R2, GPIO.OUT)

def TurnRight(delay):
    GPIO.output(L1, GPIO.HIGH)
    GPIO.output(L2, GPIO.HIGH)
    time.sleep(delay)
    Stop()
    #make pins high and stop after delay
def TurnLeft(delay):
    GPIO.output(R1, GPIO.HIGH)
    GPIO.output(R2, GPIO.HIGH)
    time.sleep(delay)
    Stop()

def Forward(delay):
    GPIO.output(L1, GPIO.HIGH)
    GPIO.output(L2, GPIO.HIGH)
    GPIO.output(R1, GPIO.HIGH)
    GPIO.output(R2, GPIO.HIGH)
    time.sleep(delay)
    Stop()

def Stop():
    GPIO.output(L1, GPIO.LOW)
    GPIO.output(L2, GPIO.LOW)
    GPIO.output(R1, GPIO.LOW)
    GPIO.output(R2, GPIO.LOW)

Forward(2)
Stop()
GPIO.cleanup()
