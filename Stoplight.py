import time
import RPi.GPIO as GPIO

class Stoplight:
    def __init__(self, rPin, yPin, gPin):
        self.rPin = rPin
        self.yPin = yPin
        self.gPin = gPin
        self.state = "none"
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(rPin, GPIO.OUT)
        GPIO.setup(yPin, GPIO.OUT)
        GPIO.setup(gPin, GPIO.OUT)
        GPIO.output(rPin, 0)
        GPIO.output(yPin, 0)
        GPIO.output(gPin, 0)

    def redLight(self):
        GPIO.output(self.rPin, 1)
        GPIO.output(self.yPin, 0)
        GPIO.output(self.gPin, 0)
        self.state = "red"
    
    def yellowLight(self):
        GPIO.output(self.rPin, 0)
        GPIO.output(self.yPin, 1)
        GPIO.output(self.gPin, 0)
        self.state = "yellow"

    def greenLight(self):
        GPIO.output(self.rPin, 0)
        GPIO.output(self.yPin, 0)
        GPIO.output(self.gPin, 1)
        self.state = "green"

    def getState(self):
        return self.state

    def closeLight(self):
        GPIO.cleanup()