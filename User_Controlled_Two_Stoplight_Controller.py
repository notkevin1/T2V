from mfrc522 import SimpleMFRC522
import Stoplight
import Car
import time
import multitimer
import sys
import select

RED_RED_STATE = 0
GREEN_RED_STATE = 1
YELLOW_RED_STATE = 2
RED_YELLOW_STATE = 3
RED_GREEN_STATE = 4

CURRENT_STATE = RED_RED_STATE
NEXT_STATE = GREEN_RED_STATE

timerCount = 1
currentTimerCount = 0
timerLength = 2

Car1 = Car.Car("FC:58:FA:22:CA:65", 1061751233680)
Car2 = Car.Car("FC:58:FA:22:BC:A6", 245521366938)
Car3 = Car.Car("FC:58:FA:22:B6:3D", 863444709922)
Car4 = Car.Car("00:14:03:05:FF:D2", 317637173945)

S1stoppedCarID = 0
S2stoppedCarID = 0

S1 = Stoplight.Stoplight(22,27,17)
S2 = Stoplight.Stoplight(13,6,5)

reader1 = SimpleMFRC522(0)
reader2 = SimpleMFRC522(1)

Car1.connect()
Car2.connect()
# Car3.connect()

Car1.send('1')
Car2.send('1')
# Car3.send('1')

currentRFID = 0

def determineMessage(RFIDid, msg):
    if RFIDid == Car1.getRFIDid():
        Car1.send(msg)
        return 0
    if RFIDid == Car2.getRFIDid():
        Car2.send(msg)
        return 0
    elif RFIDid == Car3.getRFIDid():
        Car3.send(msg)
        return 0
    elif RFIDid == Car4.getRFIDid():
        Car4.send(msg)
        return 0
    return -1

def RFIDread(reader):
    global currentRFID
    try:
        RFIDid, text = reader.read()
    except Exception as e:
        print(e)
    return RFIDid

def changeStates():
    global timerCount
    global currentTimerCount
    global CURRENT_STATE
    global NEXT_STATE
    global LAST_STATE
    currentTimerCount += 1
    if currentTimerCount == timerCount:
        LAST_STATE = CURRENT_STATE
        CURRENT_STATE = NEXT_STATE
        currentTimerCount = 0

timer = multitimer.MultiTimer(timerLength, changeStates)
timer.start()

while True:
    while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        line = sys.stdin.readline()
        if line:
            if 'l' in line:
                currentTimerCount = currentTimerCount - 1
            elif 'p' in line:
                currentTimerCount = timerCount - 1
        else:
            continue
    if CURRENT_STATE == RED_RED_STATE:
        rfidID = RFIDread(reader1)
        if determineMessage(rfidID, '0') != -1:
            S1stoppedCarID = rfidID
        rfidID = RFIDread(reader2)
        if determineMessage(rfidID, '0') != -1:
            S2stoppedCarID = rfidID
        timerCount = 2
        S1.redLight()
        S2.redLight()
        if LAST_STATE == YELLOW_RED_STATE:
            NEXT_STATE = RED_GREEN_STATE
        elif LAST_STATE == RED_YELLOW_STATE:
            NEXT_STATE = GREEN_RED_STATE
    elif CURRENT_STATE == GREEN_RED_STATE:
        rfidID = RFIDread(reader2)
        if determineMessage(rfidID, '0') != -1:
            S2stoppedCarID = rfidID
        if S1stoppedCarID != 0:
            determineMessage(S1stoppedCarID, '1')
            S1stoppedCarID = 0
        timerCount = 5
        S1.greenLight()
        S2.redLight()
        NEXT_STATE = YELLOW_RED_STATE
    elif CURRENT_STATE == YELLOW_RED_STATE:
        rfidID = RFIDread(reader2)
        if determineMessage(rfidID, '0') != -1:
            S2stoppedCarID = rfidID
        timerCount = 1
        S1.yellowLight()
        S2.redLight()
        NEXT_STATE = RED_RED_STATE
    elif CURRENT_STATE == RED_GREEN_STATE:
        rfidID = RFIDread(reader1)
        if determineMessage(rfidID, '0') != -1:
            S1stoppedCarID = rfidID
        if S2stoppedCarID != 0:
            determineMessage(S2stoppedCarID, '1')
            S2stoppedCarID = 0
        timerCount = 5
        S1.redLight()
        S2.greenLight()
        NEXT_STATE = RED_YELLOW_STATE
    elif CURRENT_STATE == RED_YELLOW_STATE:
        rfidID = RFIDread(reader1)
        if determineMessage(rfidID, '0') != -1:
            S1stoppedCarID = rfidID
        timerCount = 1
        S1.redLight()
        S2.yellowLight()
        NEXT_STATE = RED_RED_STATE