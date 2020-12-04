#import libraries and files
import threading
import Haiqal.path as Haiqal
import JY.facial_req as JY
import obstacleAvoidance as Wendy
from move import destroy
import turn
import time
import move
import ultra
import Adafruit_PCA9685
import RPIO.GPIO as GPIO

#Prerequisites
move.setup()
time.sleep(1)
turn.resetMotor()
time.sleep(1)

scanNum = 3 #left, middle, right
scanPos = 1 #next pos for ultrasonic scanning 
scanDir = 1 #dir for robot scanning, 1 as leftmost, -1 as rightmost
scanList = [0,1,2] #stores dist of obstacles in 3 dir
scanServo = 1 #serial num of servo
rangeKeep = 0.9 #threshold of obstacle's distance

wheelMax = 350
wheelMin = 160

#Set 3 camera position
def cameraPos1():
    turn.neck_turn(430)
    
def cameraPos2():
    turn.neck_turn(580)
    
def cameraPos3():
    turn.neck_turn(300)
    
#PWM Settings
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

wheelMax = 350
wheelMin = 180

def moveAngle(degree):
    if degree>wheelMax:degree=wheelMax
    elif degree<wheelMin:degree=wheelMin
    pwm.set_pwm(2,0,degree)

def forward_left(instructionTime):
    moveAngle(300)
    move.move_forward(80)
    time.sleep(float(instructionTime))
    move.motorStop()

def backward_left(instructionTime):
    moveAngle(300)
    move.move_backward(80)
    time.sleep(float(instructionTime))
    move.motorStop()

def forward(instructionTime):
    move.move_backward(80)
    time.sleep(float(instructionTime))
    moveAngle(300)
    time.sleep(0.5)
    moveAngle(240)
    move.motorStop()

def turn_left(instructionTime):
    moveAngle(330)
    move.move_backward(80)
    time.sleep(float(instructionTime))
    move.motorStop()

if __name__ == "__main__":
    movingFlag = False

    # Process Coordinates
    instructArr = Haiqal.main()
    for instructionData in instructArr:
        instructionColl = instructionData.split(":")
        instruction = instructionColl[0]
        instructionTime = instructionColl[1]

        if instruction == "Forward_Left":
            forward_left(instructionTime)
        elif instruction == "Backward_Left":
            backward_left(instructionTime)
        elif instruction == "Forward":
            t1 = threading.Thread(target=Wendy.startAvoid)
            t2 = threading.Thread(target=forward, args=instructionTime)
            t1.start()
            t2.start()
            t1.join()
            t2.join()
        elif instruction == "Take Picture":
            JY.face_start()
            time.sleep(5)
            JY.face_stop()
        elif instruction == "Turn_left":
            t1 = threading.Thread(target=Wendy.startAvoid)
            t2 = threading.Thread(target=turn_left, args=instructionTime)
            t1.start()
            t2.start()
            t1.join()
            t2.join()
        elif instruction == "End":
            destroy()
            GPIO.cleanup()
    pass
