import turn
import time
import move
import ultra
import Adafruit_PCA9685

#Prerequisites
move.setup()
time.sleep(1)
turn.resetMotor()

scanNum = 3 #left, middle, right
scanPos = 1 #next pos for ultrasonic scanning 
scanDir = 1 #dir for robot scanning, 1 as leftmost, -1 as rightmost
scanList = [0,1,2] #stores dist of obstacles in 3 dir
scanServo = 1 #serial num of servo
rangeKeep = 1.0 #threshold of obstacle's distance

wheelMax = 350
wheelMin = 180

#Set 3 camera position
def cameraPos1():
    turn.neck_turn(430)
    
def cameraPos2():
    turn.neck_turn(580)
    
def cameraPos3():
    turn.neck_turn(300)