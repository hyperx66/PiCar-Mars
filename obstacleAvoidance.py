import turn
import time
import move
import ultra
import Adafruit_PCA9685

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

move.move_forward(80)

def moveAngle(degree):
    if degree>wheelMax:degree=wheelMax
    elif degree<wheelMin:degree=wheelMin
    pwm.set_pwm(2,0,degree)

#Set 3 camera position
def cameraPos1():
    turn.neck_turn(350)
    
def cameraPos2():
    turn.neck_turn(520)
    
def cameraPos3():
    turn.neck_turn(210)
    

while True:
    if scanPos == 1:
        cameraPos2()
        time.sleep(0.15)
        scanList[0] = ultra.checkdist()
    elif scanPos == 2:
        cameraPos1()
        time.sleep(0.15)
        scanList[1] = ultra.checkdist()
    elif scanPos == 3:
        cameraPos3()
        time.sleep(0.15)
        scanList[2] = ultra.checkdist()
        
    scanPos += scanDir #update scanPos

    if scanPos > scanNum or scanPos <1:
        #replace scan direction
        if scanDir == 1: scanDir = -1
        elif scanDir == -1: scanDir = 1
        #restore scanned location
        scanPos += scanDir*2
    
    if min(scanList)<rangeKeep:
        if scanList.index(min(scanList)) == 0:
            moveAngle(200)
        elif scanList.index(min(scanList)) == 2:
            moveAngle(300)
        elif scanList.index(min(scanList)) == 1:
            if scanList[0] < scanList[2]:
                moveAngle(200)
            else:
                moveAngle(300)
        if max(scanList) < rangeKeep or min(scanList) < rangeKeep/3:
            print("Object sibei near")
            move.move_backward(80)
            
    else:
        moveAngle(240)
        move.move_forward(80)