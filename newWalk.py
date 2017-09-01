import sys
import motion
import time
from time import sleep
import almath
import math
from math import exp,pow,fabs,cos,sin,sqrt
from naoqi import ALProxy


initialSentence = """ 
    Vamos
"""


####--------------------- Global variables, all proxies and Nao resources
class globalVariables:

    def __init__(self, robotIP):
        self.IP = robotIP
        print(self.IP)
        PORT = 9559
        try:
            self.motion = ALProxy("ALMotion", self.IP, PORT)
        except Exception, e:
            print "Could not create proxy to ALMotion"
            print "Error was: ", e

        try:
            self.posture = ALProxy("ALRobotPosture", self.IP, PORT)
        except Exception, e:
            print "Could not create proxy to ALRobotPosture"
            print "Error was: ", e
       
        try:
            self.memory = ALProxy("ALMemory", self.IP, PORT)
        except Exception,e:
            print "Could not create proxy to ALRobotPosture"
            print "Error was: ", e

        try:
            self.sonar = ALProxy("ALSonar", self.IP, PORT)
        except Exception, e:
            print "Could not create proxy to ALSonar"
            print "Error was: ", e


    #position = [X,Y,ABSTHETA,RELTHETA]
    X = 0
    Y = 1
    ABSTHETA = 2
    RELTHETA = 3
    V = 4

    #Paths
    LSONAR = "Device/SubDeviceList/US/Left/Sensor/Value"
    RSONAR = "Device/SubDeviceList/US/Right/Sensor/Value"
    ANGLEZ = "Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value" 


####---------------------


def adjustmentTheta2(gVars, targetAngle):
    #Get the actual value of theta
    #actRelTheta = gVars.memory.getData(gVars.ANGLEZ)
    difTheta = angleDifference(gVars,targetAngle)
    print "Angle Without adjustment : " , gVars.memory.getData(gVars.ANGLEZ) , "Target Angle : " , targetAngle , "Difference Theta : " , difTheta
    #Move the difference
    gVars.motion.moveTo(0,0,difTheta,getWalkCalibration(difTheta))

# FRONT, LEFT, RIGHT or BACK
def angleDifference(gVars, targetAngle):
    actRelTheta = gVars.memory.getData(gVars.ANGLEZ)
    AngleDif = actRelTheta-targetAngle
    ##print AngleDif
    # To set Rotation sense
    # Check if correct 
    # n represents the sign of AngleDif
    if(AngleDif>0):
        n = 1
    else:
        n = -1  


#-------------------------------------------------------------------------------------------------------------#
#                                           main()                                                            #
#-------------------------------------------------------------------------------------------------------------
def main(robotIP):

    # Init proxies.    
    gVars = globalVariables(robotIP)
    gVars.posture.goToPosture("StandInit",0.5) 

    cont =0 
    initialAngle= gVars.memory.getData(gVars.ANGLEZ) #initialAngle
    
    #Los tres giros que debe de hacer el nao para leer la NaoMark
    gVars.motion.moveTo(0, 0, (math.pi/6) )

    gVars.motion.moveTo(0, 0, -(math.pi/6) )

    gVars.motion.moveTo(0, 0, -(math.pi/6) )

    while (True):
        
        actRelTheta = gVars.memory.getData(gVars.ANGLEZ)
        
        print" actual valor theta"
        print actRelTheta

        print" initial "
        print initialAngle

        sleep(1)

        #gVars.motion.moveTo(0.5, 0, 0 )

        
'''
        #check if wall
        gVars.sonar.subscribe("UltraSonicSensors")
        
        promLSonar=0
        promDSonar=0

        acumLSonar=0
        acumRSonar=0

        for n in range(1,10):
            lsonar = gVars.memory.getData(gVars.LSONAR)
        
            acumLSonar= lsonar + acumLSonar
        
        promLSonar= acumLSonar/10

        print "SONARS: ", promLSonar
'''

#-------------------------------------------------------------------------------------------------------------#
#                                                 -                                                           #
#-------------------------------------------------------------------------------------------------------------#


# To be modified if we want to load programm with an already explored graph()
if __name__ == "__main__":
    robotIp = "127.0.0.1"

    if len(sys.argv) <= 1:
        print "Usage python gVars.motion_walk.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)

    # 