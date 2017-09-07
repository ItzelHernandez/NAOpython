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

    #Step 11
    walkParameterFixedRotation = [ ["MaxStepX", 0.08],["MaxStepY", 0.14] ,["MaxStepTheta", 0.1963],["MaxStepFrequency", 0.5],["StepHeight", 0.04],["TorsoWx", 0.0],["TorsoWy", 0.0]]
    

####---------------------

# FRONT, LEFT, RIGHT or BACK
def angleDifference(gVars, initialAngle):
    actRelTheta = gVars.memory.getData(gVars.ANGLEZ)
    AngleDif = actRelTheta-targetAngle
    ##print AngleDif
    # To set Rotation sense
    # Check if correct 
    # n represents the sign of AngleDif
    if(AngleDif>0):
        n=1
      
    else:
        n=-1
        

#-------------------------------------------------------------------------------------------------------------#
#                                           main()                                                            #
#-------------------------------------------------------------------------------------------------------------
def main(robotIP):

    # Init proxies.    
    gVars = globalVariables(robotIP)
    gVars.posture.goToPosture("StandInit",0.5) 

    initialAngle= gVars.memory.getData(gVars.ANGLEZ) #initialAngle
    position = gVars.motion.getRobotPosition(False)
    print "Robot Move:", position

    gVars.motion.moveTo(0.75, 0, 0) #gira 30
        
    actRelTheta = gVars.memory.getData(gVars.ANGLEZ)
        
    print" actual valor theta=" + str (actRelTheta)

    print" initial=  " + str(initialAngle)

    if (initialAngle >0):

        if (actRelTheta >0):
            correctionAngle= ((actRelTheta)- initialAngle) #cuando ambos son positivos
            print "correct=  " +  str(correctionAngle)
            gVars.motion.moveTo(0, 0, (correctionAngle),gVars.walkParameterFixedRotation)
            position =gVars.motion.getRobotPosition(False)
            print "Robot Move:", position 

        else:
            correctionAngle= (actRelTheta*-1)- initialAngle
            correctionAngle= correctionAngle*(-1)
            print "correct=  " +  str(correctionAngle)
            gVars.motion.moveTo(0, 0, (correctionAngle), gVars.walkParameterFixedRotation) 
            position = gVars.motion.getRobotPosition(False)
            print "Robot Move:", position

    elif (initialAngle <0):

        if (actRelTheta >0):
            correctionAngle= actRelTheta - (initialAngle*-1)
            correctionAngle= correctionAngle* (-1)
            print "correct=  " +  str(correctionAngle)
            gVars.motion.moveTo(0, 0, (correctionAngle),gVars.walkParameterFixedRotation)
            position = gVars.motion.getRobotPosition(False)
            print "Robot Move:", position

        else: #ambos son negativos
            correctionAngle= actRelTheta - (initialAngle)
            print "correct=  " +  str(correctionAngle)
            gVars.motion.moveTo(0, 0, (correctionAngle),gVars.walkParameterFixedRotation)
            position = gVars.motion.getRobotPosition(False)
            print "Robot Move:",position

        
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