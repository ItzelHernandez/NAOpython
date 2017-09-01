import sys
import motion
import time
from time import sleep
import almath
from math import exp,pow,fabs,cos,sin,sqrt
from naoqi import ALProxy

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

    
    pi = 3.14159265359
    walkParameterFixedRotation      = [ ["MaxStepX", 0.08],["MaxStepY", 0.14] ,["MaxStepTheta", 0.1963],["MaxStepFrequency", 0.5],["StepHeight", 0.04],["TorsoWx", 0.0],["TorsoWy", 0.0]]
    walkParameterAngularAdjustment  = [ ["MaxStepX", 0.08],["MaxStepY", 0.14] ,["MaxStepTheta", 0.0175],["MaxStepFrequency", 0.5],["StepHeight", 0.04],["TorsoWx", 0.0],["TorsoWy", 0.0]]
    walkParameterSideAdjustment     = [ ["MaxStepX", 0.08],["MaxStepY", 0.101],["MaxStepTheta", 0.0175],["MaxStepFrequency", 0.5],["StepHeight", 0.04],["TorsoWx", 0.0],["TorsoWy", 0.0]]

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
def main(robotIP, PORT=9559):

    # Init proxies.    
    gVars = globalVariables(robotIP)
    gVars.posture.goToPosture("StandInit",0.5) 
    cont =0 
    initialAngle= gVars.memory.getData(gVars.ANGLEZ) #initialAngle
    gVars.motion.moveTo(0.5, 0, 0 )

    while (cont<=1):
        
        actRelTheta = gVars.memory.getData(gVars.ANGLEZ)

        if actRelTheta != initialAngle:
            angleDifference= fabs(actRelTheta - initialAngle)
            print "angleDiff:"
            print angleDifference
        
        gVars.motion.moveTo(0.5, 0, 0 )

        cont= cont+1

        sleep(1)
        

#-------------------------------------------------------------------------------------------------------------#
#                                                 -                                                           #
#-------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    robotIp = "127.0.0.1"

    if len(sys.argv) <= 1:
        print "Usage python gVars.motion_walk.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)

    # 