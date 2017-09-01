import sys
from time import sleep
import numpy as np
import cv2

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
import math
import almath as m # python's wrapping of almath
from math import exp,pow,fabs,cos,sin,sqrt
import argparse
import time

# Global variable to store the ReactToTouch module instance
ReactToTouch = None
memory = None
flag = False
naoMarkRed= 64
naoMarkWhite= 80
naoMarkBrown= 108

initialSentence = """ 
    Vamos
"""


####--------------------- Global variables, all proxies and Nao resources
class globalVariables(myBroker):
    tempMem = ALProxy('ALMemory')
    
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



#-------------------------------------------------------------------------------------------------------------#
#                                           main()                                                            #
#-------------------------------------------------------------------------------------------------------------
def mainRoutine():

    # Greetings
    tts = ALProxy('ALTextToSpeech')
    # si jala pero debemos checar en un ambiente mas real al del concurso como
    # se va a comportar el reconocimiento
    
    # ---------- Speech Recognition ----------------- #
    asr = ALProxy('ALSpeechRecognition')
    
    asr.setLanguage('Spanish')
    tts.say(initialSentence)

    # Init proxies.    
    gVars = globalVariables()
    #gVars.posture.goToPosture("StandInit",0.5) 
    motionProxy  = ALProxy('ALMotion')
    postureProxy = ALProxy('ALRobotPosture')
    # Wake up robot
    motionProxy.wakeUp()
    cont =0 
    initialAngle= gVars.tempMem.getData(gVars.ANGLEZ) #initialAngle
    #gVars.motion.moveTo(0.5, 0, 0 )

    while (True):
        
        actRelTheta = gVars.tempMem.getData(gVars.ANGLEZ)
        
        print" actual valor theta"
        print actRelTheta

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


# ---------- ------------------ ----------------- #

class ReactToTouch(ALModule):
    """ A simple module able to react
        to touch events.
    """
    def __init__(self, name):
        ALModule.__init__(self, name)
        # No need for IP and port here because
        # we have our Python broker connected to NAOqi broker

        # Create a proxy to ALTextToSpeech for later use
        # self.tts = ALProxy("ALTextToSpeech")

        # Subscribe to TouchChanged event:
        global memory
        memory = ALProxy('ALMemory')
        memory.subscribeToEvent('MiddleTactilTouched',
            'ReactToTouch',
            'onTouchedStart')

    def onTouchedStart(self, strVarName, value):
        """ This will be called each time a touch
        is detected.
        """
        # Unsubscribe to the event when starts the routine,
        # to avoid repetitions
        memory.unsubscribeToEvent('MiddleTactilTouched',
            'ReactToTouch')
        
        mainRoutine() # This will be the main routine

        # Subscribe again to the event (RearTactilTouched)
        memory.subscribeToEvent('RearTactilTouched',
            'ReactToTouch',
            'onTouchedEnd')

    def onTouchedEnd(self, strVarName, value):
        # global variable to let the application that the routine has ended
        global flag
        flag = True

def main(ip, port):
    """ Main entry point
    """
    # We need this broker to be able to construct
    # NAOqi modules and subscribe to other modules
    # The broker must stay alive until the program exists
    myBroker = ALBroker('myBroker',
       '0.0.0.0',   # listen to anyone
       0,           # find a free port and use it
       ip,          # parent broker IP
       port)        # parent broker port


    global ReactToTouch
    global flag
    ReactToTouch = ReactToTouch('ReactToTouch')

    try:
        while True:
            if flag:
                print
                print 'The routine has finished in a clean way.'
                myBroker.shutdown()
                sys.exit(0)
            
    except KeyboardInterrupt:
        print
        print 'Interrupted by user, shutting down'
        myBroker.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', type=str, default='127.0.0.1',
                        help='Robot ip address')
    parser.add_argument('--port', type=int, default=9559,
                        help='Robot port number')
    args = parser.parse_args()
    main(args.ip, args.port)

    # 