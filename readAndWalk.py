
import sys
from time import sleep
import numpy as np
import cv2

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
import argparse

import motion
import time
import almath
import math
from math import exp,pow,fabs,cos,sin,sqrt

nao_ip="0.0.0.0"

# Global variable to store the ReactToTouch module instance
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

ReactToTouch = None
memory = None
flag = False
initialSentence = """ 
    Hola! 
"""

# ValidateNaoMark --------------

def readNaoMark():

  # Create a proxy to ALLandMarkDetection
  try:
    landMarkProxy = ALProxy("ALLandMarkDetection")
  except Exception, e:
    print "Error when creating landmark detection proxy:"
    print str(e)
    exit(1)

  # Subscribe to the ALLandMarkDetection proxy
  # This means that the module will write in ALMemory with
  # the given period below
  period = 500
  landMarkProxy.subscribe("Test_LandMark", period, 0.0 )

  # ALMemory variable where the ALLandMarkdetection modules
  # outputs its results
  memValue = "LandmarkDetected"

  # Create a proxy to ALMemory
  try:
    memoryProxy = ALProxy("ALMemory")
  except Exception, e:
    print "Error when creating memory proxy:"
    print str(e)
    exit(1)
    
    global naoMarkDetected 
    #naoMarkDetected= 64
    #print "Nao Mark detected:"
    #print naoMarkDetected


  # A simple loop that reads the memValue and checks whether landmarks are detected.
  for i in range(0, 10):
    time.sleep(0.5)
    val = memoryProxy.getData(memValue)

    # Check whether we got a valid output.
    if(val and isinstance(val, list) and len(val) >= 2):
      # Second Field = array of Mark_Info's.
      markInfoArray = val[1]

      try:
        # Browse the markInfoArray to get info on each detected mark.
        for markInfo in markInfoArray:

          # Second Field = Extra info (ie, mark ID).
          markExtraInfo = markInfo[1]

          naoMarkDetected= markExtraInfo[0]

          return naoMarkDetected

      except Exception, e:
        print "Error msg %s" % (str(e))
    else:
        print "No landmark detected"


  # Unsubscribe the module.
  landMarkProxy.unsubscribe("Test_LandMark")

  print "Test terminated successfully."

# ValidateNaoMark --------------





####---------------------####
#       MAIN ROUTINE  #
####---------------------####
def mainRoutine():
    # Greetings
    tts = ALProxy('ALTextToSpeech')

    # ---------- Speech Recognition ----------------- #
    asr = ALProxy('ALSpeechRecognition')
    tempMem = ALProxy('ALMemory')
    asr.setLanguage('Spanish')
    tts.say(initialSentence)
    vocabulary = ['si', 'no', 'porfavor']
    
    #Define NaoMarks
    naoMarkValue=64
    naoMarkDetected=0

    #Initialize global variables
    gVars = globalVariables(nao_ip)

    #Nao Initial posture
    gVars.posture.goToPosture("StandInit",0.5) 
    initialAngle= gVars.memory.getData(gVars.ANGLEZ) #initialAngle
    

    #Read NaoMarks

    gVars.motion.moveTo(0, 0, -(math.pi/6))
    value= readNaoMark()
    print value

    if value == naoMarkValue:
      print"este es el camino"
      gVars.motion.moveTo(1, 0, 0)

    else:
      gVars.motion.moveTo(0, 0, (math.pi/6))
      value= readNaoMark()
      print value

      if value == naoMarkValue:
        print"este es el camino"

      else:
        
        gVars.motion.moveTo(0, 0, (math.pi/6))
        print" voy en camino" # esta es cuando debe de ir a al naoMark 64
        gVars.motion.moveTo(1, 0, 0)
        #value= readNaoMark()
        #print value

          
    '''
    while (True):
        
        actRelTheta = gVars.memory.getData(gVars.ANGLEZ)
        
        print" actual valor theta"
        print actRelTheta

        print" initial "
        print initialAngle

        sleep(1)
'''

    print ("fin")

####---------------------####
#       END MAIN ROUTINE  #
####---------------------####



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

    #nao_ip= ip

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

    global nao_ip
    nao_ip= args.ip

    main(args.ip, args.port)
