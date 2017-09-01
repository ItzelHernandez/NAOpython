# -*- encoding: UTF-8 -*-
"""
# API: doc.aldebaran.com/2-1/naoqi/sensors 
The routine starts with the MiddleTactilTouched and after the routine
has finished the RearTactilTouched exit the program. 
"""

import sys
from time import sleep
import numpy as np
import cv2

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
import math
import almath as m # python's wrapping of almath
import argparse
import time

# Global variable to store the ReactToTouch module instance
ReactToTouch = None
memory = None
flag = False
naoMarkDetected= 80

initialSentence = """ 
    Vamos
"""

# NaoMark detection ----------------------------
def detectNaoMark():

  global naoMarkDetected
  print "Nao Mark detected:"
  print naoMarkDetected

  try:
    landMarkProxy = ALProxy("ALLandMarkDetection")
  except Exception, e:
    print "Error when creating landmark detection proxy:"
    print str(e)
    exit(1)

  period = 500
  landMarkProxy.subscribe("Test_LandMark", period, 0.0 )

  memValue = "LandmarkDetected"

  # Create a proxy to ALMemory
  try:
    memoryProxy = ALProxy("ALMemory")
  except Exception, e:
    print "Error when creating memory proxy:"
    print str(e)
    exit(1)

  # A simple loop that reads the memValue and checks whether landmarks are detected.
  for i in range(0, 20):
    time.sleep(0.5)
    val = memoryProxy.getData(memValue)

    # Check whether we got a valid output.
    if(val and isinstance(val, list) and len(val) >= 2):
      timeStamp = val[0]
      markInfoArray = val[1]

      try:
        # Browse the markInfoArray to get info on each detected mark.
        for markInfo in markInfoArray:

          # First Field = Shape info.
          markShapeInfo = markInfo[0]

          # Second Field = Extra info (ie, mark ID).
          markExtraInfo = markInfo[1]
          #print "mark  ID: %d" % (markExtraInfo[0])
          print "markInfo"
          print markExtraInfo[0]


          if markExtraInfo[0] == naoMarkDetected : #naoMark 64
            print "es metal"
            walkToRed()

          if markExtraInfo[0] == naoMarkDetected: #naoMark 80
            print "es plastico"
            walkToWhite()

          if markExtraInfo[0] == naoMarkDetected: #naoMark 108
            print "es carton"
            walkToBrown()

      except Exception, e:
        print "Error msg %s" % (str(e))
    else:
        print "No landmark detected"

  landMarkProxy.unsubscribe("Test_LandMark")

# NaoMark detection ----------------------------



# Walk ----------------------------

def walkToRed():

  motionProxy  = ALProxy('ALMotion')
  postureProxy = ALProxy('ALRobotPosture')
# Wake up robot
  motionProxy.wakeUp()

  postureProxy.goToPosture("StandInit", 0.5)
  motionProxy.setMoveArmsEnabled(True, True)

  motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

  initRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False))
  X = 1.0 # 100 cm al frente
  Y = 0
  Theta = 0
  motionProxy.moveTo(X, Y, Theta, [ ["MaxStepX", 0.045],["MaxStepFrequency", 0.2] ]) # default of 0.02
  sleep(0.5)

  #Gira
  X = 0 
  Y = 0
  Theta = (math.pi/2)-0.4
  motionProxy.moveTo(X, Y, Theta,  [ ["MaxStepX", 0.045],["MaxStepFrequency", 0.2] ]) 

  #Avanza derecho
  sleep(0.5)
  X = 0.6 
  Y = 0
  Theta = 0
  motionProxy.moveTo(X, Y, Theta, [ ["MaxStepX", 0.045],["MaxStepFrequency", 0.2] ]) # default of 0.02

  sleep(1)

  #Gira
  X = 0 
  Y = 0
  Theta = -(math.pi)/2
  motionProxy.moveTo(X, Y, Theta,  [ ["MaxStepX", 0.045],["MaxStepFrequency", 0.2] ]) 

  #Avanza 30 cm al frente
  sleep(0.5)
  X = 0.5 # 50 cm al frente
  Y = 0
  Theta = 0
  motionProxy.moveTo(X, Y, Theta, [ ["MaxStepX", 0.045],["MaxStepFrequency", 0.2] ]) # default of 0.02

  ####
  sleep(1)
  # wait is useful because with post moveTo is not blocking function
  motionProxy.waitUntilMoveIsFinished()

  endRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False))
  robotMove = m.pose2DInverse(initRobotPosition)*endRobotPosition

  robotMove.theta = m.modulo2PI(robotMove.theta)
 
  print "Robot Move:", robotMove

  postureProxy.goToPosture("StandInit", 0.5)

  # Go to rest position
  motionProxy.rest()

# Walk ----------------------------


def mainRoutine():
    # Greetings
    tts = ALProxy('ALTextToSpeech')
    # si jala pero debemos checar en un ambiente mas real al del concurso como
    # se va a comportar el reconocimiento
    # ---------- Speech Recognition ----------------- #
    asr = ALProxy('ALSpeechRecognition')
    tempMem = ALProxy('ALMemory')
    asr.setLanguage('Spanish')
    tts.say(initialSentence)
    vocabulary = ['si', 'no', 'porfavor']
    # wait for answer
   
    #------Walking ------------#
    # valida NaoMark before walk
    
    #detectColor()

    global naoMarkDetected
    naoMarkDetected= 64


    detectNaoMark()

    sleep(2)
  
    print"Termina"
# ---------- ------------------ ----------------- #

class ReactToTouch(ALModule):
    """ A simple module able to react
        to touch events.
    """
    def __init__(self, name):
        ALModule.__init__(self, name)
        # No need for IP and port here because
        # we have our Python broker connected to NAOqi broker

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


if len(li) == 0:
    print('the list is empty')