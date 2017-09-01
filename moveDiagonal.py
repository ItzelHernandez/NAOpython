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
naoMarkRed= 64
naoMarkWhite= 80
naoMarkBrown= 108

initialSentence = """ 
    Vamos
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

# Walk ----------------------------

def walkRoutine1():

  motionProxy  = ALProxy('ALMotion')
  postureProxy = ALProxy('ALRobotPosture')
# Wake up robot
  motionProxy.wakeUp()

  # Send robot to Stand Init
  postureProxy.goToPosture("StandInit", 0.5)

  #####################
  ## Enable arms control by move algorithm
  #####################
  motionProxy.setMoveArmsEnabled(True, True)
  
  #####################
  ## FOOT CONTACT PROTECTION
  #####################
  motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

  #####################
  ## get robot position before move
  ##################### Avanza diagonal, primera validacion
  initRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False)) 
  X = 0.60
  Y = 0.80
  Theta = 0#(math.pi)/2.0
  motionProxy.moveTo(X, Y, Theta, [ ["MaxStepX", 0.045],["MaxStepFrequency", 0.2] ]) 
  sleep(1)
  # wait is useful because with post moveTo is not blocking function
  motionProxy.waitUntilMoveIsFinished()

  #####################
  ## get robot position after move
  #####################
  endRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False))

  #####################
  # compute and print the robot motion
  #####################
  robotMove = m.pose2DInverse(initRobotPosition)*endRobotPosition
  # return an angle between ]-PI, PI]
  robotMove.theta = m.modulo2PI(robotMove.theta)
  #print "Robot Move:", robotMove

  postureProxy.goToPosture("StandInit", 0.5)


def walkRoutine2():

  motionProxy  = ALProxy('ALMotion')
  postureProxy = ALProxy('ALRobotPosture')

  postureProxy.goToPosture("StandInit", 0.5)
  motionProxy.setMoveArmsEnabled(True, True)
  motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

  #####################
  ## get robot position before move
  ##################### Avanza diagonal, primera validacion
  initRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False)) 
  X = 0.45
  Y = -0.70
  Theta = 0#(math.pi)/2.0
  motionProxy.moveTo(X, Y, Theta, [ ["MaxStepX", 0.045],["MaxStepFrequency", 0.2] ]) 
  sleep(1)
  # wait is useful because with post moveTo is not blocking function
  motionProxy.waitUntilMoveIsFinished()

  #####################
  ## get robot position after move
  #####################
  endRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False))

  #####################
  # compute and print the robot motion
  #####################
  robotMove = m.pose2DInverse(initRobotPosition)*endRobotPosition
  # return an angle between ]-PI, PI]
  robotMove.theta = m.modulo2PI(robotMove.theta)
  #print "Robot Move:", robotMove

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
    
    walkRoutine2()
    sleep(1)
    value= readNaoMark()

    print value
    
    if value == naoMarkRed:
      print"voy a la canasta roja"

    else:
      print"no es igual"
  
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


if len(li) == 0:
    print('the list is empty')