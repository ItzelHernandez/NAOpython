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

# Global variable to store the ReactToTouch module instance
ReactToTouch = None
memory = None
flag = False
initialSentence = """ 
    Vamos a separar basura
"""

# Vision --------------------------

def selectPath()
  


def contoursFilter():
  ##-----Read Mask--------------------##
  img = cv2.imread('dilation3.png',0)
  ##-----Threshold Filter-------------##
  ret,thresh = cv2.threshold(img,127,255,0)
  ##-----Find contours-------------##
  contours,hierarchy = cv2.findContours(thresh, 1, 2)

  return contours

def redFilter(hsv):
    lower_range = np.array([0, 50, 50], dtype=np.uint8) #red color
    upper_range = np.array([10, 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_range, upper_range)

    #Remove noise of the selected mask
    kernel = np.ones((5,5),np.uint8)
    erosion = cv2.erode(mask, kernel, iterations=1)
    erosion2 = cv2.erode(erosion, kernel, iterations=1)
    erosion3 = cv2.erode(erosion2, kernel, iterations=1)
    dilation = cv2.dilate(erosion3,kernel, iterations =1)
    dilation2 = cv2.dilate(dilation,kernel, iterations =1) 
    dilation3 = cv2.dilate(dilation2,kernel, iterations =1)

    #cv2.imshow('dilation3',dilation3)
    cv2.imwrite('dilation3.png', dilation3)

    contRed= contoursFilter()
    lenContRed= len(contRed)

    return lenContRed


def brownFilter(hsv):
  lower_range = np.array([20, 50, 50], dtype=np.uint8) 
  upper_range = np.array([40, 255, 255], dtype=np.uint8)

  mask = cv2.inRange(hsv, lower_range, upper_range)

  #Remove noise of the selected mask
  kernel = np.ones((5,5),np.uint8)
  erosion = cv2.erode(mask, kernel, iterations=1)
  erosion2 = cv2.erode(erosion, kernel, iterations=1)
  erosion3 = cv2.erode(erosion2, kernel, iterations=1)
  dilation = cv2.dilate(erosion3,kernel, iterations =1)
  dilation2 = cv2.dilate(dilation,kernel, iterations =1) 
  dilation3 = cv2.dilate(dilation2,kernel, iterations =1)

  dilation3Brown = dilation3
  # cv2.imshow('dilation3Brown',dilation3Brown)
  cv2.imwrite('dilation3.png', dilation3)

  contBrown= contoursFilter()
  
  lenContBrown= len(contBrown)

  return lenContBrown


def whiteFilter(hsv):
  lower_range = np.array([0, 0, 140], dtype=np.uint8) #red color
  upper_range = np.array([0, 255, 255], dtype=np.uint8)

  mask = cv2.inRange(hsv, lower_range, upper_range)

  #Remove noise of the selected mask
  kernel = np.ones((5,5),np.uint8)
  erosion = cv2.erode(mask, kernel, iterations=1)
  erosion2 = cv2.erode(erosion, kernel, iterations=1)
  erosion3 = cv2.erode(erosion2, kernel, iterations=1)
  dilation = cv2.dilate(erosion3,kernel, iterations =1)
  dilation2 = cv2.dilate(dilation,kernel, iterations =1) 
  dilation3 = cv2.dilate(dilation2,kernel, iterations =1)

  dilation3White = dilation3
  # cv2.imshow('dilation3White',dilation3White)
  cv2.imwrite('dilation3.png', dilation3)
  
  contWhite= contoursFilter()

  lenContWhite= len(contWhite)

  return lenContWhite


# Vision --------------------------


# Walk ----------------------------

def walkTurnAround():

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
  #####################
  initRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False))
  X = 0
  Y = 0
  Theta = (math.pi)-0.5
  motionProxy.post.moveTo(X, Y, Theta)
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
  print "Robot Move:", robotMove

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
    
    # ---------- ------------------ ----------------- #
    # ---------- Vision Recognition ----------------- #
    photoCP = ALProxy('ALPhotoCapture')
    photoCP.setResolution(2)
    photoCP.setPictureFormat('jpg')
    photoCP.takePictures(5,'/home/nao/pythonProjects', 'nao')
    img=cv2.imread('nao_4.jpg')  #take the last image (the good one)
    #cv2.imshow('nao9',img)
    #hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    sleep(3)

    l0=redFilter(hsv) #Metal
    l1=brownFilter(hsv) #Carton
    l2=whiteFilter(hsv) #Plastico
      
    if l0 != 0 or l1 != 0 or l2 != 0:
      l=[l0,l1,l2]
      l.sort()
      print("l array:")
      print(l);

    colorDetected= l[2] #guarda mayor longitud

    if colorDetected == l0:
      tts.say("Es metal")
      walkTurnAround()
      print("Red detected")
    
    elif colorDetected == l1:
      tts.say("Es Carton")
      print("Brown detected")

    elif colorDetected == l2:
      tts.say("Es Plastico")
      print("White detected")    

    else:
      tts.say("No detecto nada")

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
