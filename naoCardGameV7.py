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
import argparse

# Global variable to store the ReactToTouch module instance
ReactToTouch = None
memory = None
flag = False
initialSentence = """ 
    Hola! Me llamo NAO y vamos a jugar, Estan listos?
"""

# Vision --------------------------
trial=0 #intentos para buscar carta 

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
    # ---------- ------------------ ----------------- #
    # ---------- Vision Recognition ----------------- #
    #tts.say("Hola Seño")
    differentFruit = 0
    while(True):
      photoCP = ALProxy('ALPhotoCapture')
      photoCP.setResolution(2)
      photoCP.setPictureFormat('jpg')
      photoCP.takePictures(5,'/home/nao/pythonProjects', 'nao')
      image=cv2.imread('nao_4.jpg')  #take the last image (the good one)
      height, width = image.shape[:2]
      image2  = image.copy()
      image3  = np.zeros((height,width,3), np.uint8)        
      gray    = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
      thresh  = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
      kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 5))
      thresh  = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
      blurred = cv2.GaussianBlur(thresh, (5, 5), 0)
      edged   = cv2.Canny(blurred, 50, 200, 255)
      contours,hierarchy = cv2.findContours(edged,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
      
      p = 0
      validArea = 0
      biggestArea = 0;
      biggestContour = 0;
      for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True);
        if len(approx) == 4 or len(approx) == 5: 
          pt = contours[p]                   
          if(cv2.contourArea(pt) > biggestArea):
            biggestContour = p
            biggestPoint = pt
            biggestArea = cv2.contourArea(pt)
          if(biggestArea > (8000)):
            validArea = 1

        p = p + 1
      hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  
      
      red = green = blue = 0
      counter = 0
      for x in biggestPoint:
        rangePixels = 3
        xCoord = x[0][0]
        yCoord = x[0][1]

        for i in range(-1*rangePixels + xCoord ,rangePixels + xCoord ):
          for j in range(-1*rangePixels + yCoord,rangePixels + yCoord):
            if(i < width and i >= 0):
              if(j < height and j >= 0):
                bT = int(image2[j,i,0])
                gT = int(image2[j,i,1])
                rT = int(image2[j,i,2])

                color     = int(hsv[j,i,0])
                saturation  = int(hsv[j,i,1])
                value     = int(hsv[j,i,2])
                
                if(color < 250 and saturation > 80 and value > 80):
                  blue += bT
                  green += gT
                  red += rT
                  counter = counter + 1
                  cv2.circle(image,(i,j), 2, (0,200,0), -1)

      if counter != 0:
        blue = blue/(counter)
        green = green/(counter)
        red = red/(counter)

        if(validArea == 1):
          print ("Red: %d Green: %d Blue: %d\n" %(red, green, blue))
          if(red > green and blue > (0.6*red) and blue < red):
            print "Watermelon Detected\n"
            if(differentFruit != 1):
              tts.say("Esa es una sandía")
              differentFruit = 1
          elif(red > green and red > (1.1*blue)):
            print "Apple Detected\n"
            if(differentFruit != 2):
              tts.say("Esa es una manzana")
              differentFruit = 2
          elif(green > red and green > blue):
            print "Lemon Detected\n"
            if(differentFruit != 3):
              tts.say("Eso es un limón")
              differentFruit = 3
          elif(blue > green and blue > (1.1*red)):
            print "Pineapple detected\n"
            if(differentFruit != 4):
              tts.say("Esa es una piña")
              differentFruit = 4

    tts.say("Adios Kevin")


    '''
    tts.say("Ahora empecemos... podrias mostrarme la carta de un zapato?")
    sleep(5) #el niño busca la carta del perro, que es roja
    repeatRed=1

    while repeatRed > 0: #valida si es el perro de la carta roja
      photoCP.takePictures(5,'/home/nao/pythonProjects', 'nao')
      img=cv2.imread('nao_4.jpg') 
      hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      sleep(3)
      l0=redFilter(hsv)
      l1=brownFilter(hsv)
      l2=whiteFilter(hsv)  
      
      if l0 != 0 or l1 != 0 or l2 != 0:

        l=[l0,l1,l2]
        l.sort()
        print("l array:")
        print(l);

        colorDetected= l[2] #poner el nombre del color de la longitud mas larga

        if colorDetected == l0:
          tts.say("Muy bien, si es un zapato!")
          print("Red detected")
          repeatRed=0

        else:
          tts.say("Ups... vuelve a intentar")
          repeatRed=1

    tts.say("Ahora.. ¿podrias mostrarme la carta de un arbol?")
    sleep(5) #el niño busca la carta del perro, que es roja
    repeatBrown=1
    #hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    while repeatBrown > 0:
      photoCP.takePictures(5,'/home/nao/pythonProjects', 'nao')
      img=cv2.imread('nao_4.jpg') 
      hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      sleep(3)
      l0=redFilter(hsv)
      l1=brownFilter(hsv)
      l2=whiteFilter(hsv) 
      
      if l0 != 0 or l1 != 0 or l2 != 0:

        l=[l0,l1,l2]
        l.sort()
        print("l array:")
        print(l);

        colorDetected= l[2] #poner el nombre del color de la longitud mas larga
        print ("Color detected")
        print colorDetected

        if colorDetected == l1:
          tts.say("Muy bien, si es un arbol")
          repeatBrown=0

        else:
          tts.say("Ups... vuelve a intentar")
          repeatBrown=1
      '''

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
