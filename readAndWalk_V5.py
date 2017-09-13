
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
naoMarkValue=0


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

    #Paths
    LSONAR = "Device/SubDeviceList/US/Left/Sensor/Value"
    RSONAR = "Device/SubDeviceList/US/Right/Sensor/Value"
    ANGLEZ = "Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value" 

    #Step 11
    walkParameterFixedRotation = [ ["MaxStepX", 0.08],["MaxStepY", 0.14] ,["MaxStepTheta", 0.1963],["MaxStepFrequency", 0.5],["StepHeight", 0.04],["TorsoWx", 0.0],["TorsoWy", 0.0]]
    
####---------------------

ReactToTouch = None
memory = None
flag = False
initialSentence = """ 
    Hola! 
"""

# Stiffness ON RightArm ----------
def StiffnessOnRArm(gVars):
    names = "RShoulderPitch"
    #names2 = "RShoulderRoll"

    stiffnessLists = 0.0
    timeLists = 1.0
    gVars.motion.stiffnessInterpolation(names, stiffnessLists, timeLists)
    #gVars.motion.stiffnessInterpolation(names2, stiffnessLists, timeLists)

    # print motion state
    #print motionProxy.getSummary()

# Stiffness ON RightArm ----------


#CanastaOpen ---

def OpenHandBasket(gVars):
  names = list()
  times = list()
  keys = list()

  names.append("LElbowRoll")
  times.append([0.24])
  keys.append([-1.1612])

  names.append("LElbowYaw")
  times.append([0.24])
  keys.append([-1.66903])

  names.append("LHand")
  times.append([0.24])
  keys.append([0.7524])

  names.append("LShoulderPitch")
  times.append([0.24])
  keys.append([1.21335])

  names.append("LShoulderRoll")
  times.append([0.24])
  keys.append([-0.069072])

  names.append("LWristYaw")
  times.append([0.24])
  keys.append([-1.24718])

  names.append("RElbowRoll")
  times.append([0.24])
  keys.append([1.00788])

  names.append("RElbowYaw")
  times.append([0.24])
  keys.append([0.849794])

  names.append("RHand")
  times.append([0.24])
  keys.append([0.0724])

  names.append("RShoulderPitch")
  times.append([0.24])
  keys.append([1.69971])

  names.append("RShoulderRoll")
  times.append([0.24])
  keys.append([-0.251618])

  names.append("RWristYaw")
  times.append([0.24])
  keys.append([-0.105888])

  try:
    # uncomment the following line and modify the IP if you use this script outside Choregraphe.
    # motion = ALProxy("ALMotion", IP, 9559)
    gVars.motion.angleInterpolation(names, keys, times, True)
  except BaseException, err:
    print err
#CanastaOpen ---


# CanastaClose ---
def BasketClose(gVars):
  names = list()
  times = list()
  keys = list()

  names.append("HeadPitch")
  times.append([0.24])
  keys.append([-0.122762])

  names.append("HeadYaw")
  times.append([0.24])
  keys.append([0.0122299])

  names.append("LAnklePitch")
  times.append([0.24])
  keys.append([-0.352862])

  names.append("LAnkleRoll")
  times.append([0.24])
  keys.append([-0.0122299])

  names.append("LElbowRoll")
  times.append([0.24])
  keys.append([-0.835988])

  names.append("LElbowYaw")
  times.append([0.24])
  keys.append([-1.40825])

  names.append("LHand")
  times.append([0.24])
  keys.append([0.0196])

  names.append("LHipPitch")
  times.append([0.24])
  keys.append([-0.450954])

  names.append("LHipRoll")
  times.append([0.24])
  keys.append([0.0107799])

  names.append("LHipYawPitch")
  times.append([0.24])
  keys.append([-0.00609398])

  names.append("LKneePitch")
  times.append([0.24])
  keys.append([0.707132])

  names.append("LShoulderPitch")
  times.append([0.24])
  keys.append([0.94797])

  names.append("LShoulderRoll")
  times.append([0.24])
  keys.append([0.026036])

  names.append("LWristYaw")
  times.append([0.24])
  keys.append([-1.76721])

  names.append("RAnklePitch")
  times.append([0.24])
  keys.append([-0.35738])

  names.append("RAnkleRoll")
  times.append([0.24])
  keys.append([0.0107799])

  names.append("RElbowRoll")
  times.append([0.24])
  keys.append([0.435698])

  names.append("RElbowYaw")
  times.append([0.24])
  keys.append([1.19955])

  names.append("RHand")
  times.append([0.24])
  keys.append([0.284])

  names.append("RHipPitch")
  times.append([0.24])
  keys.append([-0.457174])

  names.append("RHipRoll")
  times.append([0.24])
  keys.append([-0.00455999])

  names.append("RHipYawPitch")
  times.append([0.24])
  keys.append([-0.00609398])

  names.append("RKneePitch")
  times.append([0.24])
  keys.append([0.704148])

  names.append("RShoulderPitch")
  times.append([0.24])
  keys.append([1.42666])

  names.append("RShoulderRoll")
  times.append([0.24])
  keys.append([-0.0506639])

  names.append("RWristYaw")
  times.append([0.24])
  keys.append([0.187106])

  try:
    gVars.motion.angleInterpolation(names, keys, times, True)
  except BaseException, err:
    print err

# CanastaClose ---



# NaoMotionBasket ------

def LeaveBasket(gVars):
  names = list()
  times = list()
  keys = list()

  names.append("HeadPitch")
  times.append([0.24, 0.68, 1.16])
  keys.append([-0.122762, -0.122762, -0.122762])

  names.append("HeadYaw")
  times.append([0.24, 0.68, 1.16])
  keys.append([0.0122299, 0.0122299, 0.0122299])

  names.append("LAnklePitch")
  times.append([0.24, 0.68, 1.16])
  keys.append([-0.352862, -0.352862, -0.352862])

  names.append("LAnkleRoll")
  times.append([0.24, 0.68, 1.16])
  keys.append([-0.0122299, -0.0122299, -0.0122299])

  names.append("LElbowRoll")
  times.append([0.24, 0.68, 1.16])
  keys.append([-0.82525, -1.4097, -1.31306])

  names.append("LElbowYaw")
  times.append([0.24, 0.68, 1.16])
  keys.append([-1.40825, -1.34536, -1.28707])

  names.append("LHand")
  times.append([0.24, 0.68, 1.16])
  keys.append([0.0196, 0.0196, 0.0196])

  names.append("LHipPitch")
  times.append([0.24, 0.68, 1.16])
  keys.append([-0.450954, -0.450954, -0.450954])

  names.append("LHipRoll")
  times.append([0.24, 0.68, 1.16])
  keys.append([0.0107799, 0.0107799, 0.0107799])

  names.append("LHipYawPitch")
  times.append([0.24, 0.68, 1.16])
  keys.append([-0.00609398, -0.00609398, -0.00609398])

  names.append("LKneePitch")
  times.append([0.24, 0.68, 1.16])
  keys.append([0.707132, 0.707132, 0.707132])

  names.append("LShoulderPitch")
  times.append([0.24, 0.68, 1.16])
  keys.append([0.94797, 0.54913, 0.673384])

  names.append("LShoulderRoll")
  times.append([0.24, 0.68, 1.16])
  keys.append([0.026036, -0.0644701, -0.24855])

  names.append("LWristYaw")
  times.append([0.24, 0.68, 1.16])
  keys.append([-1.76721, -1.35917, 1.33914])

  names.append("RAnklePitch")
  times.append([0.24, 0.68, 1.16])
  keys.append([-0.35738, -0.35738, -0.35738])

  names.append("RAnkleRoll")
  times.append([0.24, 0.68, 1.16])
  keys.append([0.0107799, 0.0107799, 0.0107799])

  names.append("RElbowRoll")
  times.append([0.24, 0.68, 1.16])
  keys.append([0.435698, 0.435698, 0.435698])

  names.append("RElbowYaw")
  times.append([0.24, 0.68, 1.16])
  keys.append([1.19955, 1.19955, 1.19955])

  names.append("RHand")
  times.append([0.24, 0.68, 1.16])
  keys.append([0.284, 0.284, 0.284])

  names.append("RHipPitch")
  times.append([0.24, 0.68, 1.16])
  keys.append([-0.457174, -0.457174, -0.457174])

  names.append("RHipRoll")
  times.append([0.24, 0.68, 1.16])
  keys.append([-0.00455999, -0.00455999, -0.00455999])

  names.append("RHipYawPitch")
  times.append([0.24, 0.68, 1.16])
  keys.append([-0.00609398, -0.00609398, -0.00609398])

  names.append("RKneePitch")
  times.append([0.24, 0.68, 1.16])
  keys.append([0.704148, 0.704148, 0.704148])

  names.append("RShoulderPitch")
  times.append([0.24, 0.68, 1.16])
  keys.append([1.42666, 1.42666, 1.42666])

  names.append("RShoulderRoll")
  times.append([0.24, 0.68, 1.16])
  keys.append([-0.0506639, -0.0506639, -0.0506639])

  names.append("RWristYaw")
  times.append([0.24, 0.68, 1.16])
  keys.append([0.187106, 0.187106, 0.187106])

  try:
    gVars.motion.angleInterpolation(names, keys, times, True)
  except BaseException, err:
    print err

# NaoMotionBasket ------


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


# Vision --------------------------
def contoursFilter():

  ##-----Read Mask--------------------##
  img = cv2.imread('dilation3.png',0)
  ##-----Threshold Filter-------------##
  ret,thresh = cv2.threshold(img,127,255,0)
  ##-----Find contours-------------##
  contours,hierarchy = cv2.findContours(thresh, 1, 2)

  return contours

def redFilter(hsv):
    lower_range = np.array([0, 70, 70], dtype=np.uint8) #red color
    upper_range = np.array([3, 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_range, upper_range)

    #Remove noise of the selected mask
    kernel = np.ones((5,5),np.uint8)
    dilation3 = cv2.dilate(mask,kernel, iterations =1)

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
  kernel = np.ones((10,10),np.uint8)
  dilation3 = cv2.dilate(mask,kernel, iterations =1)

  dilation3Brown = dilation3
  # cv2.imshow('dilation3Brown',dilation3Brown)
  cv2.imwrite('dilation3.png', dilation3)

  contBrown= contoursFilter()
  
  lenContBrown= len(contBrown)

  return lenContBrown


def whiteFilter(hsv):
  lower_range = np.array([0, 0, 140], dtype=np.uint8) 
  upper_range = np.array([0, 255, 255], dtype=np.uint8)

  mask = cv2.inRange(hsv, lower_range, upper_range)

  #Remove noise of the selected mask
  kernel = np.ones((10,10),np.uint8)
  dilation3 = cv2.dilate(mask,kernel, iterations =1)

  dilation3White = dilation3
  # cv2.imshow('dilation3White',dilation3White)
  cv2.imwrite('dilation3.png', dilation3)
  
  contWhite= contoursFilter()

  lenContWhite= len(contWhite)

  return lenContWhite
# Vision --------------------------

def Recycle():
  
  #Initialize global variables
  gVars = globalVariables(nao_ip)

  BasketClose(gVars)
  StiffnessOnRArm(gVars)
  
  gVars.motion.moveTo(0, 0, (math.pi)-0.4) #gira 180

  # Check MoveFwd NaoMarks ----------
  positionInit =gVars.motion.getRobotPosition(False)
  print "Robot Move:", positionInit 
  
  initialAngle= gVars.memory.getData(gVars.ANGLEZ) #initialAngle
  #position = gVars.motion.getRobotPosition(False)
  #print "Robot Move:", position

  gVars.motion.moveTo(0.85, 0, 0) #gira 30
      
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

  # Check MoveFwd NaoMarks ----------

## Select Path -------------------------
  #naoMarkValue= 80
  #Read NaoMarks
  value= readNaoMark() #aqui es cuando ve primero la 80 (cafe)
  print value

  if value == naoMarkValue:
    print"este es el camino"
    gVars.motion.moveTo(0.50, 0, 0)
    LeaveBasket(gVars)

  else:
    gVars.motion.moveTo(0, 0, -(math.pi/6))#gira para valida la 108 (blanco)
    value= readNaoMark()
    print value

    if value == naoMarkValue:
      print"este es el camino"
      gVars.motion.moveTo(0.85, 0, 0)
      LeaveBasket(gVars)

    else: # esta es cuando debe de ir a al naoMark 64
      gVars.motion.moveTo(0, 0, (math.pi/5))
      gVars.motion.moveTo(0, 0, (math.pi/5))
      print" voy en camino" 
      gVars.motion.moveTo(0.80, 0, 0)
      LeaveBasket(gVars)



####---------------------####
#       MAIN ROUTINE  #
####---------------------####
def mainRoutine():

    #Initialize global variables
    gVars = globalVariables(nao_ip)
    StiffnessOnRArm(gVars)
    # Greetings
    tts = ALProxy('ALTextToSpeech')

    # ---------- Speech Recognition ----------------- #
    asr = ALProxy('ALSpeechRecognition')
    tempMem = ALProxy('ALMemory')
    asr.setLanguage('Spanish')
    tts.say(initialSentence)
    vocabulary = ['si', 'no', 'porfavor']

    # ---------- Vision Recognition ----------------- #
    photoCP = ALProxy('ALPhotoCapture')
    photoCP.setResolution(2)
    photoCP.setPictureFormat('jpg')
    photoCP.takePictures(5,'/home/nao/pythonProjects', 'nao')
    img=cv2.imread('nao_4.jpg')  #take the last image (the good one)
    #cv2.imshow('nao9',img)
    #hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    tts.say("Ahora empecemos")
    sleep(1)
    repeatRed=1
    
    photoCP.takePictures(5,'/home/nao/pythonProjects', 'nao')
    img=cv2.imread('nao_4.jpg') 
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    sleep(1)
    
    l0=redFilter(hsv)
    l1=brownFilter(hsv)
    l2=whiteFilter(hsv)  
      
    if l0 != 0 or l1 != 0 or l2 != 0:
      l=[l0,l1,l2] #pone en la ultima posicion el color mas grande leido
      l.sort()
      print("l array:")
      print(l);

      colorDetected= l[2] #poner el nombre del color de la longitud mas larga

      #naoMarkValue=0
      value= readNaoMark() #aqui es cuando ve primero la 80 (cafe)
      print value

      if colorDetected == l0 or value==107: #color rojo
        tts.say("Es una lata")
        global naoMarkValue
        naoMarkValue= 64
        print("Red detected")
        
      #elif colorDetected == l1 or value== 108 : #color cafe
      elif colorDetected == l1 or value== 108 : #color cafe
        tts.say("Es carton")
        global naoMarkValue
        naoMarkValue = 108
        print("Brown detected")

      elif colorDetected == l2 or value==80:#color blanco
        tts.say("Es plastico")
        global naoMarkValue
        naoMarkValue = 80
        print("White detected")   

      else:
        print("No color detected")

    # ---------- Vision Recognition ----------------- #  
    #Define NaoMarks
    #naoMarkValue=64
    #naoMarkDetected=0

    
    #Nao Initial posture
    gVars.posture.goToPosture("StandInit",0.5) 
    OpenHandBasket(gVars)

    
      
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

      memory.subscribeToEvent('FrontTactilTouched','ReactToTouch','onTouchedClassify')

    def onTouchedClassify(self, strVarName, value):

      # Subscribe again to the event (RearTactilTouched)
      memory.unsubscribeToEvent('FrontTactilTouched','ReactToTouch')
      
      Recycle()

      memory.subscribeToEvent('RearTactilTouched','ReactToTouch','onTouchedEnd')

    def onTouchedEnd(self, strVarName, value):
      memory.unsubscribeToEvent('RearTactilTouched','ReactToTouch')
      # global variable to let the application that the routine has ended
      RobotSleep()
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
