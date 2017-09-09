
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
###----  movAnimo
def mov1(gVars):
  names = list()
  times = list()
  keys = list()

  names.append("HeadPitch")
  times.append([0.6, 1.4, 2.44])
  keys.append([-0.159578, -0.159578, -0.159578])

  names.append("HeadYaw")
  times.append([0.6, 1.4, 2.44])
  keys.append([0.0367741, 0.0367741, 0.0367741])

  names.append("LAnklePitch")
  times.append([0.6, 1.4, 2.44])
  keys.append([0.0858622, 0.0858622, 0.0858622])

  names.append("LAnkleRoll")
  times.append([0.6, 1.4, 2.44])
  keys.append([-0.130348, -0.130348, -0.130348])

  names.append("LElbowRoll")
  times.append([0.6, 1.4, 2.44])
  keys.append([-0.391128, -0.391128, -0.352778])

  names.append("LElbowYaw")
  times.append([0.6, 1.4, 2.44])
  keys.append([-1.30548, -1.33769, -1.3607])

  names.append("LHand")
  times.append([0.6, 1.4, 2.44])
  keys.append([0.294, 0.294, 0.294])

  names.append("LHipPitch")
  times.append([0.6, 1.4, 2.44])
  keys.append([0.128898, 0.128898, 0.128898])

  names.append("LHipRoll")
  times.append([0.6, 1.4, 2.44])
  keys.append([0.0982179, 0.0982179, 0.0982179])

  names.append("LHipYawPitch")
  times.append([0.6, 1.4, 2.44])
  keys.append([-0.170232, -0.170232, -0.170232])

  names.append("LKneePitch")
  times.append([0.6, 1.4, 2.44])
  keys.append([-0.0844118, -0.0844118, -0.0844118])

  names.append("LShoulderPitch")
  times.append([0.6, 1.4, 2.44])
  keys.append([0.658045, 0.105804, 1.35448])

  names.append("LShoulderRoll")
  times.append([0.6, 1.4, 2.44])
  keys.append([-0.0276539, -0.04913, -0.00157595])

  names.append("LWristYaw")
  times.append([0.6, 1.4, 2.44])
  keys.append([0.101202, 0.101202, 0.101202])

  names.append("RAnklePitch")
  times.append([0.6, 1.4, 2.44])
  keys.append([0.090548, 0.090548, 0.090548])

  names.append("RAnkleRoll")
  times.append([0.6, 1.4, 2.44])
  keys.append([0.130432, 0.130432, 0.130432])

  names.append("RElbowRoll")
  times.append([0.6, 1.4, 2.44])
  keys.append([0.507797, 0.507797, 0.283832])

  names.append("RElbowYaw")
  times.append([0.6, 1.4, 2.44])
  keys.append([1.3146, 1.3146, 1.3146])

  names.append("RHand")
  times.append([0.6, 1.4, 2.44])
  keys.append([0.2916, 0.2916, 0.2916])

  names.append("RHipPitch")
  times.append([0.6, 1.4, 2.44])
  keys.append([0.130348, 0.130348, 0.130348])

  names.append("RHipRoll")
  times.append([0.6, 1.4, 2.44])
  keys.append([-0.101202, -0.101202, -0.101202])

  names.append("RHipYawPitch")
  times.append([0.6, 1.4, 2.44])
  keys.append([-0.170232, -0.170232, -0.170232])

  names.append("RKneePitch")
  times.append([0.6, 1.4, 2.44])
  keys.append([-0.0843279, -0.0843279, -0.0843279])

  names.append("RShoulderPitch")
  times.append([0.6, 1.4, 2.44])
  keys.append([0.748635, 0.188724, 1.34076])

  names.append("RShoulderRoll")
  times.append([0.6, 1.4, 2.44])
  keys.append([0.056716, 0.113474, 0.05825])

  names.append("RWristYaw")
  times.append([0.6, 1.4, 2.44])
  keys.append([0.00609397, 0.00609397, 0.00609397])

  try:
    # uncomment the following line and modify the IP if you use this script outside Choregraphe.
    # motion = ALProxy("ALMotion", IP, 9559)
    gVars.motion.angleInterpolation(names, keys, times, True)
  
  except BaseException, err:
    print err
###----  movAnimo

def mov2(gVars):
  names = list()
  times = list()
  keys = list()

  names.append("HeadPitch")
  times.append([1.04, 1.56, 3.12])
  keys.append([-0.161112, -0.161112, -0.153442])

  names.append("HeadYaw")
  times.append([1.04, 1.56, 3.12])
  keys.append([0.0106959, 0.0106959, 0.00762796])

  names.append("LAnklePitch")
  times.append([1.04, 1.56, 3.12])
  keys.append([0.091998, 0.091998, 0.0873961])

  names.append("LAnkleRoll")
  times.append([1.04, 1.56, 3.12])
  keys.append([-0.130348, -0.130348, -0.12728])

  names.append("LElbowRoll")
  times.append([1.04, 1.56, 3.12])
  keys.append([-1.05995, -0.852862, -0.472429])

  names.append("LElbowYaw")
  times.append([1.04, 1.56, 3.12])
  keys.append([-1.30394, -1.29167, -1.29627])

  names.append("LHand")
  times.append([1.04, 1.56, 3.12])
  keys.append([0.2864, 0.2864, 0.2928])

  names.append("LHipPitch")
  times.append([1.04, 1.56, 3.12])
  keys.append([0.130432, 0.130432, 0.130432])

  names.append("LHipRoll")
  times.append([1.04, 1.56, 3.12])
  keys.append([0.092082, 0.092082, 0.090548])

  names.append("LHipYawPitch")
  times.append([1.04, 1.56, 3.12])
  keys.append([-0.164096, -0.164096, -0.159494])

  names.append("LKneePitch")
  times.append([1.04, 1.56, 3.12])
  keys.append([-0.0859461, -0.0859461, -0.0923279])

  names.append("LShoulderPitch")
  times.append([1.04, 1.56, 3.12])
  keys.append([-0.18719, -0.921975, 1.54776])

  names.append("LShoulderRoll")
  times.append([1.04, 1.56, 3.12])
  keys.append([0.248467, 0.202446, 0.0536479])

  names.append("LWristYaw")
  times.append([1.04, 1.56, 3.12])
  keys.append([-0.415757, -0.415757, -0.408086])

  names.append("RAnklePitch")
  times.append([1.04, 1.56, 3.12])
  keys.append([0.092082, 0.092082, 0.0890141])

  names.append("RAnkleRoll")
  times.append([1.04, 1.56, 3.12])
  keys.append([0.12583, 0.12583, 0.131966])

  names.append("RElbowRoll")
  times.append([1.04, 1.56, 3.12])
  keys.append([1.21344, 1.0539, 0.461776])

  names.append("RElbowYaw")
  times.append([1.04, 1.56, 3.12])
  keys.append([1.45726, 1.47106, 1.47567])

  names.append("RHand")
  times.append([1.04, 1.56, 3.12])
  keys.append([0.2868, 0.2868, 0.2892])

  names.append("RHipPitch")
  times.append([1.04, 1.56, 3.12])
  keys.append([0.128814, 0.128814, 0.128814])

  names.append("RHipRoll")
  times.append([1.04, 1.56, 3.12])
  keys.append([-0.101202, -0.101202, -0.0996681])

  names.append("RHipYawPitch")
  times.append([1.04, 1.56, 3.12])
  keys.append([-0.164096, -0.164096, -0.159494])

  names.append("RKneePitch")
  times.append([1.04, 1.56, 3.12])
  keys.append([-0.0904641, -0.0904641, -0.0858622])

  names.append("RShoulderPitch")
  times.append([1.04, 1.56, 3.12])
  keys.append([-0.108872, -0.820649, 1.442])

  names.append("RShoulderRoll")
  times.append([1.04, 1.56, 3.12])
  keys.append([-0.147306, -0.0890141, 0.0137641])

  names.append("RWristYaw")
  times.append([1.04, 1.56, 3.12])
  keys.append([0.0475121, 0.0475121, 0.056716])

  try:
      # uncomment the following line and modify the IP if you use this script outside Choregraphe.
      # motion = ALProxy("ALMotion", IP, 9559)
      gVars.motion.angleInterpolation(names, keys, times, True)
    
  except BaseException, err:
    print err

def mov3(gVars):
  names = list()
  times = list()
  keys = list()

  names.append("HeadPitch")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([-0.159578, -0.153442, -0.153442, -0.159578])

  names.append("HeadYaw")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([0.00762796, 0.00762796, 0.00762796, 0.00762796])

  names.append("LAnklePitch")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([0.093532, 0.0966001, 0.095066, 0.093532])

  names.append("LAnkleRoll")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([-0.124212, -0.12728, -0.12728, -0.124212])

  names.append("LElbowRoll")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([-0.423342, -1.3146, -1.53856, -0.423342])

  names.append("LElbowYaw")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([-1.17662, -1.1214, -0.734827, -1.17662])

  names.append("LHand")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([0.2912, 0.2928, 0.2928, 0.2912])

  names.append("LHipPitch")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([0.128898, 0.130432, 0.130432, 0.128898])

  names.append("LHipRoll")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([0.0982179, 0.090548, 0.090548, 0.0982179])

  names.append("LHipYawPitch")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([-0.162562, -0.167164, -0.167164, -0.162562])

  names.append("LKneePitch")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([-0.0859461, -0.0923279, -0.0923279, -0.0859461])

  names.append("LShoulderPitch")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([1.4726, 1.7932, 0.395731, 1.4726])

  names.append("LShoulderRoll")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([0.148756, 0.766959, 0.0720561, 0.148756])

  names.append("LWristYaw")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([0.075124, -0.0583338, -0.0859461, 0.075124])

  names.append("RAnklePitch")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([0.090548, 0.0874801, 0.0874801, 0.090548])

  names.append("RAnkleRoll")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([0.1335, 0.131966, 0.131966, 0.1335])

  names.append("RElbowRoll")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([0.426494, 0.472515, 0.472515, 0.426494])

  names.append("RElbowYaw")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([1.20415, 1.468, 1.468, 1.20415])

  names.append("RHand")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([0.286, 0.2932, 0.2932, 0.286])

  names.append("RHipPitch")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([0.124212, 0.130348, 0.130348, 0.124212])

  names.append("RHipRoll")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([-0.101202, -0.0996681, -0.0996681, -0.101202])

  names.append("RHipYawPitch")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([-0.162562, -0.167164, -0.167164, -0.162562])

  names.append("RKneePitch")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([-0.0843279, -0.0889301, -0.0889301, -0.0843279])

  names.append("RShoulderPitch")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([1.46348, 1.5233, 1.5233, 1.46348])

  names.append("RShoulderRoll")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([-0.15651, 0.0152981, 0.0260359, -0.15651])

  names.append("RWristYaw")
  times.append([0.16, 0.72, 1.32, 2])
  keys.append([0.0858622, 0.0551819, 0.0551819, 0.0858622])

  try:
    # uncomment the following line and modify the IP if you use this script outside Choregraphe.
    # motion = ALProxy("ALMotion", IP, 9559)
    gVars.motion.angleInterpolation(names, keys, times, True)
  except BaseException, err:
    print err


####---------------------####
#       MAIN ROUTINE  #
####---------------------####
def mainRoutine():
    # Greetings
    tts = ALProxy('ALTextToSpeech')
    tts.say(initialSentence)

    #Initialize global variables
    gVars = globalVariables(nao_ip)

    #Nao Initial posture
    gVars.posture.goToPosture("Stand",0.5) 

    mov1(gVars)


def Classification():
  gVars = globalVariables(nao_ip)
  mov2(gVars)
  mov3(gVars)


def RobotSleep():
  gVars = globalVariables(nao_ip)
  # Go to rest position
  gVars.motion.rest()

    
      
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

        memory.subscribeToEvent('FrontTactilTouched',
            'ReactToTouch',
            'onTouchedClassify')

    def onTouchedClassify(self, strVarName, value):

      # Subscribe again to the event (RearTactilTouched)
      memory.unsubscribeToEvent('FrontTactilTouched','ReactToTouch')
      Classification()

      memory.subscribeToEvent('RearTactilTouched','ReactToTouch','onTouchedEnd')

    def onTouchedEnd(self, strVarName, value):
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
