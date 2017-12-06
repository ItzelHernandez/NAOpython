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

import motion
import time
import almath
import math
from math import exp,pow,fabs,cos,sin,sqrt

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
    Hola! Me llamo NAO y vamos a jugar
"""
#, Estan listos?


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
  #for i in range(0, 10):
  for i in range(0, 3):
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



###----  movFracaso
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
###----  movFracaso

###----  movÉxito
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


###----  movÁnimo
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


def mov4 (gVars):
  names = list()
  times = list()
  keys = list()

  names.append("HeadPitch")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([-0.159578, -0.170316, -0.170316, -0.159578])

  names.append("HeadYaw")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([0.00762796, 0.00609398, 0.00609398, 0.00762796])

  names.append("LAnklePitch")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([0.093532, 0.0950661, 0.0950661, 0.093532])

  names.append("LAnkleRoll")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([-0.124212, -0.128814, -0.128814, -0.124212])

  names.append("LElbowRoll")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([-0.423342, -0.739346, -0.739346, -0.423342])

  names.append("LElbowYaw")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([-1.17662, -1.13827, -1.13827, -1.17662])

  names.append("LHand")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([0.2912, 0.2804, 0.2804, 0.2912])

  names.append("LHipPitch")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([0.128898, 0.130432, 0.130432, 0.128898])

  names.append("LHipRoll")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([0.0982179, 0.092082, 0.092082, 0.0982179])

  names.append("LHipYawPitch")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([-0.162562, -0.170232, -0.170232, -0.162562])

  names.append("LKneePitch")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([-0.0859461, -0.0828779, -0.0828779, -0.0859461])

  names.append("LShoulderPitch")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([1.4726, 1.44499, 1.44499, 1.4726])

  names.append("LShoulderRoll")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([0.148756, 0.187106, 0.187106, 0.148756])

  names.append("LWristYaw")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([0.075124, 0.35738, 0.35738, 0.075124])

  names.append("RAnklePitch")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([0.090548, 0.0890141, 0.0890141, 0.090548])

  names.append("RAnkleRoll")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([0.1335, 0.138102, 0.138102, 0.1335])

  names.append("RElbowRoll")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([0.426494, 1.29167, 1.49109, 0.426494])

  names.append("RElbowYaw")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([1.20415, 1.11824, 0.719404, 1.20415])

  names.append("RHand")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([0.286, 0.29, 0.29, 0.286])

  names.append("RHipPitch")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([0.124212, 0.122678, 0.122678, 0.124212])

  names.append("RHipRoll")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([-0.101202, -0.0950661, -0.0950661, -0.101202])

  names.append("RHipYawPitch")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([-0.162562, -0.170232, -0.170232, -0.162562])

  names.append("RKneePitch")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([-0.0843279, -0.0873961, -0.0873961, -0.0843279])

  names.append("RShoulderPitch")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([1.46348, 1.78102, 0.64739, 1.46348])

  names.append("RShoulderRoll")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([-0.15651, -0.727158, -0.092082, -0.15651])

  names.append("RWristYaw")
  times.append([0.16, 0.92, 1.48, 2.6])
  keys.append([0.0858622, 0.0889301, 0.105804, 0.0858622])

  try:
    # uncomment the following line and modify the IP if you use this script outside Choregraphe.
    # motion = ALProxy("ALMotion", IP, 9559)
    gVars.motion.angleInterpolation(names, keys, times, True)
  except BaseException, err:
    print err


def goodbye(gVars):
	names = list()
	times = list()
	keys = list()

	names.append("HeadPitch")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([-0.177986, -0.177986, -0.177986, -0.177986, -0.177986, -0.177986, -0.177986, -0.177986])

	names.append("HeadYaw")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([0, 0, 0, 0, 0, 0, 0, 0])

	names.append("LAnklePitch")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([0.0858622, 0.0858622, 0.0858622, 0.0858622, 0.0858622, 0.0858622, 0.0858622, 0.0858622])

	names.append("LAnkleRoll")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([-0.128814, -0.128814, -0.128814, -0.128814, -0.128814, -0.128814, -0.128814, -0.128814])

	names.append("LElbowRoll")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([-0.42641, -0.454022, -0.7869, -0.7869, -0.7869, -0.7869, -0.322098, -0.322098])

	names.append("LElbowYaw")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([-1.1981, -1.20883, -1.70585, -1.33769, -0.773178, -1.88226, -1.21037, -1.21037])

	names.append("LHand")
	times.append([0.24, 0.76, 1.2, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([0.294, 0.294, 1, 0.974, 0.974, 0.974, 0.974, 0.0268, 0.0268])

	names.append("LHipPitch")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([0.128898, 0.128898, 0.128898, 0.128898, 0.128898, 0.128898, 0.128898, 0.128898])

	names.append("LHipRoll")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([0.10282, 0.10282, 0.10282, 0.10282, 0.10282, 0.10282, 0.10282, 0.10282])

	names.append("LHipYawPitch")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([-0.168698, -0.168698, -0.168698, -0.168698, -0.168698, -0.168698, -0.168698, -0.168698])

	names.append("LKneePitch")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([-0.0874801, -0.0874801, -0.0874801, -0.0874801, -0.0874801, -0.0874801, -0.0874801, -0.0874801])

	names.append("LShoulderPitch")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([0.866668, -0.0261199, -0.564554, -0.54768, -0.523136, -0.533874, 0.908086, 1.39897])

	names.append("LShoulderRoll")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([0.029104, 0.02757, 0.305224, 0.0137641, -0.067538, 0.308292, 0.00149202, -0.024586])

	names.append("LWristYaw")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([0.08126, 0.08126, 1.1796, 1.1796, 1.1796, 1.1796, -0.0521979, -0.0521979])

	names.append("RAnklePitch")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([0.0874801, 0.0874801, 0.0859461, 0.0859461, 0.0859461, 0.0859461, 0.0859461, 0.0859461])

	names.append("RAnkleRoll")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([0.130432, 0.130432, 0.130432, 0.130432, 0.130432, 0.130432, 0.130432, 0.130432])

	names.append("RElbowRoll")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([0.423426, 0.423426, 0.423426, 0.423426, 0.423426, 0.423426, 0.423426, 0.423426])

	names.append("RElbowYaw")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([1.19494, 1.19494, 1.19494, 1.19494, 1.19494, 1.19494, 1.19494, 1.19494])

	names.append("RHand")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([0.284, 0.284, 0.284, 0.284, 0.284, 0.284, 0.284, 0.284])

	names.append("RHipPitch")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([0.128814, 0.128814, 0.128814, 0.128814, 0.128814, 0.128814, 0.128814, 0.128814])

	names.append("RHipRoll")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([-0.10427, -0.10427, -0.10427, -0.10427, -0.10427, -0.10427, -0.10427, -0.10427])

	names.append("RHipYawPitch")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([-0.168698, -0.168698, -0.168698, -0.168698, -0.168698, -0.168698, -0.168698, -0.168698])

	names.append("RKneePitch")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([-0.0889301, -0.0889301, -0.0889301, -0.0889301, -0.0889301, -0.0889301, -0.0889301, -0.0889301])

	names.append("RShoulderPitch")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([1.47728, 1.47728, 1.46194, 1.46194, 1.46194, 1.46194, 1.46194, 1.46194])

	names.append("RShoulderRoll")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([-0.066004, -0.066004, -0.107422, -0.107422, -0.107422, -0.107422, -0.107422, -0.107422])

	names.append("RWristYaw")
	times.append([0.24, 0.76, 1.84, 2.6, 3.48, 4.28, 5.28, 6.36])
	keys.append([0.0919981, 0.0919981, 0.0919981, 0.0919981, 0.0919981, 0.0919981, 0.0919981, 0.0919981])

	try:
	  gVars.motion.angleInterpolation(names, keys, times, True)
	except BaseException, err:
	  print err


# Vision --------------------------
trial=0 #intentos para buscar carta 

# def contoursFilter():

#   ##-----Read Mask--------------------##
#   img = cv2.imread('dilation3.png',0)
#   ##-----Threshold Filter-------------##
#   ret,thresh = cv2.threshold(img,127,255,0)
#   ##-----Find contours-------------##
#   contours,hierarchy = cv2.findContours(thresh, 1, 2)

#   return contours

# def redFilter(hsv):
#     lower_range = np.array([0, 50, 50], dtype=np.uint8) #red color
#     upper_range = np.array([10, 255, 255], dtype=np.uint8)

#     mask = cv2.inRange(hsv, lower_range, upper_range)

#     #Remove noise of the selected mask
#     kernel = np.ones((5,5),np.uint8)
#     erosion = cv2.erode(mask, kernel, iterations=1)
#     erosion2 = cv2.erode(erosion, kernel, iterations=1)
#     erosion3 = cv2.erode(erosion2, kernel, iterations=1)
#     dilation = cv2.dilate(erosion3,kernel, iterations =1)
#     dilation2 = cv2.dilate(dilation,kernel, iterations =1) 
#     dilation3 = cv2.dilate(dilation2,kernel, iterations =1)

#     #cv2.imshow('dilation3',dilation3)
#     cv2.imwrite('dilation3.png', dilation3)

#     contRed= contoursFilter()
#     lenContRed= len(contRed)

#     return lenContRed


# def brownFilter(hsv):
#   lower_range = np.array([20, 50, 50], dtype=np.uint8) 
#   upper_range = np.array([40, 255, 255], dtype=np.uint8)

#   mask = cv2.inRange(hsv, lower_range, upper_range)

#   #Remove noise of the selected mask
#   kernel = np.ones((5,5),np.uint8)
#   erosion = cv2.erode(mask, kernel, iterations=1)
#   erosion2 = cv2.erode(erosion, kernel, iterations=1)
#   erosion3 = cv2.erode(erosion2, kernel, iterations=1)
#   dilation = cv2.dilate(erosion3,kernel, iterations =1)
#   dilation2 = cv2.dilate(dilation,kernel, iterations =1) 
#   dilation3 = cv2.dilate(dilation2,kernel, iterations =1)

#   dilation3Brown = dilation3
#   # cv2.imshow('dilation3Brown',dilation3Brown)
#   cv2.imwrite('dilation3.png', dilation3)

#   contBrown= contoursFilter()
  
#   lenContBrown= len(contBrown)

#   return lenContBrown


# def whiteFilter(hsv):
#   lower_range = np.array([0, 0, 140], dtype=np.uint8) #red color
#   upper_range = np.array([0, 255, 255], dtype=np.uint8)

#   mask = cv2.inRange(hsv, lower_range, upper_range)

#   #Remove noise of the selected mask
#   kernel = np.ones((5,5),np.uint8)
#   erosion = cv2.erode(mask, kernel, iterations=1)
#   erosion2 = cv2.erode(erosion, kernel, iterations=1)
#   erosion3 = cv2.erode(erosion2, kernel, iterations=1)
#   dilation = cv2.dilate(erosion3,kernel, iterations =1)
#   dilation2 = cv2.dilate(dilation,kernel, iterations =1) 
#   dilation3 = cv2.dilate(dilation2,kernel, iterations =1)

#   dilation3White = dilation3
#   # cv2.imshow('dilation3White',dilation3White)
#   cv2.imwrite('dilation3.png', dilation3)
  
#   contWhite= contoursFilter()

#   lenContWhite= len(contWhite)

#   return lenContWhite


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

    #Initialize global variables
    gVars = globalVariables(nao_ip)

    #Nao Initial posture
    gVars.posture.goToPosture("Stand",0.5) 

    # wait for answer
    # ---------- ------------------ ----------------- #
    # ---------- Vision Recognition ----------------- #
    #tts.say("Hola Seño")
    differentFruit = 0
    state = 0
    fraseIncorrecta = 0
    fraseAlentadora = 0
    timer = 0
    waitTimer = 2
    #waitTimer = 3
    previousFruit = 0
    
    while(True):
      timer += 1
      # photoCP = ALProxy('ALPhotoCapture')
      # photoCP.setResolution(2)
      # photoCP.setPictureFormat('jpg')
      # photoCP.takePictures(5,'/home/nao/pythonProjects', 'nao')
      # image=cv2.imread('nao_4.jpg')  #take the last image (the good one)
      # height, width = image.shape[:2]
      # image2  = image.copy()
      # image3  = np.zeros((height,width,3), np.uint8)        
      # gray    = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
      # thresh  = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
      # kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 5))
      # thresh  = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
      # blurred = cv2.GaussianBlur(thresh, (5, 5), 0)
      # edged   = cv2.Canny(blurred, 50, 200, 255)
      # contours,hierarchy = cv2.findContours(edged,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
      print timer
      alentar = 0
      incorrecto = 0
      
      if(state == 0):
        tts.say("Vamos a empezar")
        sleep(1)
        tts.say("Estás listo?")
        sleep(1)
        tts.say("Muestra la carta de una manzana")
        state = 1
      elif(state == 1):
        if(differentFruit == 0 and timer > waitTimer):
          timer = 0
          alentar = 1
          incorrecto = 0
        elif(differentFruit == 2):
          timer = 0
          mov2(gVars)
          tts.say("Muy bien!, eso es una manzana")
          mov2(gVars)
          sleep(0.5)
          tts.say("Lo hiciste increíble")
          sleep(1)
          previousFruit = differentFruit
          differentFruit = 0
          state = 2
        elif(differentFruit != 2 and differentFruit != 0):
          if(previousFruit != differentFruit):
          	incorrecto = 1
          	alentar = 0
          	previousFruit = differentFruit
          timer = 0
          differentFruit = 0

      elif(state == 2):
        tts.say("Ahora muestra la carta de una piña")
        sleep(1)
        state = 3
      elif(state == 3):
        if(differentFruit == 0 and timer > waitTimer):
          timer = 0
          alentar = 1
          incorrecto = 0
        elif(differentFruit == 4):
          timer = 0
          mov2(gVars)
          tts.say("Correcto!, eso es una piña")
          mov2(gVars)
          sleep(0.5)
          tts.say("Estás haciendo muy bien!")
          previousFruit = differentFruit
          differentFruit = 0
          state = 4
        elif(differentFruit != 4 and differentFruit != 0):
          if(previousFruit != differentFruit):
          	incorrecto = 1
          	alentar = 0
          	previousFruit = differentFruit
          timer = 0
          differentFruit = 0

      elif(state == 4):
        tts.say("Esta vez, muestra la carta de un limón")
        sleep(1)
        state = 5
      elif(state == 5):
        if(differentFruit == 0 and timer > waitTimer):
          timer = 0
          alentar = 1
          incorrecto = 0
        elif(differentFruit == 3):
          timer = 0
          mov2(gVars)
          tts.say("Muy bien!, eso es un limón")
          mov2(gVars)
          sleep(0.5)
          tts.say("Síguele así. Estás haciendo fantástico!")
          previousFruit = differentFruit
          differentFruit = 0
          state = 6
        elif(differentFruit != 3 and differentFruit != 0):
          if(previousFruit != differentFruit):
          	incorrecto = 1
          	alentar = 0
          	previousFruit = differentFruit
          timer = 0
          differentFruit = 0

      elif(state == 6):
        tts.say("Por último, muestra la carta de una sandía")
        sleep(1)
        state = 7
      elif(state == 7):
        if(differentFruit == 0 and timer > waitTimer):
          timer = 0
          alentar = 1
          incorrecto = 0
        elif(differentFruit == 1):
          timer = 0
          mov2(gVars)
          tts.say("Correcto!, eso es una sandía")
          mov2(gVars)
          sleep(0.5)
          previousFruit = differentFruit
          differentFruit = 0
          state = 8
        elif(differentFruit != 1 and differentFruit != 0):
          if(previousFruit != differentFruit):
          	incorrecto = 1
          	alentar = 0
          	previousFruit = differentFruit
          timer = 0
          differentFruit = 0
            
      elif(state == 8): 
        tts.say("Excelente, has mostrado todas las cartas correctas")
        sleep(1)
        tts.say("Estuviste increíble")
        sleep(1)
        goodbye(gVars)
        tts.say("Nos vemos en la próxima vez")
        state = 9
      

      if(alentar == 1 and incorrecto == 0):
        if(fraseAlentadora == 0):
          tts.say("Vamos, tú puedes")
          mov3(gVars)
          sleep(1)
          fraseAlentadora = 1
        elif(fraseAlentadora == 1):
          tts.say("Concéntrate")
          mov4(gVars)
          sleep(1)
          fraseAlentadora = 2
        elif(fraseAlentadora == 2):
          tts.say("Échale ganas")
          mov3(gVars)
          sleep(1)
          fraseAlentadora = 3 
        elif(fraseAlentadora == 3):
          tts.say("Ánimo")
          mov4(gVars)
          sleep(1)
          fraseAlentadora = 0


      elif(alentar == 0 and incorrecto == 1):
        if(fraseIncorrecta == 0):
          tts.say("Yo sé que tú puedes")
          mov1(gVars)
          tts.say("Prueba de nuevo")
          fraseIncorrecta = 1
        elif(fraseIncorrecta == 1):
          tts.say("Casi lo logras")
          mov1(gVars)
          tts.say("Intenta de nuevo")
          fraseIncorrecta = 2
        elif(fraseIncorrecta == 2):
          tts.say("Buen intento")
          mov1(gVars)
          tts.say("Prueba de nuevo")
          fraseIncorrecta = 3
        elif(fraseIncorrecta == 3):
          tts.say("Casi lo tienes")
          mov1(gVars)
          tts.say("Intenta de nuevo")
          fraseIncorrecta = 0

      # p = 0
      # validArea = 0
      # biggestArea = 0;
      # biggestContour = 0;
      # for cnt in contours:
      #   approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True);
      #   if len(approx) == 4 or len(approx) == 5: 
      #     pt = contours[p]                   
      #     if(cv2.contourArea(pt) > biggestArea):
      #       biggestContour = p
      #       biggestPoint = pt
      #       biggestArea = cv2.contourArea(pt)
      #     if(biggestArea > (8000)):
      #       validArea = 1

      #   p = p + 1
      # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  
      
      # red = green = blue = 0
      #counter = 0
      # for x in biggestPoint:
      #   rangePixels = 3
      #   xCoord = x[0][0]
      #   yCoord = x[0][1]

      #   for i in range(-1*rangePixels + xCoord ,rangePixels + xCoord ):
      #     for j in range(-1*rangePixels + yCoord,rangePixels + yCoord):
      #       if(i < width and i >= 0):
      #         if(j < height and j >= 0):
      #           bT = int(image2[j,i,0])
      #           gT = int(image2[j,i,1])
      #           rT = int(image2[j,i,2])

      #           color     = int(hsv[j,i,0])
      #           saturation  = int(hsv[j,i,1])
      #           value     = int(hsv[j,i,2])
                
      #           if(color < 250 and saturation > 80 and value > 80):
      #             blue += bT
      #             green += gT
      #             red += rT
      #             counter = counter + 1
      #             cv2.circle(image,(i,j), 2, (0,200,0), -1)

      #if counter != 0:
      #   blue = blue/(counter)
      #   green = green/(counter)
      #   red = red/(counter)

      #   if(validArea == 1):
          # print ("Red: %d Green: %d Blue: %d\n" %(red, green, blue))

      value= readNaoMark() #aqui es cuando ve primero la 80 (cafe)
      print "readNaoMark"
      print value

      if(value==68):
        print "Watermelon Detected\n"
        if(differentFruit != 1):
          #tts.say("Esa es una sandía")
          differentFruit = 1
      elif(value==80):
        print "Apple Detected\n"
        if(differentFruit != 2):
          #tts.say("Esa es una manzana")
          differentFruit = 2
      elif(value==112):
        print "Lemon Detected\n"
        if(differentFruit != 3):
          #tts.say("Eso es un limón")
          differentFruit = 3
      elif(value==114):
        print "Pineapple detected\n"
        if(differentFruit != 4):
          #tts.say("Esa es una piña")
          differentFruit = 4

    tts.say("Adios Kevin")
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

    global nao_ip
    nao_ip= args.ip

    main(args.ip, args.port)
