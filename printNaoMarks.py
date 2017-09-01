import time
from naoqi import ALProxy

IP = "169.254.129.162"  # Replace here with your NaoQi's IP address.
PORT = 9559

naoMarkRed= 64

# Create a proxy to ALLandMarkDetection
try:
  landMarkProxy = ALProxy("ALLandMarkDetection", IP, PORT)
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
  memoryProxy = ALProxy("ALMemory", IP, PORT)
except Exception, e:
  print "Error when creating memory proxy:"
  print str(e)
  exit(1)
  
  global naoMarkDetected 
  #naoMarkDetected= 64
  print "Nao Mark detected:"
  print naoMarkDetected


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

        #naoMarkDetected= markExtraInfo[0]
        # Second Field = Extra info (ie, mark ID).
        markExtraInfo = markInfo[1]
        print "mark  ID: %d" % (markExtraInfo[0])

    except Exception, e:
      print "Error msg %s" % (str(e))
  else:
      print "No landmark detected"


# Unsubscribe the module.
landMarkProxy.unsubscribe("Test_LandMark")

print "Test terminated successfully."