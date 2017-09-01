from naoqi import ALProxy
IP = "192.168.0.100"
PORT = 9559
# Create a proxy to ALMemory.
memProxy = ALProxy("ALMemory", IP, PORT)
# Get data from landmark detection (assuming landmark detection has been activated).
data = memProxy.getData("LandmarkDetected")