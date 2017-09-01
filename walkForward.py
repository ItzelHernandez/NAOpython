'''Walk: Small example to make Nao walk'''
import sys
import motion
import time
import almath
import networkx as nx
from math import exp,pow,fabs,cos,sin,sqrt
from scipy.integrate import quad
from scipy.integrate import dblquad
from naoqi import ALProxy
from collections import OrderedDict

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

    
    pi = 3.14159265359
    walkParameterFixedRotation      = [ ["MaxStepX", 0.08],["MaxStepY", 0.14] ,["MaxStepTheta", 0.1963],["MaxStepFrequency", 0.5],["StepHeight", 0.04],["TorsoWx", 0.0],["TorsoWy", 0.0]]
    walkParameterAngularAdjustment  = [ ["MaxStepX", 0.08],["MaxStepY", 0.14] ,["MaxStepTheta", 0.0175],["MaxStepFrequency", 0.5],["StepHeight", 0.04],["TorsoWx", 0.0],["TorsoWy", 0.0]]
    walkParameterSideAdjustment     = [ ["MaxStepX", 0.08],["MaxStepY", 0.101],["MaxStepTheta", 0.0175],["MaxStepFrequency", 0.5],["StepHeight", 0.04],["TorsoWx", 0.0],["TorsoWy", 0.0]]

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
    
    #SONAR
    THRESHOLDSONAR = 0.45
    FRONT = 0
    RIGHT = 1
    LEFT = 2
    BACK = 3

def getWalkCalibration(theta):
    if(theta < 0):
	theta*=-1
    if(theta > 0.349):
        theta /= 2
    return [ ["MaxStepX", 0.01],["MaxStepY", 0.101],["MaxStepTheta", theta],["MaxStepFrequency", 0.5],["StepHeight", 0.04],["TorsoWx", 0.0],["TorsoWy", 0.0]]

def StiffnessOff(gVars):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 0.0
    pTimeLists = 1.0
    gVars.motion.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

def StiffnessOn(gVars):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    gVars.motion.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

# Returns a projection vector relative to the FRAME_ROBOT axis depending on the next point 
# we want to reach command to send to move() and moveTo()
# Information to store into the edges!!!! 

def firstIteration(gVars):
    #Get position values for initial node
    #position = [x,y,absTh,relTh]
    angle = (gVars.memory.getData(gVars.ANGLEZ))
    position  = gVars.motion.getRobotPosition(False)
    position.append(angle)
    print "Initial position values: ", position
    
    # Walk 70 cm to the front on x
    # gVars.motion.moveTo(0.7,0,0)
    moveReallyForward(gVars, 0.7,angle)
    
    # Sets actual node number for loop 
    
def checkIfWall(gVars, targetAngle):
    #check if wall
    gVars.sonar.subscribe("UltraSonicSensors")
    for n in range(1,10):
        lsonar = gVars.memory.getData(gVars.LSONAR)
        rsonar = gVars.memory.getData(gVars.RSONAR)

    if(lsonar > 2.55): 
        lsonar = rsonar
    if(rsonar > 2.55): 
        rsonar = lsonar 

    averageSonar = (lsonar+rsonar)/2  #es para definir la distancia que hay con referencia a una pared
    print "SONARS: ", lsonar , rsonar, "average : ", averageSonar
    
    if(averageSonar > gVars.THRESHOLDSONAR):#THRESHOLDSONAR es para indicar que hay una pared
        #Add new node and corresponding edge with angles
        print "PATH CLEAR, Wall NOT Detected"

    else:
        print "Wall Detected"

    gVars.sonar.unsubscribe("UltraSonicSensors")


def referenceAngles(gVars, AngleRef):
    #Consider discontinuity of angles, to resolve
    iRelTheta = AngleRef
    leftAngle = iRelTheta-gVars.pi/2
    if leftAngle < -gVars.pi:         # to make sure that angle is in range (-pi, pi)
        leftAngle+= 2*gVars.pi
    rightAngle = iRelTheta+(gVars.pi/2)
    if rightAngle > gVars.pi:         # to make sure that angle is in range (-pi, pi)
        rightAngle -= 2*gVars.pi
    backAngle = iRelTheta+(gVars.pi)
    if backAngle > gVars.pi:          # to make sure that angle is in range (-pi, pi)
        backAngle -= 2*gVars.pi
        
    #print (iRelTheta , rightAngle, leftAngle,backAngle)
    
    angles = [iRelTheta,rightAngle,leftAngle,backAngle]
    return angles

# targetAngle is one of the actual node target angle 
# FRONT, LEFT, RIGHT or BACK
def angleDifference(gVars, targetAngle):
    actRelTheta = gVars.memory.getData(gVars.ANGLEZ)
    AngleDif = actRelTheta-targetAngle
    ##print AngleDif
    # To set Rotation sense
    # Check if correct 
    # n represents the sign of AngleDif
    if(AngleDif>0):
        n = 1
    else:
        n = -1  
    
    absAngleDif = fabs(AngleDif)
    #means that we will pass trough the discontinuity point
    if(absAngleDif>gVars.pi): # If the difference is bigger than pi,
        n = n*(-1)      # then we can move faster to the other direction
        absAngleDif = 2*gVars.pi-absAngleDif # to reach destination angle
        AngleDif = n*absAngleDif
        #print "discontinuity! ", AngleDif              
    
    return AngleDif

# Do the last final steps to adjust to the angle
# targetAngle is one of the actual node target angle 
# FRONT, LEFT, RIGHT or BACK
def adjustmentTheta2(gVars, targetAngle):
    #Get the actual value of theta
    #actRelTheta = gVars.memory.getData(gVars.ANGLEZ)
    difTheta = angleDifference(gVars,targetAngle)
    print "Angle Without adjustment : " , gVars.memory.getData(gVars.ANGLEZ) , "Target Angle : " , targetAngle , "Difference Theta : " , difTheta
    #Move the difference
    gVars.motion.moveTo(0,0,difTheta,getWalkCalibration(difTheta))
    

# Function using moveTo() command to adjust the direction while moving
# For now only adjust the robots orientation, has to be improved for x and y axis
# And used for movements in every directions.
def moveReallyForward(gVars, distanceX,next_angle):
    #Initialize sonars
    gVars.sonar.subscribe("UltraSonicSensors")
    lsonar = gVars.memory.getData(gVars.LSONAR)
    rsonar = gVars.memory.getData(gVars.RSONAR)
    averageSonar = (lsonar+rsonar)/2
    
    #Initial position 
    InitPosition  = gVars.motion.getRobotPosition(False)
    
    #Relative angle to do real time walking corrections 
    rInitAngle = gVars.memory.getData(gVars.ANGLEZ)
    moveMag = 0
    
    while(moveMag < distanceX and (averageSonar > 0.26) ):    
        #Check if nao is near the wall
        lsonar = gVars.memory.getData(gVars.LSONAR)
        rsonar = gVars.memory.getData(gVars.RSONAR)
        averageSonar = (lsonar+rsonar)/2
        
        #print "SONARS : ", lsonar , rsonar
        pos  = gVars.motion.getRobotPosition(False)
        rAngle = gVars.memory.getData(gVars.ANGLEZ)

        difRel = rAngle - rInitAngle 
    
        dX =(pos[gVars.X]-iX)
        dY =(pos[gVars.Y]-iY)
        dTheta = angleDifference(gVars,next_angle)
        dMag = sqrt(pow(dY,2)+pow(dX,2))

        moveX = (pos[gVars.X]-InitPosition[gVars.X])
        moveY = (pos[gVars.Y]-InitPosition[gVars.Y])
        moveMag = sqrt(pow(dY,2)+pow(dX,2))

        nX = dX - dMag*cos(dTheta) * uix
        nY = dY - dMag*cos(dTheta) * uiy
        magN = sqrt(pow(nX,2)+pow(nY,2)) 
        ##print cambio   #Debug Purposes
        if(magN > 0.025):
            vY = (magN / 0.4)*0.15
        else:
            vY = 0
        if(fabs(difRel ) > 0.0175 ):
            ##print ("If error > 0.0175, error = ", difRel)   #Debug purposes 

            #error / errorMax (0.25) ; errorMax = 4° 
            gVars.motion.move(0.06,vY,((difRel/0.1396)*0.25)/gVars.pi,gVars.walkParameterSideAdjustment)
        else:
            gVars.motion.move(0.06,vY,0,gVars.walkParameterSideAdjustment)

    #When the distance is reached the robot stops to move, small inertial (<1cm) error only       
    gVars.motion.stopMove()
    #Initialize 
    gVars.sonar.unsubscribe("UltraSonicSensors")


#-------------------------------------------------------------------------------------------------------------#
#                                           Main()                                                            #
#-------------------------------------------------------------------------------------------------------------#
def main(robotIP):
    
    # Init proxies.    
    gVars = globalVariables(robotIP)
    gVars.posture.goToPosture("StandInit",0.5)

    # Final sequence of instructions to make the programm work and do the depth exploration
    # Creation of Initial node and first node, move to the front of 0.7m and enters the 
    # labyrinth
    print "___________________________________________________________________________________________"
    firstIteration(gVars)
    print "End of First Iteration"
    print "___________________________________________________________________________________________"
    # Loop to execute continually until we reach the end of the labyrinth, 
    # (end condition still to be found)
    print "Starting the while Loop"
    while(True): 
        if gVars.gMap.node[gVars.actualNode]['pos'] == 'empty':
            print "Node Uknown explore the options"
            # --------------------------------------------------------- #
            #                     Explore actual node                   # 
            # --------------------------------------------------------- # 
            # Set absolute node x,y positions and angle FRONT reference
            print "___________________________________________________________________________________________"
            print "New node positioning "
            angle = (gVars.memory.getData(gVars.ANGLEZ))
            InitialAngle = InitialOrientationAngle(gVars, angle)
            position  = gVars.motion.getRobotPosition(False)
            position.append(InitialAngle)

            gVars.gMap.add_node(gVars.actualNode,pos = position)
            print "Position of the New Node : " , position
            print "Actual Node : " , gVars.actualNode
            nodeTargetAngles = referenceAngles(gVars, InitialAngle)
            
            print "Reference Angles [F,R,L,B]: ", nodeTargetAngles , "Initial Angle : ", angle 
            print "Initial Angle and Target angle [F] must be almost the same "
            print "___________________________________________________________________________________________"
            
            #nodeAbsTargetAngles = referenceAngles(gVars, position[gVars.ABSTHETA])
            #print "Reference Absolute Angles [F,R,L,B]: ", nodeAbsTargetAngles , "Initial Angle : ", position[gVars.ABSTHETA] 
            #print "Initial Angle and Target Absolute angle [F] must be almost the same "
            #print "___________________________________________________________________________________________"
            
            # Check if there is a wall in the front, if it is free creates a new node 
            # with it's corresponding edge.
            # checkFront(gVars) #equivalence
            print "-------------------------------------START WALL CHEKING------------------------------------"
            print "___________________________________________________________________________________________"
            
            print "Checking FRONT Wall"
            
            checkIfWall(gVars, nodeTargetAngles[gVars.FRONT])
            print "FRONT PATH EXPLORED"
            print "___________________________________________________________________________________________"
            print "Checking RIGHT Wall"
            # Move to the right to check the next wall
            print "Start movement to the right" 
            gVars.motion.moveTo(0,0,angleDifference(gVars,nodeTargetAngles[gVars.RIGHT]),gVars.walkParameterFixedRotation)
            
            # Correction of movement to reach exact target point
            print "ADJUSTMENT TO THE RIGHT WALL"
            #print "Angle Without adjustment : " , gVars.memory.getData(gVars.ANGLEZ) , "Target Angle : " , nodeTargetAngles[gVars.RIGHT]
            adjustmentTheta2(gVars,nodeTargetAngles[gVars.RIGHT])
            
            print "Angle after adjustment : " , gVars.memory.getData(gVars.ANGLEZ)
            position = gVars.motion.getRobotPosition(False)
            print "Absolute Angle after adjustment : " , position[gVars.ABSTHETA]
            print "Right wall check"
            checkIfWall(gVars, nodeTargetAngles[gVars.RIGHT])
            
            print "RIGHT PATH EXPLORED"
            print "___________________________________________________________________________________________"
            print "Checking LEFT Wall"
            # Move to the left to check the next wall 
            print "Start movement to the right" 
            gVars.motion.moveTo(0,0,angleDifference(gVars,nodeTargetAngles[gVars.LEFT]),gVars.walkParameterFixedRotation)
            
            # Correction of movement to reach exact target point
            print "ADJUSTMENT TO THE LEFT WALL"
            #print "Angle Without adjustment : " , gVars.memory.getData(gVars.ANGLEZ) , "Target Angle : " , nodeTargetAngles[gVars.LEFT]
            adjustmentTheta2(gVars,nodeTargetAngles[gVars.LEFT])
            
            print "Angle after adjustment : " , gVars.memory.getData(gVars.ANGLEZ)
            position = gVars.motion.getRobotPosition(False)
            print "Absolute Angle after adjustment : " , position[gVars.ABSTHETA]
            print "Left wall check"
            checkIfWall(gVars, nodeTargetAngles[gVars.LEFT])
            
            print "LEFT PATH EXPLORED"
            print "___________________________________________________________________________________________"
            

        else: 
            print "Node previously known"
            # If the node is already known, we jump directly to choosing the next node 
            # depending on search algorithm. 
            #
            # Return target angles relative to FRONT of gVars.actualNode
            # angles = [iRelTheta,rightAngle,leftAngle,backAngle] 

            angle = (gVars.memory.getData(gVars.ANGLEZ))
            InitialAngle = InitialOrientationAngle(gVars, angle)
            nodeTargetAngles = referenceAngles(gVars, InitialAngle)

            nodeAbsTargetAngles = referenceAngles(gVars, position[gVars.ABSTHETA])
            print "Target Absolute Angles of the node : " , nodeAbsTargetAngles
        
        #print "next node :", next_node    
        # Move to the next node!!!
        print "Next node to visit : " , next_node , "Next Target Angle : " , next_angle   
        print "Startint movment to the next node "
        gVars.motion.moveTo(0,0,angleDifference(gVars,next_angle),gVars.walkParameterFixedRotation)
        print "ADJUSTMENT TO THE NEXT NODE "
        print "Angle Without adjustment : " , gVars.memory.getData(gVars.ANGLEZ) , "Target Angle : " , next_angle 
        adjustmentTheta2(gVars,next_angle)
        print "Angle after adjustment : " , gVars.memory.getData(gVars.ANGLEZ)
        
        # To improve: 
        # When node is already explored do a command similar to moveReallyForward 
        # (modify that function)
        # to reach the exact point already discovered, would permit to reduce 
        # position error when going back. 

        print "Start moving to the next node"      

        moveReallyForward(gVars,0.7,next_angle)
        adjustmentTheta2(gVars, next_angle)
        # Set variables for exploration of next node
        gVars.actualNode = next_node
     

    print "Ending LAB Solving program "
    gVars.posture.goToPosture("Crouch",0.5)    

# To be modified if we want to load programm with an already explored graph()
if __name__ == "__main__":
    robotIp = "127.0.0.1"

    if len(sys.argv) <= 1:
        print "Usage python gVars.motion_walk.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)


