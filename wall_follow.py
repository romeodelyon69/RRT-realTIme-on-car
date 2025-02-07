#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import csv
from time import gmtime, strftime, time
from os.path import expanduser
import atexit

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


#PID CONTROL PARAMS
kp = 2
kd = 4
ki = 0

servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
#DESIRED_DISTANCE_RIGHT = 0.8 # meters
#DESIRED_DISTANCE_LEFT = 0.8
VELOCITY = 5 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
PI = 3.14159

realCar = False

home = expanduser('~')
fichier_csv = strftime(home+'/rcws/logs/lidarAndActions-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv'
fichier =  open(fichier_csv, mode='w', newline="") 
writer = csv.writer(fichier)

t = time()


def shutdown():
    fichier.close()
    print('Goodbye')

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        if(realCar):
             lidarscan_topic = '/scan'
             drive_topic = "vesc/ackermann_cmd_mux/input/navigation"
        else:
             lidarscan_topic = '/scan'
             drive_topic = '/nav'
	
	
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped)

        self.DESIRED_DISTANCE_RIGHT = 0.8 # meters
        self.DESIRED_DISTANCE_LEFT = 0.8

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        angle_min = data.angle_min
        incr = data.angle_increment
        
        indiceAngle = int((angle-angle_min)/incr)

        return data.ranges[indiceAngle]

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = 0.0
        #TODO: Use kp, ki & kd to implement a PID controller for 
        angle = kp * error + ki * integral - kd *(error - prev_error)

        '''if(angle < 10 and angle > 0):
             velocity = 1.5
        elif(angle < 20 and angle > 10):
             velocity = 1
        else:
             velocity = 0.5'''

        velocity = velocity / (1 + (abs(angle) ))

        prev_error = error
        integral += error 

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        #print("consigne angle : ", angle, "consigne vitesse : ", velocity)
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

        return velocity, angle

    def followRight(self, theta, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        a = leftDist[0] 
        b = leftDist[1] 
        alpha = math.atan2((a*math.cos(theta) - b), a*math.sin(theta))
        #print("distance : ", b*math.cos(alpha), "   a :", a, "     b :", b)
        return self.DESIRED_DISTANCE_RIGHT - (b*math.cos(alpha) + CAR_LENGTH *math.sin(alpha))

    def followLeft(self, theta, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        a = leftDist[0] 
        b = leftDist[1] 

        alpha = math.atan2((a*math.cos(theta) - b), a*math.sin(theta))
        #print("distance : ", b*math.cos(alpha), "   a :", a, "     b :", b)
        return self.DESIRED_DISTANCE_RIGHT - (b*math.cos(alpha) + CAR_LENGTH *math.sin(alpha))

    def lidar_callback(self, data):
        """ 
        """
        angleX = -PI/2
        theta = 30 / 180 * 3.14159 

        indice_angle0 = int((angleX-data.angle_min) / data.angle_increment)
        indice_angle180 = int(indice_angle0 + 3.14159 / data.angle_increment)

        #rospy.loginfo_throttle(0.5,"indice0 : " + str(indice_angle0) + "indice180 : " + str(indice_angle180) + "taille Totale : " + str(len(data.ranges)))
        #rospy.loginfo_throttle(0.5,"dist right :" + str(data.ranges[indice_angle0]) + "dist left :" + str(data.ranges[indice_angle180]))

        widthTrack = (data.ranges[indice_angle0] + data.ranges[indice_angle180])

        
        rightDist = [self.getRange(data, theta + angleX), self.getRange(data, angleX)]
        errorRight = self.followRight(theta, rightDist) #TODO: replace with error returned by followLeft

        leftDist = [self.getRange(data, - theta + angleX + PI), self.getRange(data, angleX + PI)]
        errorLeft = self.followLeft(theta, leftDist) #TODO: replace with error returned by followLeft

        widthTrack = self.DESIRED_DISTANCE_LEFT + self.DESIRED_DISTANCE_RIGHT - errorLeft - errorRight

        self.DESIRED_DISTANCE_LEFT = widthTrack/2
        self.DESIRED_DISTANCE_RIGHT = widthTrack/2

        #send error to pid_control
        if(abs(errorRight) < abs(errorLeft)):
            #print("LEFTTT")
            velocity, steeringAngle = self.pid_control(-errorLeft, VELOCITY)
        else:
            #print("RIGHTTTTT")
            velocity, steeringAngle = self.pid_control(errorRight, VELOCITY)
        
        global t 

        if(time()-t > 0.05):
            dataFront = data.ranges[indice_angle0 : indice_angle180+1]
            step = int(len(dataFront)/20)
            writer.writerow([velocity, steeringAngle, dataFront[::step]])
            
            t = time()

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
    atexit.register(shutdown)
    main(sys.argv)
