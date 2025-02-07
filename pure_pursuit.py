#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from copy import deepcopy
import time
import numpy as np
import math
import csv

# TODO: import ROS msg types and libraries

p = 0.5
max_velocity = 6

file_path = '/home/romeo/rcws/logs/wp-2025-01-17-23-01-38.csv'

class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        # TODO: create ROS subscribers and publishers. Il faut pouvoir conduire, récupérer la position en fait le lidar ne doit pas être nécessaire
        realCar = False

        if(realCar):
             lidarscan_topic = '/scan'
             loc_topic = '/pf/pose/odom'
             drive_topic = "/vesc/ackermann_cmd_mux/input/navigation"
        else:
             lidarscan_topic = '/scan'
             loc_topic = '/odom'
             drive_topic = '/nav'
	
	
        #self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.loc_sub = rospy.Subscriber(loc_topic, Odometry, self.pose_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

        self.marker_pub = rospy.Publisher ("/dynamic_viz",Marker, queue_size=10)

        self.L = 1
        self.waypoint = []
        self.waypoint = self.parse_waypoint(file_path)

        self.orientation = 0
        

        
    
    def parse_waypoint(self, file_path):
        waypoints = []
        with open(file_path, mode='r', encoding='utf-8') as fichier:
            lecteur_csv = csv.reader(fichier)
            next(lecteur_csv)

            for ligne in lecteur_csv:
                x,y = float(ligne[0]), float(ligne[1])
    
                point = Point()
                point.x = x
                point.y = y

                if waypoints == []:
                    waypoints.append(point)
                else:
                    lastPoint = waypoints[-1]
                    if(self.dist(lastPoint, point) > self.L/2):
                        waypoints.append(point)
        return waypoints
    
    def pose_callback(self, pose_msg):
        carPose = pose_msg.pose.pose

        quaternion = np.array([carPose.orientation.x, carPose.orientation.y, carPose.orientation.z,carPose.orientation.w]) 
        angles = euler_from_quaternion(quaternion)
        self.orientation = angles[2]

        rospy.loginfo_throttle(0.5, "position de la voiture :" + str(pose_msg.pose.pose.position.x) + str(pose_msg.pose.pose.position.y))

        
        # TODO: find the current waypoint to track using methods mentioned in lecture
        waypoint = self.find_waypoint(pose_msg.pose.pose)
        
        # TODO: transform goal point to vehicle frame of reference
        goal = Point()

        #translation 
        goal.x, goal.y, goal.z = waypoint.x - carPose.position.x, waypoint.y - carPose.position.y, 0

        #rotation
        quaternion = np.array([carPose.orientation.x, carPose.orientation.y, carPose.orientation.z,carPose.orientation.w])
        angles = euler_from_quaternion(quaternion)
        yaw = angles[2]

        goal.x, goal.y = np.cos(yaw) * goal.x + np.sin(yaw) * goal.y, - np.sin(yaw)*goal.x + np.cos(yaw) * goal.y
        #goal.x, goal.y = np.sin(yaw)*goal.x - np.cos(yaw) * goal.y, np.cos(yaw) * goal.x + np.sin(yaw) * goal.y
        

        # TODO: calculate curvature/steering angle
        curvature = 2 * goal.y / (self.L ** 2)

        steering_angle = p * curvature

        if(abs(steering_angle) > 0.4189):
            steering_angle = steering_angle/abs(steering_angle) * 0.4189

        velocity = max_velocity * np.cos(steering_angle*2)**2

        
        # TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "purePursuit"
        rospy.loginfo_throttle(0.5,"consigne angle : " + str(steering_angle) + "consigne vitesse : " + str(velocity))
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.id = 1234321
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = waypoint.x
        marker.pose.position.y = waypoint.y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        # Example
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.6
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)
        
        

    def intersect_line_circle(self, circle_center, radius, point1, point2):
        ''' point 1 est dans le cercle et point2 à l'exterieur '''
        t = 1/2 
        precision = 0.01
        incr = 1/4

        for _ in range(5):
            currentPoint = Point()
            currentPoint.x, currentPoint.y = point1[0] * (1-t) + point2[0] * t, point1[1] * (1-t) + point2[1] * t
            d = self.dist(circle_center, currentPoint)

            if(abs(d - radius) < precision):
                return currentPoint
            
            elif(d < radius):
                t += incr
            
            else:
                t -= incr

            incr *= 1/2
        
        return currentPoint

    def isAhead(self, point, carPose):
        
        directionAngle = self.orientation

        directionVectorX = -math.sin(directionAngle)
        directionVectorY = math.cos(directionAngle)

        directionVectorX = math.cos(directionAngle)
        directionVectorY = math.sin(directionAngle)

        vectX = point.x -carPose.position.x
        vectY = point.y -carPose.position.y
        

        dotProduct = vectX * directionVectorX + vectY * directionVectorY

        
        return dotProduct > 0

    def find_waypoint(self, pose):
        inf_dist_over_L = 1000
        wayPoint2 = None

        sup_dist_under_L = 0
        wayPoint1 = None

        for waypoint in self.waypoint:
            
            if(self.isAhead(waypoint, pose)):
                
                d = self.dist(pose.position, waypoint)

                

                if(d > self.L and d < inf_dist_over_L):
                    inf_dist_over_L = d
                    wayPoint1 = waypoint.x, waypoint.y

                if(d < self.L and d > sup_dist_under_L):
                    sup_dist_under_L = d
                    wayPoint2= waypoint.x, waypoint.y
        
        

        if(wayPoint1 != None and wayPoint2 != None):

            rospy.loginfo_throttle(0.5, str(wayPoint1[0]) + str(wayPoint1[1]))
        
            return self.intersect_line_circle(pose.position, self.L, wayPoint1, wayPoint2)
        
        elif(wayPoint1 != None):
            #print("terrible 2")
            point = Point()
            point.x = wayPoint1[0]
            point.y = wayPoint1[1]
            return point
        
        elif(wayPoint2 != None):
            #print("terrible 1")
            point = Point()
            point.x = wayPoint2[0]
            point.y = wayPoint2[1]
            return point
        
        else:
            #print("on a rien trouvé ca n'a aucun sens")
            return self.waypoint[0]

    def dist(self, pose1, pose2):
        x1, y1 = pose1.x, pose1.y
        x2, y2 = pose2.x, pose2.y

        return ((x2 - x1)**2 + (y2 - y1)**2)**(1/2)

def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()