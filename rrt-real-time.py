"""
ESE 680
RRT assignment
Author: Hongrui Zheng

This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math
import random

import csv
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
import time
import subprocess
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
#from tf import transform_listener

realCar = False
infinity = float('inf')
DEPTH =60
TEMPERATURE = 0.90
ITERATION = 1000
car_size = 0.60
STRAIGHT_LINE_COEFF = 0.3



if(realCar):
    lidarscan_topic = '/scan'
    loc_topic = '/pf/pose/odom'
    drive_topic = "/vesc/ackermann_cmd_mux/input/navigation"
    file_path = '/home/nvidia/romeoNzt/wp-2025-02-07-13-04-47.csv'
else:
    lidarscan_topic = '/scan'
    loc_topic = '/odom'
    drive_topic = '/nav'
    file_path = '/home/romeo/rcws/logs/wp-2025-01-17-23-01-38.csv'



# TODO: import as you need

# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
    def __init__(self):
        self.x = None
        self.y = None

        self.x_grid = None
        self.y_grid = None

        self.parent = None
        self.cost = None # only used in RRT*
        self.is_root = False

        self.orientation = np.array([0,0])       #normalized vector representing the orientation of the robot at this point, based on the line between the parent and the node

        self.bubbles = []   #On va découper la grille en une union (non disjointe) de disque de diamètre 2*self.neighboor_dist répartis sur une grille de maille neighboor_dist*neigboor_dist. On pourra ainsi optimiser la recherche des voisins puisqu'ils seront nécessairement dans les boules contenant le noeud
        self.neighboor = []



# class def for RRT
class RRT(object):
    def __init__(self):
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.
        
        #map_topic = rospy.get_param('map')

        # TODO: create subscribers

        print("Don't forget to launch : rosrun map_server map_server/home/romeo/catkin_ws/mymap.yaml")
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        rospy.Subscriber(loc_topic, Odometry, self.pose_callback)

        rospy.Subscriber(lidarscan_topic, LaserScan, self.scan_callback, queue_size=1)

        self.scheduler = rospy.Publisher('scheduler', String, queue_size=10)

        self.way_point_publisher = rospy.Publisher('waypoints', String, queue_size=5)

        # publishers
         # TODO: create a drive message publisher, and other publishers that you might need
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        self.marker_pub = rospy.Publisher ("/dynamic_viz",Marker, queue_size=1000)

        # class attributes
        # TODO: maybe create your occupancy grid here
        self.L = 0.5                #distance at which the samples are created from the nearest point in the tree
        self.neighboor_dist = 0.7   #distance at which you consider two points are close enough to be linked

        self.gauche = 0
        self.droite = 0
        self.haut = 0
        self.bas = 0
        
        self.have_launched_the_map = False
        self.is_runing = False
        self.occupancy_grid = OccupancyGrid()
        self.local_grid = OccupancyGrid()

        self.curent_path = []

        self.pose = Point()
        self.orientation = 0

        self.t = time.time()
        self.time_origin = time.time()

        self.total_time_near = 0
        self.total_time_find_path = 0
        self.total_time_rewiring = 0
        self.nb_iterations = 0

        self.nb_near = 0
        self.nb_rewiring = 0

        self.waypoints = self.parse_waypoint(file_path)

        self.bubbles_list = []
        self.bubble_upper_corner_id = (0,0)

        self.targetPoint= self.waypoints[0]
        self.targetPointIndex = 0

        self.nb_collisions = 0
        self.nb_success = 0

    def parse_waypoint(self, file_path):
        waypoints = []
        with open(file_path, mode='r') as fichier:
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
                    if(dist((lastPoint.x, lastPoint.y), (point.x, point.y)) > 1):
                        waypoints.append(point)
        return waypoints

    
    def grid_pose_from_world_pose(self, point):
        i,j = (point.y - self.occupancy_grid.info.origin.position.y)/self.occupancy_grid.info.resolution, (point.x - self.occupancy_grid.info.origin.position.x)/self.occupancy_grid.info.resolution 
        return int(i * self.occupancy_grid.info.width + j)
    
    def world_pose_from_grid_pose(self, index):
        i,j = divmod(index, self.occupancy_grid.info.width)
        point = Point()
        point.x = j * self.occupancy_grid.info.resolution
        point.y = i * self.occupancy_grid.info.resolution
        return point
    
    def map_callback(self, map_msg):
        if self.have_launched_the_map:
            return 
    
        self.occupancy_grid = map_msg
        self.occupancy_grid.data = list(self.occupancy_grid.data)
        print("map caracteristics : width", self.occupancy_grid.info.width, " height : ", self.occupancy_grid.info.height, " resolution : ", self.occupancy_grid.info.resolution)
        self.local_grid.data = [0 for i in range(len(self.occupancy_grid.data))]
        time.sleep(1)

        self.find_map_corner()
        self.thicken_map()

        self.have_launched_the_map = True

        #on va donc pouvoir construire les bules dans le rectangle défini par les bords de la carte (self.gauche, haut, droite, bas)
        w = int((self.droite - self.gauche) * self.occupancy_grid.info.resolution/(self.neighboor_dist)) + 1 + 1
        h = int((self.bas - self.haut) * self.occupancy_grid.info.resolution/(self.neighboor_dist)) + 1 + 1
        
        #le upper corner est défini par haut et gauche 
        i = int(self.haut * self.occupancy_grid.info.resolution / (self.neighboor_dist))
        j = int(self.gauche * self.occupancy_grid.info.resolution / (self.neighboor_dist))

        self.bubble_upper_corner_id = (i,j)

        print("on a donc h et w : ", h, w)
        print("upper corner bubble : ", i,j)
        self.bubbles_list = [[[] for i in range(w)] for j in range(h)]
        
        
    def thicken_map(self):
        '''On epaissit les murs de la cartes pour que le robot ne se prenne pas dedans'''
        grid = self.occupancy_grid.data
        w = self.occupancy_grid.info.width
        h = len(grid) // w

        thickness = int(car_size // self.occupancy_grid.info.resolution)

        for i, valeur in enumerate(grid):
            if valeur == 100:
                ligne, colonne = divmod(i, w)

                for j in range(-thickness,thickness):
                    for k in range(-thickness + abs(j),thickness - abs(j)):
                        index = (ligne + j) * w + colonne + k
                        if index >= 0 and index < len(grid) and grid[index] < 100:
                            grid[index] = 95
    
    def pose_callback(self, pose_msg):
        car_pose = pose_msg.pose.pose.position
        self.pose = car_pose

        #print(self.pose.x, self.pose.y)
        
        if time.time() - self.t > 0.02:
            self.t = time.time()
            quaternion = np.array([pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z,pose_msg.pose.pose.orientation.w]) 
            angles = euler_from_quaternion(quaternion)
            self.orientation = angles[2]

    def scan_callback(self, data):
        
        if self.have_launched_the_map:
            if not self.is_runing:
                self.nb_iterations += 1
                #prendre en compte les donnees du lidar pour construire une carte local qui s ajoute a la carte generale
                angle = data.angle_min
                
                #takes perhaps to much time, esay improve : save the added points in a list and remove them one by one
                
                '''
                self.erase_local_grid()

                for dist in data.ranges:
                    cell = self.cell_from_dist(dist, angle)
                    angle += data.angle_increment
                    if cell < len(self.local_grid.data) and cell > 0:
                        self.local_grid.data[cell] = 100 
                '''               
                
                self.next_waypoint()

                temps = time.time()
                path = self.find_shortest_path((self.pose.x, self.pose.y), (self.targetPoint.x, self.targetPoint.y), 0)
                self.total_time_find_path += (time.time() - temps)

                if path != []:
                    self.way_point_publisher.publish(self.path2msg(path))
                    #print("new path !")

                rospy.loginfo_throttle(0.5,"le temps moyen des fonctions find_path est de : " + str(self.total_time_find_path/self.nb_iterations))


    def cell_from_dist(self, dist, angle): 
        '''Prend une distance a la voiture et un angle de direction et renvoie la cellule sur la carte correspondant a ce point'''
        total_angle = angle + self.orientation

        x, y = dist * math.cos(total_angle), dist*math.sin(total_angle)

        point = Point()
        point.x, point.y = self.pose.x + x, self.pose.y + y

        return self.grid_pose_from_world_pose(point)
        
    def next_waypoint(self):
                
        d = dist((self.pose.x, self.pose.y), (self.targetPoint.x, self.targetPoint.y))

        if(d < self.L*12):
            print("ON CHANGE DE POINT !!!!!!!!!!!!!!!!!!!!!!!!!!")
            self.targetPointIndex = (self.targetPointIndex + 1)
            if self.targetPointIndex >= len(self.waypoints):
                self.targetPointIndex = 0
            self.targetPoint = self.waypoints[self.targetPointIndex]
            self.next_waypoint()                                        #on vérifie que le nouveau point n'est pas trop proche du robot aussi
        
    
    def erase_local_grid(self):
        for i in range(len(self.occupancy_grid.data)):
            self.local_grid.data[i] = 0

    def find_shortest_path(self, start_point, end_point, START_ID):

        self.is_runing = True
        global TEMPERATURE
        tree = []
        best_path_length = infinity


        temps = time.time()

        w = len(self.bubbles_list[0])
        h = len(self.bubbles_list)

        
        #on reinitialise les boules
        for i in range(h):
            for j in range(w):
                self.bubbles_list[i][j] = []

        
        self.total_time_near += (time.time()-temps)

        origin_node = Node()
        origin_node.x = start_point[0]
        origin_node.y = start_point[1]

        origin_node.cost = 0

        origin_node.x_grid = int((origin_node.x - self.occupancy_grid.info.origin.position.x)/self.occupancy_grid.info.resolution)
        origin_node.y_grid = int((origin_node.y - self.occupancy_grid.info.origin.position.y)/self.occupancy_grid.info.resolution)

        origin_node.is_root = True
        
        origin_node.orientation = np.array([math.cos(self.orientation), math.sin(self.orientation)])

        goal_x, goal_y = end_point


        tree.append(origin_node)
        #print("origin_node : ", origin_node.x, origin_node.y, origin_node.x_grid, origin_node.y_grid)

        #on doit ajouter le noeud à la liste des bubbles de la grille qui contiennet bien le point
        temps = time.time()
        x, y = origin_node.x, origin_node.y
        i,j = int((y - self.occupancy_grid.info.origin.position.y)/(self.neighboor_dist)), int((x - self.occupancy_grid.info.origin.position.x)/(self.neighboor_dist))
        i,j = i - self.bubble_upper_corner_id[0], j - self.bubble_upper_corner_id[1]
        #le point (x,y) peut alors appartenir aux boules (i,j), (i+1,j), (i,j+1), (i+1,j+1)
        for k in range(0,2):
            for l in range(0,2):
                if i+k >= 0 and i+k < h and j+l >= 0 and j+l < w:
                    bubleX, bubleY = (j+l+self.bubble_upper_corner_id[1])*(self.neighboor_dist) + self.occupancy_grid.info.origin.position.x, (i+k+self.bubble_upper_corner_id[0])*(self.neighboor_dist) + self.occupancy_grid.info.origin.position.y
                    d = dist((bubleX, bubleY), (x, y))

                    if d < 2*self.neighboor_dist:
                        origin_node.bubbles.append(self.bubbles_list[i+k][j+l])
                        self.bubbles_list[i+k][j+l].append(origin_node)

        self.total_time_near += (time.time()-temps)
        
        path = []
        t = 0
        while(t < ITERATION):
            self.nb_near += 1
            #print("iteration : ", t, self.bubbles_list)
            #printMarker(0,0,(0,1,1), 12345678, self.marker_pub)
            printMarker(goal_x,goal_y,(0,1,1), 123456789, self.marker_pub)
            #print(len(tree))
            if t < len(self.curent_path):           #on va essayer de suivre le chemin déjà calculé
                new_node = self.curent_path[t]
            else:
                if(t == ITERATION-1):                                   #on termine toujours par le but pour recalculer si un meilleur chemin a été trouver depuis la dernière fois
                    sample_point = self.sample(0, goal_x, goal_y)
                else:
                    sample_point = self.sample(TEMPERATURE, goal_x, goal_y)

                temps = time.time()

                nearest_node = self.nearest(tree, sample_point)
                self.total_time_near += (time.time()-temps)

                new_node = self.steer(nearest_node, sample_point)

            '''print("on a un nouveau noeud : ", new_node.x, new_node.y)'''

            if(self.check_collision(nearest_node, new_node)):
                #on doit ajouter le noeud à la liste des bubbles de la grille qui contiennet bien le point
                x, y = new_node.x, new_node.y
                w = len(self.bubbles_list[0])
                h = len(self.bubbles_list)

                i,j = int((y - self.occupancy_grid.info.origin.position.y)/self.neighboor_dist), int((x - self.occupancy_grid.info.origin.position.x)/self.neighboor_dist)
                i,j = i - self.bubble_upper_corner_id[0], j - self.bubble_upper_corner_id[1]

                #le point (x,y) peut alors appartenir aux boules (i,j), (i+1,j), (i,j+1), (i+1,j+1)
                for k in range(0,2):
                    for l in range(0,2):
                        if i+k >= 0 and i+k < h and j+l >= 0 and j+l < w:
                            bubleX, bubleY = (j+l+self.bubble_upper_corner_id[1])*self.neighboor_dist + self.occupancy_grid.info.origin.position.x, (i+k+self.bubble_upper_corner_id[0])*self.neighboor_dist + self.occupancy_grid.info.origin.position.y
                            d = dist((bubleX, bubleY), (new_node.x, new_node.y))
                            '''print("distance entre le point et la boule : ", d)'''
                            if d < 2*self.neighboor_dist:
                                '''print("on a ajouté un noeud à la boule : ", i+k, j+l)'''
                                new_node.bubbles.append(self.bubbles_list[i+k][j+l])
                                self.bubbles_list[i+k][j+l].append(new_node)

                
                temps = time.time()
                new_node.neighboor = self.near(tree, new_node)

                self.total_time_near += (time.time()-temps)

                tree.append(new_node)
                #printMarker(new_node.x, new_node.y, (1,0,0), len(tree), self.marker_pub)
                
                #print(len(new_node.neighboor))
                #print(new_node.x, new_node.y)

                temps = time.time()
                for neighboor, d in new_node.neighboor:
                    neighboor.neighboor.append((new_node, d))
                    attempt_dist = neighboor.cost + self.line_cost(new_node, neighboor)

                    if attempt_dist < new_node.cost:
                        new_node.parent = neighboor
                        new_node.cost = attempt_dist

                        vector = np.array([new_node.x - neighboor.x, new_node.y - neighboor.y])
                        unit_vector = vector / d

                        new_node.orientation = unit_vector
                        #print("find a better parent for the node")'''
                
                self.nb_rewiring += 1
                self.rewire(new_node, tree, 0)
                self.total_time_rewiring += (time.time() - temps)
                rospy.loginfo_throttle(0.5,"le temps moyen des fonctions rewiring sur une itération est de : " + str(self.total_time_rewiring / self.nb_iterations))
                self.nb_success += 1
            else:
                self.nb_collisions += 1
                #print("on a collisioné")

            '''if self.nb_collisions % 1000 ==0:
                print(self.nb_collisions, " collisions")
                print(self.nb_success, " success")
                print("ratio : ", self.nb_success/(self.nb_collisions + self.nb_success))
            '''

            if(self.is_goal(tree[-1], goal_x, goal_y)):
                distance = tree[-1].cost
                if(distance < best_path_length):

                    best_path_length = distance
                    #print("meilleur distance pour le moment :", dist)
                    path = self.find_path(tree, tree[-1])
                    TEMPERATURE = 0.999
            
            
            t += 1 

        rospy.loginfo_throttle(0.5,"le temps moyen des fonctions near et nearest sur une iteration est de : " + str(self.total_time_near / self.nb_iterations))

        self.print_path(path, START_ID)
        '''
        print("origin_node : ", origin_node.x, origin_node.y, origin_node.x_grid, origin_node.y_grid)
        print("over", best_path_length)
        print("distance a vol d'oiseau : ", dist((self.pose.x, self.pose.y), (self.targetPoint.x, self.targetPoint.y)))
        print("position :", self.pose.x, self.pose.y)
        '''

        self.is_runing = False

        return path



    def path2msg(self, path):
        msg = String()
        for waypoint in path:
            msg.data += ('(' + str(waypoint.x) + ',' + str(waypoint.y) + ')')
        
        return msg

    def rewire(self, node, tree, depth):
        if depth >= DEPTH:
            return 

        for neighboor, d in node.neighboor:
            attempt_dist = node.cost + self.line_cost(node, neighboor)
            if neighboor.cost > attempt_dist:
                #print("rewiring the tree")
                neighboor.parent = node
                neighboor.cost = attempt_dist

                vector = np.array([neighboor.x - node.x, neighboor.y - node.y])
                unit_vector = vector / d

                neighboor.orientation = unit_vector
                
                self.rewire(neighboor, tree, depth + 1)

    def find_map_corner(self):

        grid = self.occupancy_grid.data
        w = self.occupancy_grid.info.width
        
        print("largeur de la carte : ", w)
        h = len(grid) // w
        haut = h
        bas = -1
        gauche = w
        droite = -1

        for i, valeur in enumerate(grid):
            if valeur > 60:
                ligne, colonne = divmod(i, w)


                haut = min(haut, ligne)
                bas = max(bas, ligne)
                gauche = min(gauche, colonne)
                droite = max(droite, colonne)

        map_offset = 20
        #Les +map_offset sont la pour que les coins de la carte ne soient pas jamais explores sinon c'est un peu triste et tres galere 
        self.haut = max(haut-map_offset, 0)
        self.bas = min(bas+map_offset, h-1)
        self.gauche = max(gauche-map_offset, 0)
        self.droite = min(droite+map_offset, w-1)
        
        print("gauche : ", gauche, " droite : ", droite , " haut : ", haut, " bas : ", bas)


    def find_goal(self):
        return -3.5,6
    def sample(self, temperature, goal_x, goal_y):
        
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        
        t = random.random()
        if(t > temperature):
            return goal_x, goal_y

        while(True):
            x,y = random.randint(self.gauche, self.droite), random.randint(self.haut, self.bas)
            #print("random point : ", x,y)
            index = y * self.occupancy_grid.info.width + x
            if(self.occupancy_grid.data[index] < 20):
                x,y = x * self.occupancy_grid.info.resolution, y * self.occupancy_grid.info.resolution 
                x += self.occupancy_grid.info.origin.position.x
                y += self.occupancy_grid.info.origin.position.y
                return (x,y)
        



    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node = None
        min_d = infinity

        #on va chercher dans les boules proche du noeud s'il y a d'autres points jusqu'à en trouver, (s'il y a beaucoup de point la recherche sera donc très rapide)
        x, y = sampled_point[0], sampled_point[1]
        w = len(self.bubbles_list[0])
        h = len(self.bubbles_list)

        i,j = int((y - self.occupancy_grid.info.origin.position.y)/(self.neighboor_dist)), int((x - self.occupancy_grid.info.origin.position.x)/(self.neighboor_dist))
        i,j = i - self.bubble_upper_corner_id[0], j - self.bubble_upper_corner_id[1]

        #On va parcourir les bulles de façon concentrique autour du sampled_point jusqu'à trouver une boulle non vide
        for r in range(h + w):
            for l in range(-r, r+1):
                k = r - abs(l)
                if i+k >= 0 and i+k < h and j+l >= 0 and j+l < w:
                    for node in self.bubbles_list[i+k][j+l]:
                        #print("on a trouvé un voisin dans la boule : ", i+k, j+l, "qui vaut : ", self.bubbles_list[i+k][j+l], " dans ", self.bubbles_list) 

                        d = dist((node.x, node.y), sampled_point)
                        if d < min_d:
                            min_d = d
                            nearest_node = node
                if i-k >= 0 and i-k < h and j+l >= 0 and j+l < w:
                    for node in self.bubbles_list[i-k][j+l]:
                        #print("on a trouvé un voisin dans la boule : ", i-k, j+l, "qui vaut : ", self.bubbles_list[i-k][j+l], " dans ", self.bubbles_list) 

                        d = dist((node.x, node.y), sampled_point)
                        if d < min_d:
                            min_d = d
                            nearest_node = node

            if nearest_node != None:
                break

        '''

        for node in tree:
            d = dist((node.x, node.y), sampled_point)
            if d < min_d:
                min_d = d
                nearest_node = node
        '''             
        return nearest_node
        

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """


        
        d = dist((nearest_node.x, nearest_node.y), sampled_point)

        if(d > self.L):
            vx = sampled_point[0] - nearest_node.x
            vy = sampled_point[1] - nearest_node.y

            #print(((nearest_node.x, nearest_node.y), sampled_point))

            x = nearest_node.x + vx/d*self.L
            y = nearest_node.y + vy/d*self.L

            
        else:
            x = sampled_point[0]
            y = sampled_point[1]

        new_node = Node()

        new_node.x = x
        new_node.y = y

        new_node.x_grid = int((x - self.occupancy_grid.info.origin.position.x)/self.occupancy_grid.info.resolution)
        new_node.y_grid = int((y - self.occupancy_grid.info.origin.position.y)/self.occupancy_grid.info.resolution)
        new_node.parent = nearest_node
        new_node.cost = nearest_node.cost + self.line_cost(new_node, nearest_node)
            
        return new_node

    #on utilise l'algo de Bresenham parce qu'on fait des lignes droites dans une grille 
    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        x0,y0,x1,y1 = nearest_node.x_grid, nearest_node.y_grid, new_node.x_grid, new_node.y_grid
    
        """Yield integer coordinates on the line from (x0, y0) to (x1, y1).

        Input coordinates should be integers.

        The result will contain both the start and the end point.
        """
        dx = x1 - x0
        dy = y1 - y0

        xsign = 1 if dx > 0 else -1
        ysign = 1 if dy > 0 else -1

        dx = abs(dx)
        dy = abs(dy)

        if dx > dy:
            xx, xy, yx, yy = xsign, 0, 0, ysign
        else:
            dx, dy = dy, dx
            xx, xy, yx, yy = 0, ysign, xsign, 0

        D = 2*dy - dx
        y = 0

        for x in range(dx + 1):
            x_cur, y_cur =  x0 + x*xx + y*yx, y0 + x*xy + y*yy
            index = y_cur * self.occupancy_grid.info.width + x_cur
            if(self.occupancy_grid.data[index] > 60):
                return False
            if D >= 0:
                y += 1
                D -= 2*dx
            D += 2*dy
            #printMarker(x_cur * self.occupancy_grid.info.resolution + self.occupancy_grid.info.origin.position.x, y_cur * self.occupancy_grid.info.resolution + self.occupancy_grid.info.origin.position.y, (1,1,0), random.randint(0,1000000), self.marker_pub)

        return True

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        d = dist((latest_added_node.x, latest_added_node.y), (goal_x, goal_y))
                 
        if d < self.L:
            #print("salamalecoum mes petits loucoums")
            return True
            
        return False

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        current_node = latest_added_node

        while(not current_node.is_root):
            path.append(current_node)
            current_node = current_node.parent
        path.reverse()
        return path
    
    def print_path(self, path, START_ID):
        for i,node in enumerate(path):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.id = START_ID + i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.position.x = node.x
            marker.pose.position.y = node.y
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            # Example
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            self.marker_pub.publish(marker)

    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line connected to the tree
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        d = dist((n1.x, n1.y), (n2.x, n2.y))
        
        if d == 0:
            return 0
        
        vector = (n2.x - n1.x, n2.y - n1.y)
        unit_vector = (vector[0]/d, vector[1]/d)

        direction_score = (1-(unit_vector[0] * n1.orientation[0] + unit_vector[1] * n1.orientation[1]))/2*STRAIGHT_LINE_COEFF

        return d+direction_score

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes in a tuple with the node first and the distance to the neighboor second
        """
        neighborhood = []
       
        #on va partir les noeuds qui sont dans les boules de rayon self.neighboor_dist autour de node
        for buble in node.bubbles:
            for neighboor in buble:
                #print("on a un voisin potentiel")
                d = dist((node.x, node.y), (neighboor.x, neighboor.y))
                if d < 0.0001:              #on evite de considérer le noeud lui même
                    continue
                if d < self.neighboor_dist:
                    neighborhood.append((neighboor, d))
        '''
        
        for neighboor in tree:
            d = dist((node.x, node.y), (neighboor.x, neighboor.y))
            if d < self.neighboor_dist:
                neighborhood.append(neighboor)
        '''

        return neighborhood
        
    
def dist(point1, point2):
    global total_dist_time

    x1,y1,x2,y2 = point1[0], point1[1], point2[0], point2[1]

    return math.sqrt(((x1 - x2) **2 + (y1 - y2)**2))

def printMarker(x,y, colour, id, pub):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.id = id
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    # Example
    marker.scale.x = 0.25
    marker.scale.y = 0.25
    marker.scale.z = 0.25
    marker.color.a = 1.0
    marker.color.r = colour[0]
    marker.color.g = colour[1]
    marker.color.b = colour[2]
    pub.publish(marker)



def main():
    rospy.init_node('rrt')
    rrt = RRT()
    subprocess.Popen(["rosrun", "map_server", "map_server", "/home/romeoNzt/mapirl.yaml"])
    rospy.spin()


if __name__ == '__main__':
    main()