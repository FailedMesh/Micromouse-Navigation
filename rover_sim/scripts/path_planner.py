from numpy.core.defchararray import endswith
import rospy
import numpy as np
import math
import os
import cv2
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from rover_sim.msg import float_array
from rover_sim.msg import path_array
#from Dijkstra import BinaryHeap, binarize, createGraph, dijkstra, findShortestPath

class BinaryHeap():
    
    def __init__(self):
        self.heap = []
        self.count = 0
        
    def insert(self, data):
        
        if self.count == 0:
            self.heap.append(data)
            self.count += 1
            
        else:
            self.heap.append(data)
            self.count += 1
            child_pos = self.count - 1
            parent_pos = (child_pos - 1)//2
            child = self.heap[child_pos]
            parent = self.heap[parent_pos]

            while (not (parent[1] < child[1])) and (child_pos != 0) : #Heap invariant
                
                #Switch parent and child:
                self.heap[child_pos] = parent
                self.heap[parent_pos] = child

                #Define new parent and child:
                child_pos = parent_pos
                parent_pos = (child_pos - 1)//2
                child = self.heap[child_pos]
                parent = self.heap[parent_pos]
                
    def poll(self):
        
        if self.count == 0:
            return None
        
        root = self.heap[0]
        
        parent_pos = 0
        left = parent_pos*2 + 1
        right = parent_pos*2 + 2
        
        while (right <= self.count):
                          
            if (right == self.count):
                if (self.heap[left] == None):
                    break
                else:
                    self.heap[parent_pos] = self.heap[left]
                    self.heap[left] = None
                    parent_pos = left
                    
            else:
                if (self.heap[left] ==  None and self.heap[right] == None):
                    break
                    
                elif (self.heap[left] ==  None):
                    self.heap[parent_pos] = self.heap[right]
                    self.heap[right] = None
                    parent_pos = right
                    
                elif (self.heap[right] == None):
                    self.heap[parent_pos] = self.heap[left]
                    self.heap[left] = None
                    parent_pos = left
                    
                elif (self.heap[left][1] < self.heap[right][1]):
                    self.heap[parent_pos] = self.heap[left]
                    self.heap[left] = None
                    parent_pos = left
                    
                else:
                    self.heap[parent_pos] = self.heap[right]
                    self.heap[right] = None
                    parent_pos = right

                
            left = parent_pos*2 + 1
            right = parent_pos*2 + 2
            
        del self.heap[parent_pos]
        self.count -= 1
            
        return root
        


def binarize(image, resolution, rover_length, rover_width, breathing_room = 0.02):
    
    rover_length += breathing_room
    rover_width += breathing_room

    pixel_width = math.ceil(max(rover_length, rover_width)/resolution)
    graph_width = math.ceil(image.shape[0]/pixel_width)
    pixel_resolution = pixel_width*resolution

    graph_image = np.zeros((graph_width, graph_width), dtype = 'uint8')

    for i in range(graph_width):
        for j in range(graph_width):
            window = image[i*pixel_width : (i+1)*pixel_width, j*pixel_width : (j+1)*pixel_width]
            if (0 not in window) and (205 not in window):
                graph_image[i][j] = 255
            else:
                graph_image[i][j] = 0
                
    return graph_image, pixel_resolution



def createGraph(graph_image, pixel_resolution):
    
    graph = {}
    n = len(graph_image)

    straight_distance = pixel_resolution
    diagonal_distance = math.sqrt(2)*pixel_resolution

    for i in range(n):
        for j in range(n):

            neighbours = []

            if j != n-1:
                neighbours.append((i, j+1, straight_distance))
            if (i != 0) and (j != n-1):
                neighbours.append((i-1, j+1, diagonal_distance))
            if i != 0:
                neighbours.append((i-1, j, straight_distance))
            if (i != 0) and (j != 0):
                neighbours.append((i-1, j-1, diagonal_distance))
            if j != 0:
                neighbours.append((i, j-1, straight_distance))
            if (i != n-1) and (j != 0):
                neighbours.append((i+1, j-1, diagonal_distance))
            if i != n-1:
                neighbours.append((i+1, j, straight_distance))
            if (i != n-1) and (j != n-1):
                neighbours.append((i+1, j+1, diagonal_distance))

            open_neighbours = []

            for neighbour in neighbours:
                if graph_image[neighbour[0], neighbour[1]] == 255:
                    open_neighbours.append(neighbour)

            graph[(i, j)] = open_neighbours
            
    return graph



def dijkstra(graph, graph_width, start = (0, 0)):
    visited = np.full((graph_width, graph_width), False)
    distance = np.full((graph_width, graph_width), np.inf)
    previous = {}
    distance[start] = 0
    queue = BinaryHeap()
    queue.insert((start, 0))
    while queue.count != 0:
        index, minDist = queue.poll()
        visited[index] = True
        if distance[index] < minDist:
            continue
        for neighbour in graph[index]:
            if visited[neighbour[0], neighbour[1]]:
                continue
            newDist = distance[index] + neighbour[2]
            if newDist < distance[neighbour[0], neighbour[1]]:
                previous[(neighbour[0], neighbour[1])] = index
                distance[neighbour[0], neighbour[1]] = newDist
                queue.insert(((neighbour[0], neighbour[1]), newDist))
                
    return distance, previous



def findShortestPath(graph, image_size, start_coord, end_coord):
    distance, previous = dijkstra(graph, image_size, start_coord)
    path = []
    if distance[end_coord] == np.inf:
        return []
    path.append(end_coord)
    last_at = end_coord
    at = previous[end_coord]
    while previous[at] != start_coord:
        x0, y0 = last_at[0], last_at[1]
        x1, y1 = at[0], at[1]
        x2, y2 = previous[at][0], previous[at][1]
        if (x1 - x0):
            m1 = (y1 - y0)/(x1 - x0)
        else:
            m1 = "inf"
        if (x2 - x1):
            m2 = (y2 - y1)/(x2 - x1)
        else:
            m2 = "inf"
        if m1 != m2:
            path.append(at)
        at = previous[at]
        last_at = at
    #path.append(start_coord)
    path.reverse()
    
    return path

def plan_returning_path(battery_over):

    global start, end
    global transformation
    global resolution, pixel_resolution
    global start_sub

    if battery_over.data == True:
        os.system("rosrun map_server map_saver -f /home/failedmesh/catkin_ws/src/Micromouse-Navigation/rover_sim/scripts/latest_map")
        #os.system("killall -9 rviz")
        #os.system("roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/failedmesh/latest_map.yaml")
        
        #Import saved map:
        map = cv2.imread("latest_map.pgm", -1)
        resolution = 0.05

        #Dimensions in metres:
        rover_length = 0.03
        rover_width = 0.03
        breathing_room = 0.02

        #Getting path:
        graph_image, pixel_resolution = binarize(map, resolution, rover_length, rover_width, breathing_room)
        graph = createGraph(graph_image, pixel_resolution)


        print(start, end)
        path = findShortestPath(graph, graph_image.shape[0], start, end)
        factor = resolution/pixel_resolution
        inverse_transform = np.linalg.inv(transform)

        path_on_map = path_array()
        path_image = graph_image
        if len(path) != 0:
            for point in path:
                path_image[point[0], point[1]] = 150
                image_pos = np.array([[point[0]], [point[1]], [factor*20]])
                #print(image_pos.shape, inverse_transform.shape)
                pos_on_map = np.dot(inverse_transform, image_pos)
                point_on_map = float_array([pos_on_map[0][0], pos_on_map[1][0]])
                path_on_map.path.append(point_on_map)

        print("Path found")

        while path_pub.get_num_connections() == 0:
            continue
        path_pub.publish(path_on_map)
        print(path_on_map)

        cv2.imwrite("path_image.jpg", path_image)

def receive_odometry(pose):

    global start
    global transformation, transform, resolution, pixel_resolution

    x = pose.x
    y = pose.y
    #print(x, y)
    factor = resolution/pixel_resolution
    map_pos = np.array([[x], [y], [1]])
    transform = np.multiply(transformation, factor)
    pos_on_image = np.dot(transform, map_pos)
    x = int(round(pos_on_image[0][0]))
    y = int(round(pos_on_image[1][0]))
    #print("changed start coordinate")
    start = (x, y)
    #print(start)

def charging_point(pose):

    global end
    global transformation, pixel_resolution, transform
    print(pose)
    x = pose.x
    y = pose.y
    factor = resolution/pixel_resolution
    map_pos = np.array([[x], [y], [1]])
    transform = np.multiply(transformation, factor)
    pos_on_image = np.dot(transform, map_pos)
    x = int(round(pos_on_image[0][0]))
    y = int(round(pos_on_image[1][0]))
    end = (x, y)
    print(end)



if __name__ == '__main__':

    #Global Variables:
    resolution = 0.05
    pixel_resolution = 0.05
    start = (0, 0)
    end = (192, 192)
    transformation = np.array([[0, -20, 183], [20, 0, 192], [0, 0, 20]])
    print("initialized")
    path_pub = rospy.Publisher('/path', path_array, queue_size = 100)

    rospy.init_node('path_planner')
    end_sub = rospy.Subscriber('/end_coordinate', Pose2D, charging_point)
    start_sub = rospy.Subscriber('/start_coordinate', Pose2D, receive_odometry)
    check_battery = rospy.Subscriber('out_of_battery', Bool, plan_returning_path)
    rospy.spin()