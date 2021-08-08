import rospy
import numpy as np
import math
import os
import cv2
from std_msgs.msg import Bool
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



def findShortestPath(graph, image_size, start, end):
    distance, previous = dijkstra(graph, image_size, start)
    path = []
    if distance[end] == np.inf:
        return None
    at = end
    while previous[at] != start:
        path.append(at)
        at = previous[at]
    path.append(start)
    path.reverse()
    
    return path

def plan_returning_path(battery_over):
    if battery_over.data == True:
        os.system("rosrun map_server map_saver -f /home/failedmesh/catkin_ws/src/Micromouse-Navigation/rover_sim/scripts/latest_map")
        #os.system("killall -9 rviz")
        #os.system("roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/failedmesh/latest_map.yaml")
        
        #Import saved map:
        map = cv2.imread("latest_map.pgm", -1)
        resolution = 0.05

        #Dimensions in metres:
        rover_length = 0.08
        rover_width = 0.08
        breathing_room = 0.02

        #Getting path:
        graph_image, pixel_resolution = binarize(map, resolution, rover_length, rover_width, breathing_room)
        graph = createGraph(graph_image, pixel_resolution)
        start = (100, 100)
        end = (55, 89)
        path = findShortestPath(graph, graph_image.shape[0], start, end)
        


if __name__ == '__main__':
    rospy.init_node('path_planner')
    check_battery = rospy.Subscriber('out_of_battery', Bool, plan_returning_path)
    rospy.spin()