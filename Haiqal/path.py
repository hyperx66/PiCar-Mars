import math
import numpy as np
import matplotlib.pyplot as plt
import json
from collections import namedtuple
from operator import index, itemgetter
from pprint import pformat
import numpy
import time
from shapely.geometry import Point, Polygon
import copy

path = []
distanceDict = {}
coordNamesAssgn = []
coordDict = {}
pointX = []
pointY = []
pointStatus = []
pointName = []
pointXDelete = []
pointYDelete = []
instructArr = []
delPointX = []
delPointY = []

toDelArr = np.array([])

def pointCleanUp():
    global pointXDelete
    global pointYDelete
    coordFile = open("./coordinates.txt", "r")
    for line in coordFile:
        values = line.split()
        if int(values[2]) != -1:
            pointX.append(float(values[0]))
            pointY.append(float(values[1]))

    for i in range(len(pointX)):
        xAxis = pointX[i]
        yAxis = pointY[i]

        xLimit = xAxis+0.03
        yLimit = yAxis+0.03
        xMin = xAxis-0.03
        yMin = yAxis-0.03

        if xAxis not in pointXDelete and yAxis not in pointYDelete:
            for j in range(len(pointX)):
                if i != j:
                    xPoint = pointX[j]
                    yPoint = pointY[j]
                    if xAxis not in pointXDelete and yAxis not in pointYDelete:
                        if xPoint>=xMin and xPoint<=xLimit and yPoint>=yMin and yPoint<=yLimit:
                            pointXDelete.append(xPoint)
                            pointYDelete.append(yPoint)

    for j in range(len(pointXDelete)):
        toDelX = pointXDelete[j]
        toDelY = pointYDelete[j]
        for k in range(len(pointX)):
            xItem = pointX[k]
            yItem = pointY[k]
            if xItem == toDelX and yItem == toDelY:
                del pointX[k]
                del pointY[k]
                break
    pointXDelete = []
    pointYDelete = []

# def clear():
#     coordFile = open("C:\\Users\\ASUS\\Downloads\\APA LANJIAO\\Workspace\\test.txt", "r")
#     clear = open("C:\\Users\\ASUS\\Downloads\\APA LANJIAO\\Workspace\\new.txt", "w")

#     for line in coordFile:
#         values = line.split()
#         clear.write(values[0] + " " + values[1] + "\n")
#         pointX.append(values[0])
#         pointY.append(values[1])
#     print(len(pointX))
#     print(len(pointY))

def clearPoints():
    coords = [(-0.650, 0.599),(2.654, 0.498),(2.590, -0.581),(-0.650, -0.520),(-0.650, 0.599)] #play with these coordinates 
    # coords = [[-1.167, 8.35],[5.141, 8.16],[5.141, -2],[-1.167, -2.15],[-1.167, 8.35]]
    poly = Polygon(coords)
    # print(Point(-0.216998, 0.892396).within(poly))

    # readPoints = open("C:\\Users\\ASUS\\Downloads\\APA LANJIAO\\Workspace\\new.txt", "r")
    for pX, pY in zip(pointX, pointY):
        # split = point.split(" ")
        p1 = Point(float(pX),float(pY))
        if (p1.within(poly)):
            # print("true")
            delPointX.append(pX)
            delPointY.append(pY)
        # else:
        #     clearedPointX.append(split[0])
        #     clearedPointY.append(split[1])
    for j in range(len(delPointX)):
        toDelX = delPointX[j]
        toDelY = delPointY[j]
        for k in range(len(pointX)-1):
            xItem = pointX[k]
            yItem = pointY[k]
            # print(xItem)
            if xItem == toDelX and yItem == toDelY:
                del pointX[k]
                del pointY[k]
    
    # print(pointX)
    
def cleanOutliers():
    # topMost = int(((input("Identifier for top most point: ")).split("A"))[1])
    # bottomMost = int(((input("Identifier for bottom most point: ")).split("A"))[1])
    # leftMost = int(((input("Identifier for left most point: ")).split("A"))[1])
    # rightMost = int(((input("Identifier for right most point: ")).split("A"))[1])

    print("Cleaning outliers...")
    # highestPoint = pointY[topMost]
    # lowestPoint = pointY[bottomMost]
    # leftPoint = pointX[leftMost]
    # rightPoint = pointX[rightMost]
    highestPoint = 0.642
    lowestPoint = -0.724
    leftPoint = -0.888
    rightPoint = 2.743

    for i in range(len(pointX)):
        xAxis = pointX[i]
        yAxis = pointY[i]

        if lowestPoint>yAxis or yAxis>highestPoint or leftPoint>xAxis or xAxis>rightPoint:
            pointXDelete.append(xAxis)
            pointYDelete.append(yAxis)

    for j in range(len(pointXDelete)):
        toDelX = pointXDelete[j]
        toDelY = pointYDelete[j]
        for k in range(len(pointX)):
            xItem = pointX[k]
            yItem = pointY[k]
            if xItem == toDelX or yItem == toDelY:
                del pointX[k]
                del pointY[k]
                break
    

def plotPoints():
    global coordNamesAssgn, coordDict
    plt.scatter(pointX, pointY, color="black")
    counterAlpha = 0
    coordNamesAssgn = []
    coordDict = {}
    for i in range(len(pointX)):
        coordDict['A'+str(counterAlpha)
                  ] = {'coordX': pointX[i], 'coordY': pointY[i]}
        coordNamesAssgn.append('A'+str(counterAlpha))
        counterAlpha += 1

    for i, txt in enumerate(coordNamesAssgn):
        plt.annotate(txt, (pointX[i], pointY[i]))


def calcDistance(coord1, coord2):
    distance = math.sqrt(((coord1[0]-coord2[0])**2)+((coord1[1]-coord2[1])**2))
    return distance


def generateDistance():
    global distanceDict
    # Compile key names
    for k in coordDict:
        objectFromDict = coordDict[k]
        coordX = objectFromDict["coordX"]
        coordY = objectFromDict["coordY"]

        collDict = {}
        xMax = coordX+0.7
        xMin = coordX-0.7
        yMax = coordY+0.4
        yMin = coordY-0.4
        for j in coordDict:
            if j!=k:
                consequentObject = coordDict[j]
                nextCoordX = consequentObject["coordX"]
                nextCoordY = consequentObject["coordY"]
                if nextCoordX < xMax and nextCoordX > xMin and nextCoordY > yMin and nextCoordY < yMax:
                    distance = calcDistance([coordX, coordY], [nextCoordX, nextCoordY])
                    collDict[j] = distance
        distanceDict[k] = collDict


# def dijkstra(graph, src, dest, visited=[], distances={}, predecessors={}):
#     # a few sanity checks
#     if src not in graph:
#         raise TypeError('The root of the shortest path tree cannot be found')
#     if dest not in graph:
#         raise TypeError('The target of the shortest path cannot be found')
#     # ending condition
#     if src == dest:
#         # We build the shortest path and display it
#         path = []
#         pred = dest
#         while pred != None:
#             path.append(pred)
#             pred = predecessors.get(pred, None)
#         # reverses the array, to display the path nicely
#         readable = path[0]
#         for index in range(1, len(path)):
#             readable = path[index]+'--->'+readable
#         path = path[::-1]
#         # prints it
#         for points in path:
#             coordX = coordDict[points]["coordX"]
#             coordY = coordDict[points]["coordY"]
#             plt.plot(coordX, coordY, 'ro-')
#         return distances[dest]
#         #print("path: "+readable+",   cost="+str(distances[dest]))
#     else:
#         # if it is the initial  run, initializes the cost
#         if not visited:
#             distances[src] = 0
#         # visit the neighbors
#         for neighbor in graph[src]:
#             if neighbor not in visited:
#                 new_distance = distances[src] + graph[src][neighbor]
#                 if new_distance < distances.get(neighbor, float('inf')):
#                     distances[neighbor] = new_distance
#                     predecessors[neighbor] = src
#         # mark as visited
#         visited.append(src)
#         # now that all neighbors have been visited: recurse
#         # select the non visited node with lowest distance 'x'
#         # run Dijskstra with src='x'
#         unvisited = {}
#         for k in graph:
#             if k not in visited:
#                 unvisited[k] = distances.get(k, float('inf'))
#         x = min(unvisited, key=unvisited.get)
#         dijkstra(graph, x, dest, visited, distances, predecessors)

def Dijkstra(graph,source,target):
    # These are all the nodes which have not been visited yet
    unvisited_nodes=dict.copy(graph)
    # It will store the shortest distance from one node to another
    shortest_distance={}
    # This will store the Shortest path between source and target node 
    route=[] 
    # It will store the predecessors of the nodes
    predecessor={}
    
    # Iterating through all the unvisited nodes
    for nodes in unvisited_nodes:
        
    # Setting the shortest_distance of all the nodes as infinty
        shortest_distance[nodes]=math.inf
        
    # The distance of a point to itself is 0.
    shortest_distance[source]=0
    
    # Running the loop while all the nodes have been visited
    while(unvisited_nodes):
        
        # setting the value of min_node as None
        min_Node=None
        
        # iterating through all the unvisited node
        for current_node in unvisited_nodes: 
            
        # For the very first time that loop runs this will be called
            if min_Node is None:
            
            # Setting the value of min_Node as the current node
                min_Node=current_node
                
            elif shortest_distance[min_Node] > shortest_distance[current_node]:
                
            # I the value of min_Node is less than that of current_node, set 
            #min_Node as current_node

                min_Node=current_node
                
        # Iterating through the connected nodes of current_node (for # example, a is connected with b and c having values 10 and 3 
        # respectively) and the weight of the edges

        for child_node,value in unvisited_nodes[min_Node].items():

            # checking if the value of the current_node + value of the edge 
            # that connects this neighbor node with current_node
            # is lesser than the value that distance between current nodes 
            # and its connections
            if value + shortest_distance[min_Node] < shortest_distance[child_node]:  
                
     # If true  set the new value as the minimum distance of that connection
                shortest_distance[child_node] = value + shortest_distance[min_Node]
                
           # Adding the current node as the predecessor of the child node
                predecessor[child_node] = min_Node
        
        # After the node has been visited (also known as relaxed) remove it from unvisited node
        unvisited_nodes.pop(min_Node)
        
    # Till now the shortest distance between the source node and target node 
    # has been found. Set the current node as the target node 
    node = target
    
    # Starting from the goal node, we will go back to the source node and 
# see what path we followed to get the smallest distance
    while node != source:
        
        # As it is not necessary that the target node can be reached from # the source node, we must enclose it in a try block
        try:
            route.insert(0,node)
            node = predecessor[node]
        except Exception:
            print('Path not reachable')
            break
    # Including the ssource in the path
    route.insert(0,source)
    
    # If the node has been visited,
    if shortest_distance[target] != math.inf:
        # print the shortest distance and the path taken
        print('Shortest distance is ' + str(shortest_distance[target]))
        print('And the path is ' + str(route))
        return shortest_distance[target]

def instructGen(startPoint, destPoint1, destPoint2, destPoint3):
    global instructArr

    #Pi-Car covers 1.16m in 1 sec at full speed
    print("Finding path...")
    cost1 = Dijkstra(distanceDict, startPoint, destPoint1)
    cost2 = Dijkstra(distanceDict, destPoint1, destPoint2)
    cost3 = Dijkstra(distanceDict, destPoint2, destPoint3)
    cost4 = Dijkstra(distanceDict, destPoint3, startPoint)

    print("Calculating time cost...")

    #Find time for robot to cover distance using DST formula
    time1 = round(((cost1*3.24868733967855)/1.16),2)
    time2 = round(((cost2*1.1404693999125)/1.16),2)
    time3 = round(((cost3*3.24868733967855)/1.16),2)
    time4 = round(((cost4*1.1404693999125)/1.16),2)

    print("Creating instructions...")
    instructArr.append("Forward_Left:"+str(0.7))
    instructArr.append("Take Picture:"+str(0))
    instructArr.append("Backward_Left:"+str(0.7))
    instructArr.append("Forward:"+str(time1))
    instructArr.append("Forward_Left:"+str(0.7))
    instructArr.append("Take Picture:"+str(0))
    instructArr.append("Backward_Left:"+str(0.7))
    instructArr.append("Turn_Left:"+str(time1))
    instructArr.append("Forward:"+str(time2))
    instructArr.append("Forward_Left:"+str(0.7))
    instructArr.append("Take Picture:"+str(0))
    instructArr.append("Backward_Left:"+str(0.7))
    instructArr.append("Turn_Left:"+str(time1))
    instructArr.append("Forward:"+str(time3))
    instructArr.append("Forward_Left:"+str(0.7))
    instructArr.append("Take Picture:"+str(time1))
    instructArr.append("Backward_Left:"+str(0.7))
    instructArr.append("Turn_Left:"+str(time1))
    instructArr.append("Forward:"+str(time4))
    instructArr.append("End:"+str(0))
    print("Instructions Completed...")
    return instructArr

def main():
    print("Initiating point cleanup...")
    pointCleanUp()
    print("Plotting points...")
    cleanOutliers()
    clearPoints() #clear the inner polygon
    plotPoints()
    print("Generating distance...")
    generateDistance()
    plt.show()
    startPoint = input("Enter start point:")
    destPoint1 = input("Enter destination point 1: ")
    destPoint2 = input("Enter destination point 2: ")
    destPoint3 = input("Enter destination point 3: ")
    return instructGen(startPoint, destPoint1, destPoint2, destPoint3)
main()