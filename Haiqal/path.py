import math
import numpy as np
import matplotlib.pyplot as plt
import json
from collections import namedtuple
from operator import itemgetter
from pprint import pformat

path = []
distanceDict = {}
coordNamesAssgn = []
coordDict = {}
pointX = []
pointY = []
pointName = []

toDelArr = []

pointX.append(float(0.00))
pointY.append(float(0.00))


def pointCleanUp():
    coordFile = open("coordinates.txt", "r")
    for line in coordFile:
        values = line.split()
        pointX.append(float(values[0]))
        pointY.append(float(values[1]))
    for i in range(len(pointX)):
        xAxis = pointX[i]
        yAxis = pointY[i]

        xLimit = xAxis+0.1
        yLimit = yAxis+0.1
        xMin = xAxis-0.1
        yMin = yAxis-0.1
        for j in range(len(pointX)):
            if i != j:
                xPoint = pointX[j]
                yPoint = pointY[j]
                if xPoint<xLimit and xPoint>xMin and yPoint<yLimit and yPoint>yMin:
                    toDelArr.append(j)
    for i in toDelArr:
        del pointX[i]
        del pointY[i]

def plotPoints():
    plt.scatter(pointX, pointY, color="black")

    counterAlpha = 0
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
    # Compile key names
    for k in coordDict:
        objectFromDict = coordDict[k]
        coordX = objectFromDict["coordX"]
        coordY = objectFromDict["coordY"]

        collDict = {}
        xMax = coordX+0.15
        xMin = coordX-0.15
        yMax = coordY+1
        yMin = coordY-1
        for j in coordDict:
            consequentObject = coordDict[j]
            nextCoordX = consequentObject["coordX"]
            nextCoordY = consequentObject["coordY"]
            if nextCoordX < xMax and nextCoordX > xMin and nextCoordY > yMin and nextCoordY < yMax:
                distance = calcDistance(
                    [coordX, coordY], [nextCoordX, nextCoordY])
                collDict[j] = distance
        distanceDict[k] = collDict


def dijkstra(graph, src, dest, visited=[], distances={}, predecessors={}):
    # a few sanity checks
    if src not in graph:
        raise TypeError('The root of the shortest path tree cannot be found')
    if dest not in graph:
        raise TypeError('The target of the shortest path cannot be found')
    # ending condition
    if src == dest:
        # We build the shortest path and display it
        path = []
        pred = dest
        while pred != None:
            path.append(pred)
            pred = predecessors.get(pred, None)
        # reverses the array, to display the path nicely
        readable = path[0]
        for index in range(1, len(path)):
            readable = path[index]+'--->'+readable
        path = path[::-1]
        # prints it
        for points in path:
            coordX = coordDict[points]["coordX"]
            coordY = coordDict[points]["coordY"]
            plt.plot(coordX, coordY, 'ro-')
        #print("path: "+readable+",   cost="+str(distances[dest]))
    else:
        # if it is the initial  run, initializes the cost
        if not visited:
            distances[src] = 0
        # visit the neighbors
        for neighbor in graph[src]:
            if neighbor not in visited:
                new_distance = distances[src] + graph[src][neighbor]
                if new_distance < distances.get(neighbor, float('inf')):
                    distances[neighbor] = new_distance
                    predecessors[neighbor] = src
        # mark as visited
        visited.append(src)
        # now that all neighbors have been visited: recurse
        # select the non visited node with lowest distance 'x'
        # run Dijskstra with src='x'
        unvisited = {}
        for k in graph:
            if k not in visited:
                unvisited[k] = distances.get(k, float('inf'))
        x = min(unvisited, key=unvisited.get)
        dijkstra(graph, x, dest, visited, distances, predecessors)


def main():
    pointCleanUp()
    plotPoints()
    generateDistance()
    plt.show()
    destPoint = input("Enter destination point: ")
    dijkstra(distanceDict, 'A0', destPoint)
    plt.show()
    pass


main()
