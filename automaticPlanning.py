# -*- coding: utf-8 -*-

import arcpy
import numpy as np
import math
import heapq
import copy

class Toolbox:
    def __init__(self):
        """Define the toolbox (the name of the toolbox is the name of the
        .pyt file)."""
        self.label = "Toolbox"
        self.alias = "toolbox"

        # List of tool classes associated with this toolbox
        self.tools = [Tool]


class Tool:
    def __init__(self):
        """Define the tool (tool name is the name of the class)."""
        self.label = "Tool"
        self.description = ""

    def getParameterInfo(self):
        """Define the tool parameters."""
        params = [
            arcpy.Parameter(
                displayName='Waypoints',
                name='waypoints',
                datatype='GPFeatureRecordSetLayer',
                parameterType='Optional',
                direction='Input'
            ),  
            arcpy.Parameter(
                displayName='Radar Prediction',
                name='radar_raster',
                datatype='Raster Layer',
                parameterType='Required',
                direction='Input'
            )

        ]
        return params

    def isLicensed(self):
        """Set whether the tool is licensed to execute."""
        return True

    def updateParameters(self, parameters):
        """Modify the values and properties of parameters before internal
        validation is performed.  This method is called whenever a parameter
        has been changed."""
        return

    def updateMessages(self, parameters):
        """Modify the messages created by internal validation for each tool
        parameter. This method is called after internal validation."""
        return

    def execute(self, parameters, messages):
        waypoints = parameters[0].value
        tempArr = []
        with arcpy.da.SearchCursor(waypoints, ["SHAPE@"]) as cursor:
            for row in cursor:
                point = arcpy.Point(row[0].centroid.X, row[0].centroid.Y)
                tempArr.append(point)


        
        self.radar = arcpy.Raster(parameters[1].valueAsText)
        self.resolution = self.radar.meanCellWidth
        pointArray = arcpy.Array()

        #konfiguracija
        current_map = arcpy.mp.ArcGISProject('CURRENT').activeMap
        spatial_reference = current_map.spatialReference
        lineFeatureClass = arcpy.CreateFeatureclass_management(arcpy.env.workspace,"Line","POLYLINE", spatial_reference = spatial_reference)
        pointFeatureClass = arcpy.CreateFeatureclass_management(arcpy.env.workspace,"Points","POINT", spatial_reference = spatial_reference)

        startPoint = tempArr[0]
        endPoint = tempArr[1]

        weight = 155 #The amount radar values effect path judgement

        radarValues, startIndexes, endIndexes = self.radarValueArray(startPoint, endPoint)
        path = theta_star(radarValues, startIndexes, endIndexes, weight)

        if path is None:
            arcpy.AddMessage("No path found")
            return
        arcpy.AddMessage(path)

        pointArray = self.createPointArray(startPoint, path, startIndexes)

        with arcpy.da.InsertCursor("Points", ["SHAPE@"]) as cursor:
            for point in pointArray:
                cursor.insertRow([point])

        line = arcpy.Polyline(pointArray, spatial_reference = spatial_reference)

        with arcpy.da.InsertCursor("Line", ["SHAPE@"]) as cursor:
            cursor.insertRow([line])

        current_map.addDataFromPath(lineFeatureClass)
        current_map.addDataFromPath(pointFeatureClass)
        return

    def postExecute(self, parameters):
        """This method takes place after outputs are processed and
        added to the display."""
        return
    

    def radarValueArray(self, start, end):
        array = arcpy.RasterToNumPyArray(self.radar, nodata_to_value=-200)
        startIndex = self.snapPointToRasterCenter(start)
        endIndex = self.snapPointToRasterCenter(end)
        return array, startIndex, endIndex
    
    def createPointArray(self, start: arcpy.Point, path: list, startIndexes):
        pointArray = arcpy.Array()
        pointArray.add(start)
        tempX = start.X
        tempY = start.Y
        prevY, prevX = startIndexes
        for i in range(len(path)):
            y, x = path[i]
            if y - prevY >= 1: #down
                tempY -= self.resolution * (y - prevY)
            elif y - prevY <= -1: #up
                tempY += self.resolution * abs((y - prevY))
            if x - prevX >= 1: #right
                tempX += self.resolution * (x - prevX)
            elif x - prevX <= -1: #left
                tempX -= self.resolution * abs((x - prevX))

            tempPoint = arcpy.Point(tempX,tempY)
            pointArray.add(tempPoint)
            prevY, prevX = path[i]
        
        return pointArray

    def snapPointToRasterCenter(self, point: arcpy.Point):
        rasterOriginX = self.radar.extent.XMin
        rasterOriginY = self.radar.extent.YMax 
        

        col = int((point.X - rasterOriginX) / self.resolution)
        row = int((rasterOriginY - point.Y) / self.resolution)
        

        return row, col

def theta_star(grid, start, end, weight):
    rows, cols = grid.shape
    open_set = []
    heapq.heappush(open_set, (0, start))  # (f, node)
    came_from = {start: start}
    g_score = {start: 0}
    f_score = {start: heuristic(start,end)}
    visited = set()

    while open_set:
        _, current = heapq.heappop(open_set)

        if current in visited:
            continue
        visited.add(current)



        if current == end:
            # Reconstruct path
            path = []
            while current != came_from[current]:
                path.append(current)
                current = came_from[current]
            return pathSmoothing(path[::-1],grid)

        # Explore neighbors
        neighbors = [
            (current[0] + dx, current[1] + dy)
            # for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (1,1), (-1,-1), (-1,1), (1,-1)]
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
        ]
        for neighbor in neighbors:

            if (
                0 <= neighbor[0] < rows and
                0 <= neighbor[1] < cols and
                neighbor not in visited
            ):
                if neighbor not in open_set:
                    g_score[neighbor] = float('inf')
                    came_from[neighbor] = None
                update_vertex(current,neighbor,g_score,came_from,open_set,end,grid, f_score, weight)

    return None  # No path found

def update_vertex(current, neighbor, g_score, parent, open_set, finish, grid, f_score, weight):
    new_g = g_score[current] + heuristic(current,neighbor) + weight*detectionCost(grid[neighbor])
    if new_g < g_score[neighbor]:
        g_score[neighbor] = new_g
        f_score[neighbor] = new_g + heuristic(neighbor,finish)
        parent[neighbor] = current
        if neighbor in open_set:
            open_set.remove(neighbor)
        heapq.heappush(open_set, (f_score[neighbor], neighbor))


def pathSmoothing(path, grid):
    parentIndex = 0
    resultPath = []
    resultPath.append(path[0])
    for i in range(0,len(path)-1):
        if line_of_sight(resultPath[parentIndex], path[i+1], grid) == False:
            parentIndex += 1
            resultPath.append(path[i])
    resultPath.append(path[len(path)-1])
    return resultPath


#Method used to find if there exists a line between two points that meets a criteria
def line_of_sight(point1, point2, grid):

    x0, y0 = point1
    x1, y1 = point2

    gridValue = grid[(x0,y0)]

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    sX = 1 if x1 > x0 else -1
    sY = 1 if y1 > y0 else -1

    if dx > dy:
        err = dx / 2
        while x0 != x1:
            if grid[(x0, y0)] > gridValue:
                return False
            err -= dy
            if err < 0:
                y0 += sY
                err += dx
            x0 += sX
    else:
        err = dy / 2
        while y0 != y1:
            if grid[(x0, y0)] > gridValue:
                return False
            err -= dx
            if err < 0:
                x0 += sX
                err += dy
            y0 += sY
    return True

def heuristic(a, b):
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2) ** 0.5

def manhattan(a,b):
# Manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def detectionCost(fieldStrength):
    return (fieldStrength+200)/200


