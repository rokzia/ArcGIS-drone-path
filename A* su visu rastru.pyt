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


        boolArray, startIndexes, endIndexes = self.createBoolArray(startPoint, endPoint)
        path = a_star(boolArray, startIndexes, endIndexes)

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
    

    def createBoolArray(self, start, end):
        array = arcpy.RasterToNumPyArray(self.radar, nodata_to_value=-200)
        startIndex = self.snapPointToRasterCenter(start)
        endIndex = self.snapPointToRasterCenter(end)
        return array>-110, startIndex, endIndex
    
    def createPointArray(self, start: arcpy.Point, path: list, startIndexes):
        pointArray = arcpy.Array()
        tempPoint = copy.copy(start)
        prevY, prevX = startIndexes
        for i in range(len(path)):
            y, x = path[i]
            if y - prevY == 1: #down
                tempPoint = arcpy.Point(tempPoint.X, tempPoint.Y - self.resolution)
            elif y - prevY == -1: #up
                tempPoint = arcpy.Point(tempPoint.X, tempPoint.Y + self.resolution)
            elif x - prevX == 1: #right
                tempPoint = arcpy.Point(tempPoint.X + self.resolution, tempPoint.Y)
            elif x - prevX == -1: #left
                tempPoint = arcpy.Point(tempPoint.X - self.resolution, tempPoint.Y)
            arcpy.AddMessage(f'{tempPoint.X} {tempPoint.Y}')
            pointArray.add(tempPoint)
            prevY, prevX = path[i]
        
        return pointArray

    def snapPointToRasterCenter(self, point: arcpy.Point):
        rasterOriginX = self.radar.extent.XMin
        rasterOriginY = self.radar.extent.YMax 
        

        col = int((point.X - rasterOriginX) / self.resolution)
        row = int((rasterOriginY - point.Y) / self.resolution)
        

        return row, col

def a_star(grid, start, end):
    rows, cols = grid.shape
    open_set = []
    heapq.heappush(open_set, (0, start))  # (f, node)
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, end)}
    visited = set()

    while open_set:
        _, current = heapq.heappop(open_set)

        if current in visited:
            continue
        visited.add(current)



        if current == end:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        # Explore neighbors
        neighbors = [
            (current[0] + dx, current[1] + dy)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
        ]
        for neighbor in neighbors:

            if (
                0 <= neighbor[0] < rows and
                0 <= neighbor[1] < cols and
                not grid[neighbor] and
                neighbor not in visited
            ):
                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found


    
def heuristic(a, b):
# Manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])
