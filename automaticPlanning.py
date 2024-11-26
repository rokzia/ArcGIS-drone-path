# -*- coding: utf-8 -*-

import arcpy
import numpy as np
import math
import heapq

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
                displayName='Start point',
                name='start',
                datatype='Point',
                parameterType='Optional',
                direction='Input'
            ),
            arcpy.Parameter(
                displayName='End point',
                name='end',
                datatype='Point',
                parameterType='Optional',
                direction='Input'
            ),  
            arcpy.Parameter(
                displayName='Radar Prediction',
                name='radar_raster',
                datatype='Raster Layer',
                parameterType='Required',
                direction='Input'
            ),    
            arcpy.Parameter(
                displayName='Prediction resolution',
                name='radar_resolution',
                datatype='GPLong',
                parameterType='Required',
                direction='Input'
            ),    
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
        startTemporary = [float(x.replace(',', '.')) for x in parameters[0].valueAsText.split(' ')]
        startPoint = arcpy.Point(startTemporary[0], startTemporary[1])

        endTemporary = [float(x.replace(',', '.')) for x in parameters[1].valueAsText.split(' ')]
        endPoint = arcpy.Point(endTemporary[0], endTemporary[1])

        self.raster = arcpy.Raster(parameters[2].valueAsText)
        self.resolution = parameters[3].value

        pointArray = arcpy.Array()

        #konfiguracija
        current_map = arcpy.mp.ArcGISProject('CURRENT').activeMap
        spatial_reference = current_map.spatialReference
        #arcpy.AddMessage(spatial_reference)
        #lineFeatureClass = arcpy.CreateFeatureclass_management(arcpy.env.workspace,"Line","POLYLINE", spatial_reference = spatial_reference)

        boolArray, startIndexes, endIndexes = self.createBoolArray(startPoint, endPoint)
        arcpy.AddMessage(boolArray)

        path = a_star(boolArray, startIndexes, endIndexes)
        arcpy.AddMessage(path)

        #current_map.addDataFromPath(lineFeatureClass)
        return

    def postExecute(self, parameters):
        """This method takes place after outputs are processed and
        added to the display."""
        return
    

    def createBoolArray(self, start: arcpy.Point, end: arcpy.Point):
        #starting width and height
        width = abs(start.X - end.X)
        height = abs(start.Y - end.Y)
        numberOfColumns = math.ceil(width / self.resolution)
        numberOfRows = math.ceil(height / self.resolution)
        paddingColumns = math.ceil(0.5 * width / self.resolution)
        paddingRrows = math.ceil(0.5 * height / self.resolution)
        numberOfColumns += paddingColumns * 2
        numberOfRows += paddingRrows * 2
        start.X = start.X - paddingColumns * self.resolution
        start.Y = start.Y - paddingRrows * self.resolution

        # compute start and end point indexes for further calculations
        originalStartIndex = (numberOfRows - paddingRrows - 1, paddingColumns)
        originalEndIndex = (paddingRrows, numberOfColumns - paddingColumns - 1)

        startArray = arcpy.RasterToNumPyArray(self.raster, start, numberOfColumns, numberOfRows)

        return startArray > -110, originalStartIndex, originalEndIndex

def heuristic(a, b):
    # Manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

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
