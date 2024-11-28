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
        spatial_reference = arcpy.SpatialReference(3346)
        lineFeatureClass = arcpy.CreateFeatureclass_management(arcpy.env.workspace,"Line","POLYLINE", spatial_reference = spatial_reference)
        pointFeatureClass = arcpy.CreateFeatureclass_management(arcpy.env.workspace,"Points","POINT", spatial_reference = spatial_reference)

        


        boolArray, startIndexes, endIndexes = self.createBoolArray(startPoint, endPoint)
        arcpy.AddMessage(boolArray)
        arcpy.AddMessage(startIndexes)
        arcpy.AddMessage(endIndexes)

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
    

    def createBoolArray(self, start: arcpy.Point, end: arcpy.Point):
        start = snapPointToRasterCenter(self, start)
        end = snapPointToRasterCenter(self, end)
        width = abs(start.X - end.X)
        arcpy.AddMessage(f'Width: {width}')
        height = abs(start.Y - end.Y)
        arcpy.AddMessage(f'Height: {height}')
        numberOfColumns = math.ceil(width / self.resolution) + 1
        arcpy.AddMessage(f'Number of columns: {numberOfColumns}')
        numberOfRows = math.ceil(height / self.resolution) + 1
        testArray = arcpy.RasterToNumPyArray(self.raster, start, numberOfColumns, numberOfRows)
        arcpy.AddMessage(testArray)
        arcpy.AddMessage(f'Number of rows: {numberOfRows}')
        paddingColumns = math.ceil(width / self.resolution)
        arcpy.AddMessage(f'Padding columns: {paddingColumns}')
        paddingRows = math.ceil(height / self.resolution)
        arcpy.AddMessage(f'Padding rows: {paddingRows}')
        numberOfColumns += paddingColumns * 2
        numberOfRows += paddingRows * 2

        # compute start and end point indexes for further calculations
        if start.X < end.X and start.Y < end.Y: # from bottom left to top right
            newStart = arcpy.Point(start.X - paddingColumns * self.resolution, start.Y - paddingRows * self.resolution)
            newEnd = arcpy.Point(end.X + paddingColumns * self.resolution, end.Y + paddingRows * self.resolution)
            originalStartIndex = (numberOfRows - paddingRows - 1, paddingColumns)
            originalEndIndex = (paddingRows, numberOfColumns - paddingColumns - 1)
            startArray = arcpy.RasterToNumPyArray(self.raster, newStart, numberOfColumns, numberOfRows)
        elif start.X < end.X and start.Y > end.Y: # from top left to bottom right
            newStart = arcpy.Point(start.X - paddingColumns * self.resolution, start.Y + paddingRows * self.resolution)
            newEnd = arcpy.Point(end.X + paddingColumns * self.resolution, end.Y - paddingRows * self.resolution)
            originalStartIndex = (paddingRows, paddingColumns)
            originalEndIndex = (numberOfRows - paddingRows - 1, numberOfColumns - paddingColumns - 1)
            startArray = arcpy.RasterToNumPyArray(self.raster, arcpy.Point(newStart.X, newEnd.Y), numberOfColumns, numberOfRows)
        elif start.X > end.X and start.Y > end.Y: # from top right to bottom left
            newEnd = arcpy.Point(end.X - paddingColumns * self.resolution, end.Y - paddingRows * self.resolution)
            originalStartIndex = (paddingRows, numberOfColumns - paddingColumns - 1)
            originalEndIndex = (numberOfRows - paddingRows - 1, paddingColumns)
            startArray = arcpy.RasterToNumPyArray(self.raster, newEnd, numberOfColumns, numberOfRows)
        else: # from bottom right to top left
            newStart = arcpy.Point(start.X + paddingColumns * self.resolution, start.Y - paddingRows * self.resolution)
            newEnd = arcpy.Point(end.X - paddingColumns * self.resolution, end.Y + paddingRows * self.resolution)
            originalStartIndex = (numberOfRows - paddingRows - 1, numberOfColumns - paddingColumns - 1)
            originalEndIndex = (paddingRows, paddingColumns)
            startArray = arcpy.RasterToNumPyArray(self.raster, arcpy.Point(newEnd.X, newStart.Y), numberOfColumns, numberOfRows)
        

        

        return startArray > -110, originalStartIndex, originalEndIndex
    
    def createPointArray(self, start: arcpy.Point, path: list, startIndexes):
        pointArray = arcpy.Array()
        tempPoint = copy.copy(start)
        pointArray.add(start)
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
            
            pointArray.add(tempPoint)
            prevY, prevX = path[i]
        
        return pointArray


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

def snapPointToRasterCenter(self, point: arcpy.Point):
    #lower left corner of raster
    rasterOriginX, rasterOriginY = self.raster.extent.XMin, self.raster.extent.YMax
    #cell indexes of start point
    col = int((point.X - rasterOriginX) / self.resolution)
    row = int((rasterOriginY - point.Y) / self.resolution)
    snapped_x = rasterOriginX + col * self.resolution + self.resolution / 2 - 100
    snapped_y = rasterOriginY - row * self.resolution - self.resolution / 2 - 100

    return arcpy.Point(snapped_x, snapped_y)
