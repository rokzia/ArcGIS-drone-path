# -*- coding: utf-8 -*-

import arcpy
import pathlib
import json
import xml.etree.ElementTree as xml
import numpy as np
import heapq

class Toolbox:
    def __init__(self):
        """Define the toolbox (the name of the toolbox is the name of the
        .pyt file)."""
        self.label = "Manual route planning"
        self.alias = "toolbox"

        # List of tool classes associated with this toolbox
        self.tools = [Tool]


class Tool:
    def __init__(self):
        """Define the tool (tool name is the name of the class)."""
        self.label = "Manual planning script"
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
            ),
            arcpy.Parameter(
                displayName='Waypoint file',
                name='waypoint_file',
                datatype='File',
                parameterType='Optional',
                direction='Input'
            ),
            arcpy.Parameter(
                displayName='Draw auto route',
                name='drawAutoRoute',
                datatype='GPBoolean',
                parameterType='Required',
                direction='Input'
            )
        ]

        params[0].value = arcpy.FeatureSet()
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
        self.radar = arcpy.Raster(parameters[1].valueAsText)
        waypoint_file = parameters[2].value 
        drawAutoRoute = parameters[3].value

        self.resolution = self.radar.meanCellWidth

        array = arcpy.Array()

        current_map = arcpy.mp.ArcGISProject('CURRENT').activeMap
        spatial_reference = arcpy.SpatialReference(3346)
        lineFeatureClass = arcpy.CreateFeatureclass_management(arcpy.env.workspace,"Line","POLYLINE", spatial_reference = spatial_reference)
        pointFeatureClass = arcpy.CreateFeatureclass_management(arcpy.env.workspace,"Points","POINT", spatial_reference = spatial_reference)

        #Manual planning start
        if waypoint_file:
            array = self.importFromFile(waypoint_file)
        elif waypoints:
            with arcpy.da.SearchCursor(waypoints, ["SHAPE@"]) as cursor:
                for row in cursor:
                    point = arcpy.Point(row[0].centroid.X, row[0].centroid.Y)
                    array.add(point)

        Line = arcpy.Polyline(array, spatial_reference = spatial_reference)

        with arcpy.da.InsertCursor("Line", ["SHAPE@"]) as cursor:
            cursor.insertRow([Line])

        with arcpy.da.InsertCursor("Points", ["SHAPE@"]) as cursor:
           for point in array:
                cursor.insertRow([point])

        current_map.addDataFromPath(lineFeatureClass)
        current_map.addDataFromPath(pointFeatureClass)
        #Manual planning end
        if drawAutoRoute:
            #Insert auto planning route as well
            lineFeatureClass = arcpy.CreateFeatureclass_management(arcpy.env.workspace,"LineAuto","POLYLINE", spatial_reference = spatial_reference)
            pointFeatureClass = arcpy.CreateFeatureclass_management(arcpy.env.workspace,"PointsAuto","POINT", spatial_reference = spatial_reference)

            startPoint = array[0]
            endPoint = array[-1]

            boolArray, startIndexes, endIndexes = self.createBoolArray(startPoint, endPoint)
            path = theta_star(boolArray, startIndexes, endIndexes)

            if path is None:
                arcpy.AddMessage("No path found")
                return
            else:
                arcpy.AddMessage(path)

            array = self.createPointArray(startPoint, path, startIndexes)

            with arcpy.da.InsertCursor("PointsAuto", ["SHAPE@"]) as cursor:
                for point in array:
                    cursor.insertRow([point])

            line = arcpy.Polyline(array, spatial_reference = spatial_reference)

            with arcpy.da.InsertCursor("LineAuto", ["SHAPE@"]) as cursor:
                cursor.insertRow([line])

            current_map.addDataFromPath(lineFeatureClass)
            current_map.addDataFromPath(pointFeatureClass)

        return


    def postExecute(self, parameters):
        """This method takes place after outputs are processed and
        added to the display."""
        return

    def importFromFile(self, file) -> arcpy.Array:
        array = arcpy.Array()
        suffixes = pathlib.Path(file).suffixes
        match suffixes[len(suffixes)-1]:
            case '.csv':
                try:
                    for line in open(file, 'r'):
                        splitLine = line.split(',')
                        if len(splitLine) in (2, 3):
                            points = [float(i) for i in splitLine]
                            array.add(arcpy.Point(*points))
                except:
                    arcpy.AddMessage("Invalid file or values")

            case '.xml':
                try:
                    tree = xml.parse(file)
                    root = tree.getroot()
                    for point in root.findall('Point'):
                        x, y= float(point.find('x').text), float(point.find('y').text)
                        z = point.find('z')
                        if z is not None:
                            array.add(arcpy.Point(x, y, float(z.text)))
                        else:
                            array.add(arcpy.Point(x, y))
                except:
                    arcpy.AddMessage("Invalid file or values")


            case '.json':
                try:
                    with open(file, 'r') as file:
                        data = json.load(file)
                    for line in data:
                        array.add(arcpy.Point(line['x'], line['y'], line['z'])) if 'z' in line else \
                        array.add(arcpy.Point(line['x'], line['y']))
                except:
                    arcpy.AddMessage("Invalid file or values")

            case _:
                arcpy.AddMessage("Invalid file format")

        return array
    
    def getRadarValue(self, x, y):
        return float(arcpy.GetCellValue_management(self.radar,f'{x} {y}',None).getOutput(0))
    
    def createBoolArray(self, start, end):
        array = arcpy.RasterToNumPyArray(self.radar, nodata_to_value=-200)
        startIndex = self.snapPointToRasterCenter(start)
        endIndex = self.snapPointToRasterCenter(end)
        return array>-110, startIndex, endIndex
    
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

def theta_star(grid, start, end):
    rows, cols = grid.shape
    open_set = []
    heapq.heappush(open_set, (heuristic(start,end), start))  # (f, node)
    came_from = {start: start}
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
            while current != came_from[current]:
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
                if neighbor not in open_set:
                    g_score[neighbor] = float('inf')
                    came_from[neighbor] = None
                update_vertex(current,neighbor,g_score,came_from,open_set,end,grid, f_score)

    return None  # No path found

def update_vertex(current, neighbor, g_score, parent, open_set, finish, grid, f_score):
    if line_of_sight(parent[current], neighbor, grid):
        new_g = g_score[parent[current]] + 1
        if new_g < g_score[neighbor]:
            g_score[neighbor] = new_g
            f_score[neighbor] = new_g + heuristic(neighbor,finish)
            parent[neighbor] = parent[current]
            if neighbor in open_set:
                open_set.remove(neighbor)
            heapq.heappush(open_set, (f_score[neighbor], neighbor))
    else:
        new_g = g_score[current] + 1
        if new_g < g_score[neighbor]:
            g_score[neighbor] = new_g
            f_score[neighbor] = new_g + heuristic(neighbor,finish)
            parent[neighbor] = current
            if neighbor in open_set:
                open_set.remove(neighbor)
            heapq.heappush(open_set, (f_score[neighbor], neighbor))

def line_of_sight(point1, point2, grid):

    x0, y0 = point1
    x1, y1 = point2

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    sX = 1 if x1 > x0 else -1
    sY = 1 if y1 > y0 else -1

    if dx > dy:
        err = dx / 2
        while x0 != x1:
            if grid[(x0, y0)]:
                return False
            err -= dy
            if err < 0:
                y0 += sY
                err += dx
            x0 += sX
    else:
        err = dy / 2
        while y0 != y1:
            if grid[(x0, y0)]:
                return False
            err -= dx
            if err < 0:
                x0 += sX
                err += dy
            y0 += sY
    return True

def heuristic(a, b):
# Manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

