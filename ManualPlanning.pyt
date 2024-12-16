# -*- coding: utf-8 -*-

import arcpy
import pathlib
import json
import xml.etree.ElementTree as xml
import numpy as np
import heapq
from math import sqrt, ceil

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
                displayName='Drone speed (m/s)',
                name='droneSpeed',
                datatype='GPDouble',
                parameterType='Required',
                direction='Input'
            ),
            arcpy.Parameter(
                displayName='Draw auto route',
                name='drawAutoRoute',
                datatype='GPBoolean',
                parameterType='Optional',
                direction='Input'
            ),
            #Manual route output
            arcpy.Parameter(
                displayName='Manual route time in signal (s)',
                name='timeInSignal',
                datatype='GPString',
                parameterType='Derived',
                direction='Output'
            ),
            arcpy.Parameter(
                displayName='Manual route autoDistance in signal (m)',
                name='distanceInSignal',
                datatype='GPString',
                parameterType='Derived',
                direction='Output'
            ),
            arcpy.Parameter(
                displayName='Manual route signal crossings',
                name='signalCrossings',
                datatype='GPString',
                parameterType='Derived',
                direction='Output'
            ),
            arcpy.Parameter(
                displayName='Manual route average signal strength',
                name='signalStrength',
                datatype='GPString',
                parameterType='Derived',
                direction='Output'
            ),
            #Automatic route output
            arcpy.Parameter(
                displayName='Automatic route time in signal (s)',
                name='autoTimeInSignal',
                datatype='GPString',
                parameterType='Derived',
                direction='Output'
            ),
            arcpy.Parameter(
                displayName='Automatic route autoDistance in signal (m)',
                name='autoDistanceInSignal',
                datatype='GPString',
                parameterType='Derived',
                direction='Output'
            ),
            arcpy.Parameter(
                displayName='Automatic route signal crossings',
                name='autoSignalCrossings',
                datatype='GPString',
                parameterType='Derived',
                direction='Output'
            ),
            arcpy.Parameter(
                displayName='Automatic route average signal strength',
                name='autoSignalStrength',
                datatype='GPString',
                parameterType='Derived',
                direction='Output'
            ),
            #Comparison
            arcpy.Parameter(
                displayName='Route time in signal comparison',
                name='routeComparisonTime',
                datatype='GPString',
                parameterType='Derived',
                direction='Output'
            ),
            arcpy.Parameter(
                displayName='Route average signal strength comparison',
                name='routeComparisonAverage',
                datatype='GPString',
                parameterType='Derived',
                direction='Output'
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
        waypoint_file = parameters[2].valueAsText 
        droneSpeed = parameters[3].value
        drawAutoRoute = parameters[4].value

        self.resolution = self.radar.meanCellWidth
        self.numpyRaster = arcpy.RasterToNumPyArray(self.radar, nodata_to_value=-111)

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

        manualDistance, manualTime, manualCrossings, manualAverage = self.calculateRouteMetrics(array, self.radar, droneSpeed, self.resolution)

        parameters[5].value = manualTime
        parameters[6].value = manualDistance
        parameters[7].value = manualCrossings
        parameters[8].value = manualAverage

        #Manual planning end

        if drawAutoRoute:
            #Insert auto planning route as well
            lineFeatureClass = arcpy.CreateFeatureclass_management(arcpy.env.workspace,"LineAuto","POLYLINE", spatial_reference = spatial_reference)
            pointFeatureClass = arcpy.CreateFeatureclass_management(arcpy.env.workspace,"PointsAuto","POINT", spatial_reference = spatial_reference)

            startPoint = array[0]
            endPoint = array[-1]

            weight = 3000

            startIndexes = self.snapPointToRasterCenter(startPoint)
            endIndexes = self.snapPointToRasterCenter(endPoint)

            path = a_star(self.numpyRaster, startIndexes, endIndexes, weight, self.resolution, (self.radar.extent.XMin, self.radar.extent.YMax))

            if path is None:
                arcpy.AddMessage("No path found")
                return

            array = self.createPointArray(startPoint, path, startIndexes)

            with arcpy.da.InsertCursor("PointsAuto", ["SHAPE@"]) as cursor:
                for point in array:
                    cursor.insertRow([point])

            line = arcpy.Polyline(array, spatial_reference = spatial_reference)

            with arcpy.da.InsertCursor("LineAuto", ["SHAPE@"]) as cursor:
                cursor.insertRow([line])

            current_map.addDataFromPath(lineFeatureClass)
            current_map.addDataFromPath(pointFeatureClass)

            autoDistance, autoTime, autoCrossings, autoAverage = self.calculateRouteMetrics(array, self.radar, droneSpeed, self.resolution)

            parameters[9].value = autoTime
            parameters[10].value = autoDistance
            parameters[11].value = autoCrossings
            parameters[12].value = autoAverage

            if autoTime < manualTime:
                parameters[13].value = "Automatic route spends less time in signal"
            elif autoTime > manualTime:
                parameters[13].value = "Manual route spends less time in signal"
            else:
                parameters[13].value = "Both routes spend the same time in signal"

            if manualAverage < autoAverage:
                parameters[14].value = "Manual route has better average signal strength"
            elif manualAverage > autoAverage:
                parameters[14].value = "Automatic route has better average signal strength"
            else:
                parameters[14].value = "Both routes have the same average signal strength"
        
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
                array = self.importCsv(file)

            case '.xml':
                array = self.importXml(file)

            case '.json':
                array = self.importJson(file)

            case _:
                arcpy.AddMessage("Invalid file format")

        return array
    
    def importCsv(self, file):
        array = arcpy.Array()
        try:
            for line in open(file, 'r'):
                splitLine = line.split(',')
                if len(splitLine) in (2, 3):
                    points = [float(i) for i in splitLine]
                    array.add(arcpy.Point(*points))
        except:
            arcpy.AddMessage("Invalid file or values")

        return array
    
    def importXml(self, file):
        array = arcpy.Array()
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

        return array
    
    def importJson(self, file):
        array = arcpy.Array()
        try:
            with open(file, 'r') as file:
                data = json.load(file)
            for line in data:
                array.add(arcpy.Point(float(line['x']), float(line['y']), float(line['z']))) if 'z' in line else \
                array.add(arcpy.Point(float(line['x']), float(line['y'])))
        except:
            arcpy.AddMessage("Invalid file or values")

        return array
    
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
    
    #drone speed in meters per second
    def calculateRouteMetrics(self, pointArray: arcpy.Array, droneSpeed: float, samplingInterval: int):
        totalDistanceInSignal = 0
        totalTimeInSignal = 0
        signalCrossings = 0
        average = 0
        count = 0
        previousInSignal = None

        for i in range(len(pointArray) - 1):
            x1, y1 = pointArray[i].X, pointArray[i].Y
            x2, y2 = pointArray[i + 1].X, pointArray[i + 1].Y

            points = interpolatePoints((x1, y1), (x2, y2), samplingInterval)

            for x, y in points:
                rasterValue = self.numpyRaster[*self.snapPointToRasterCenter(arcpy.Point(x, y))]
                average += rasterValue
                count += 1

                inSignal = rasterValue > -110

                if previousInSignal is not None and previousInSignal != inSignal:
                    signalCrossings += 1

                if inSignal:
                    totalDistanceInSignal += samplingInterval

                previousInSignal = inSignal
        
        totalTimeInSignal = totalDistanceInSignal / droneSpeed
        average = average / count
        return totalDistanceInSignal, totalTimeInSignal, signalCrossings, average

def a_star(grid, start, end, weight, resolution, origin):
    rows, cols = grid.shape
    open_set = []
    heapq.heappush(open_set, (0, start))  # (f, node)
    came_from = {start: start}
    g_score = {start: 0}
    f_score = {start: euclidian(start,end, resolution, origin)}
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
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (1,1), (-1,-1), (-1,1), (1,-1)]
        ]

        for neighbor in neighbors:
            if (
                0 <= neighbor[0] < rows
                and 0 <= neighbor[1] < cols 
                and neighbor not in visited
            ):
                if neighbor not in open_set:
                    f_score[neighbor] = float('inf')
                    came_from[neighbor] = None
                update_vertex(current,neighbor,g_score,came_from,open_set,end,grid, f_score, weight, resolution, origin)

    return None  # No path found


def update_vertex(current, neighbor, g_score, parent, open_set, finish, grid, f_score, weight, resolution, origin):
    new_g = g_score[current] + 1
    new_f =  new_g + euclidian(finish,neighbor, resolution, origin) + weight*detectionCost(grid[neighbor])
    if new_f < f_score[neighbor]:
        g_score[neighbor] = new_g
        f_score[neighbor] = new_f 
        parent[neighbor] = current
        if neighbor in open_set:
            open_set.remove(neighbor)
            heapq.heapify(open_set)
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


def line_of_sight(point1, point2, grid):
    if(grid[point1]<grid[point2]):
        return False
    
    x0, y0 = point1
    x1, y1 = point2

    gridValue = grid[(x0,y0)]
   
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)

    #Steps in each direction
    sX = 1 if x1 > x0 else -1
    sY = 1 if y1 > y0 else -1

    err = dx + dy #if dx > dy else dy - dx

    while x0!=x1 or y0!=y1:
        if 2*err - dy> dx - 2 * err:
            err+=dy
            x0+=sX
        else:
            err+=dx
            y0+=sY
        if grid[(x0, y0)] > gridValue:
            return False
        if grid[(x0, y0)] < gridValue:
            gridValue = grid[(x0, y0)]

    return True

#manhattan distance
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def euclidian(a, b, resolution, origin):
    rasterOriginX, rasterOriginY = origin

    #True coordinates
    x0 = ((a[0] + 0.5) * resolution) + rasterOriginX
    x1 = ((b[0] + 0.5) * resolution) + rasterOriginX
    y0 = ((a[1] + 0.5) * resolution) + rasterOriginY
    y1 = ((b[1] + 0.5) * resolution) + rasterOriginY

    return ((x1-x0)**2 + (y1-y0)**2) ** 0.5

def detectionCost(fieldStrength):
    return (fieldStrength+111)/111

def interpolatePoints(point1, point2, interval: int):
    x1, y1 = point1
    x2, y2 = point2
    autoDistance = sqrt((x2 - x1)**2 + (y2 - y1)**2)
    numSamples = ceil(autoDistance / interval)
    points = [
        (
            x1 + (x2 - x1) * i / numSamples,
            y1 + (y2 - y1) * i / numSamples
        )
        for i in range(1, numSamples + 1)
    ]
    return points
