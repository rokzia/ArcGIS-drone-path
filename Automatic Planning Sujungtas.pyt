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
        self.label = "Automatic route planning"
        self.alias = "toolbox"

        # List of tool classes associated with this toolbox
        self.tools = [Tool]


class Tool:
    def __init__(self):
        """Define the tool (tool name is the name of the class)."""
        self.label = "Automatic route script"
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
                displayName='Height raster',
                name='height_raster',
                datatype='Raster Layer',
                parameterType='Required',
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
                displayName='Minimum altitude',
                name='minimumAltitude',
                datatype='Double',
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
        self.height = arcpy.Raster(parameters[1].valueAsText)
        self.radar = arcpy.Raster(parameters[2].valueAsText)
        waypoint_file = parameters[3].value 
        self.minimumAltitude = parameters[4].value

        array = arcpy.Array()

        current_map = arcpy.mp.ArcGISProject('CURRENT').activeMap
        spatial_reference = arcpy.SpatialReference(3346)
        lineFeatureClass = arcpy.CreateFeatureclass_management(arcpy.env.workspace,"Line","POLYLINE", spatial_reference = spatial_reference)
        pointFeatureClass = arcpy.CreateFeatureclass_management(arcpy.env.workspace,"Points","POINT", spatial_reference = spatial_reference)

        arcpy.AddField_management("Points","X","FLOAT")
        arcpy.AddField_management("Points","Y","FLOAT")
        #arcpy.AddField_management('Points', 'Probability of detection', 'FLOAT') neveikia kazkodel

        if waypoint_file:
            array = self.importFromFile(waypoint_file)
        elif waypoints:
            with arcpy.da.SearchCursor(waypoints, ["SHAPE@"]) as cursor:
                for row in cursor:
                    point = arcpy.Point(row[0].centroid.X, row[0].centroid.Y)
                    height = self.getHeightValue(point.X, point.Y)
                    if height != 'NoData':
                        height = float(height)
                    point.Z = height + self.minimumAltitude if self.getHeightValue(point.X, point.Y) != 'NoData' else 0
                    self.getHeightValue(point.X, point.Y) if not 'NoData' else 0
                    array.add(point)
            arcpy.AddMessage("Pries a star")
            path = self.a_star([array[0].X, array[0].Y],[array[1].X, array[1].Y])
            array.removeAll()
            for point in path:
                # arcpy.AddMessage(point)
                geometryPoint = arcpy.Point(point.X, point.Y)
                array.add(geometryPoint)



        with arcpy.da.InsertCursor("Points", ["SHAPE@"]) as cursor:
            for p in array:
                cursor.insertRow([p])

        current_map.addDataFromPath(pointFeatureClass)


        Line = arcpy.Polyline(array, spatial_reference = spatial_reference)

        with arcpy.da.InsertCursor("Line", ["SHAPE@"]) as cursor:
            cursor.insertRow([Line])

        current_map.addDataFromPath(lineFeatureClass)

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
                arcpy.AddMessage("csv failas")
                for line in open(file, 'r'):
                    pointX, pointY, pointZ = line.split(';')
                    if pointZ > self.getHeightValue(pointX, pointY) + self.minimumAltitude:
                        array.add(arcpy.Point(pointX, pointY, pointZ))
                    else:
                        arcpy.AddMessage("Point is too low")

            case '.xml':
                tree = xml.parse(file)
                root = tree.getroot()
                for point in root.findall('Point'):
                    x, y, z = float(point.find('x').text), float(point.find('y').text), float(point.find('z').text)
                    if z > self.getHeightValue(float(point.find('x').text), float(point.find('y').text)) + self.minimumAltitude:
                        array.add(arcpy.Point(x, y, z))
                    else:
                        arcpy.AddMessage("Point is too low")


            case '.json':
                with open(file, 'r') as file:
                    data = json.load(file)
                for line in data:
                    if line['z'] > self.getHeightValue(line['x'], line['y']) + self.minimumAltitude:
                        array.add(arcpy.Point(line['x'], line['y'], line['z']))
                    else:
                        arcpy.AddMessage("Point is too low")

        return array

    class Node:
        def __init__(self,x,y,g,h, parent):
            self.X=x
            self.Y=y
            self.g=g
            self.h=h
            self.f=None
            self.parent = parent
        def setF(self):
            self.f = self.g + self.h
        def __eq__(self, value):
            if value!=None:
                return self.X == value.X and self.Y == value.Y

        def __hash__(self):
            return hash((self.X,self.Y))
        def __lt__(self, other):
            return self.f<other.f
        def __gt__(self, other):
            return self.f>other.f

        

    def getNeighbors(self,point,start,end):
        #TODO pakeisti rezius i rastro parametrus, jei imanoma
        neighbors = []
        for i in range(-10,10,5):
            for j in range(-10,10,5):
                if i == 0 and j == 0:
                    continue
                radar_check = self.getRadarValue(point.X+j,point.Y+i) == "NoData"
                if radar_check:
                    neighbors.append(self.Node(point.X+j,point.Y+i,self.distanceFromPoints([start.X,start.Y], [point.X+j,point.Y+i]),
                    self.distanceFromPoints([end.X,end.Y], [point.X+j,point.Y+i]),point))
        return neighbors

    def a_star(self, start, end):
        
        start = self.Node(start[0], start[1],0,self.distanceFromPoints([start[0],start[1]], [end[0], end[1]]),None)
        end = self.Node(end[0], end[1],0,0,None)
        open_set = []
        heapq.heappush(open_set, start)  # (f, node)
        visited = set()

        while open_set:
            current = heapq.heappop(open_set)

            if current in visited:
                continue
            visited.add(current)

            if current == end or current.h<10:
                # Reconstruct path
                path = []
                path.append(end)
                while current.parent != None:
                    path.append(current)
                    current = current.parent
                return path[::-1]

            # Explore neighbors
            neighbors = self.getNeighbors(current, start,end)
            for neighbor in neighbors:
                if (
                    neighbor not in visited
                ):
                    tentative_g_score = current.g 
                    if tentative_g_score < neighbor.g:
                        neighbor.parent = current
                        neighbor.g = tentative_g_score
                        neighbor.f = tentative_g_score + self.distanceFromPoints([neighbor.X, neighbor.Y], [end.X, end.Y])
                        heapq.heappush(open_set, neighbor)

        return None

                



    def signalToNoiseRatio(power, amplification, waveLength, rcs, distance):
        return (power * amplification**2 * waveLength**2 * rcs) / ((4 * np.pi)**3 * distance**4)
    
    def probabilityOfDetection(SignalToNoise, step=10):
        return 1 - np.exp(-SignalToNoise / step)
    
    def getHeightValue(self, x, y):
        return arcpy.GetCellValue_management(self.height,f'{x} {y}',None).getOutput(0)
    
    def getRadarValue(self, x, y):
        return arcpy.GetCellValue_management(self.radar,f'{x} {y}',None).getOutput(0)
    
    def distanceFromPoints(self,point1,point2):
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    



