# -*- coding: utf-8 -*-

import arcpy
import pathlib
import json
import xml.etree.ElementTree as xml
import numpy as np

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
        #arcpy.AddField_management('Points', 'Probability of detection', 'FLOAT') neveikia kazkodel

        if waypoint_file:
            array = self.importFromFile(waypoint_file)
        elif waypoints:
            with arcpy.da.SearchCursor(waypoints, ["SHAPE@"]) as cursor:
                for row in cursor:
                    point = arcpy.Point(row[0].centroid.X, row[0].centroid.Y)
                    point.Z = self.getHeightValue(point.X, point.Y) + self.minimumAltitude
                    array.add(point)

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
        
    def signalToNoiseRatio(power, amplification, waveLength, rcs, distance):
        return (power * amplification**2 * waveLength**2 * rcs) / ((4 * np.pi)**3 * distance**4)
    
    def probabilityOfDetection(SignalToNoise, step=10):
        return 1 - np.exp(-SignalToNoise / step)
    
    def getHeightValue(self, x, y):
        return float(arcpy.GetCellValue_management(self.height,f'{x} {y}',None).getOutput(0))
    
    def getRadarValue(self, x, y):
        return float(arcpy.GetCellValue_management(self.radar,f'{x} {y}',None).getOutput(0))
