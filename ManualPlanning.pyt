# -*- coding: utf-8 -*-

import arcpy
import numpy as np
import pathlib
import json
import xml.etree.ElementTree as xml

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
                parameterType='Optional',
                direction='Input'
            ),
            arcpy.Parameter(
                displayName='Waypoint file',
                name='waypoint_file',
                datatype='File',
                parameterType='Optional',
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
        heightRaster = parameters[1].value
        waypoint_file = parameters[2].value #Kazkodel value yra geoprocessing objektas o ne tsg string

        array = arcpy.Array()

        if(waypoint_file):
            file_path = str(waypoint_file)
            suffixes = pathlib.Path(file_path).suffixes
            match suffixes[len(suffixes)-1]:
                case '.csv':
                    arcpy.AddMessage("csv failas")
                    for line in open(file_path, 'r'):
                        pointX, pointY = line.split(';')
                        array.add(arcpy.Point(pointX, pointY))

                case '.xml':
                    tree = xml.parse(file_path)
                    root = tree.getroot()
                    for point in root.findall('Point'):
                        array.add(arcpy.Point(int(point.find('x').text), int(point.find('y').text)))


                case '.json':
                    with open(file_path, 'r') as file:
                        data = json.load(file)
                    for line in data:
                        array.add(arcpy.Point(line['x'], line['y']))

                
                    






        current_map = arcpy.mp.ArcGISProject('CURRENT').activeMap
        spatial_reference = current_map.spatialReference
        lineFeatureClass = arcpy.CreateFeatureclass_management(arcpy.env.workspace,"Line","POLYLINE", spatial_reference = spatial_reference)

        if(waypoints):
            with arcpy.da.SearchCursor(waypoints, ["SHAPE@"]) as cursor:
                for row in cursor:
                    array.add(arcpy.Point(row[0].centroid.X, row[0].centroid.Y))

        Line = arcpy.Polyline(array, spatial_reference = spatial_reference)

        with arcpy.da.InsertCursor("Line", ["SHAPE@"]) as cursor:
            cursor.insertRow([Line])

        current_map.addDataFromPath(lineFeatureClass)

        return


    def postExecute(self, parameters):
        """This method takes place after outputs are processed and
        added to the display."""
        return
