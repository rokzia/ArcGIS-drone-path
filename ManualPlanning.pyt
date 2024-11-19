# -*- coding: utf-8 -*-

import arcpy
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
            ),
            arcpy.Parameter(
                displayName='Minimum altitude',
                name='minimumAltitude',
                datatype='Double',
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
        self.raster = arcpy.Raster(parameters[1].valueAsText)
        waypoint_file = parameters[2].value 
        self.minimumAltitude = parameters[3].value

        array = arcpy.Array()

        current_map = arcpy.mp.ArcGISProject('CURRENT').activeMap
        spatial_reference = current_map.spatialReference
        lineFeatureClass = arcpy.CreateFeatureclass_management(arcpy.env.workspace,"Line","POLYLINE", spatial_reference = arcpy.SpatialReference(3346))

    
        if waypoint_file:
            self.importFromFile(waypoint_file)
        elif waypoints:
            with arcpy.da.SearchCursor(waypoints, ["SHAPE@"]) as cursor:
                for row in cursor:
                    point = arcpy.Point(row[0].centroid.X, row[0].centroid.Y)
                    point.Z = float(arcpy.GetCellValue_management(self.raster,f'{point.X} {point.Y}',None).getOutput(0)) + self.minimumAltitude
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
                    if pointZ > float(arcpy.GetCellValue_management(self.raster,f'{pointX} {pointY}',None).getOutput(0)) + self.minimumAltitude:
                        array.add(arcpy.Point(pointX, pointY, pointZ))
                    else:
                        arcpy.AddMessage("Point is too low")

            case '.xml':
                tree = xml.parse(file)
                root = tree.getroot()
                for point in root.findall('Point'):
                    x, y, z = float(point.find('x').text), float(point.find('y').text), float(point.find('z').text)
                    if z > float(arcpy.GetCellValue_management(self.raster,f'{x} {y}',None).getOutput(0)) + self.minimumAltitude:
                        array.add(arcpy.Point(x, y, z))
                    else:
                        arcpy.AddMessage("Point is too low")


            case '.json':
                with open(file, 'r') as file:
                    data = json.load(file)
                for line in data:
                    if line['z'] > float(arcpy.GetCellValue_management(self.raster,f'{line['x']} {line['y']}',None).getOutput(0)) + self.minimumAltitude:
                        array.add(arcpy.Point(line['x'], line['y'], line['z']))
                    else:
                        arcpy.AddMessage("Point is too low")
        
