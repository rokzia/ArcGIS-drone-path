# -*- coding: utf-8 -*-

import arcpy
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

        current_map = arcpy.mp.ArcGISProject('CURRENT').activeMap
        lineFeatureClass = arcpy.CreateFeatureclass_management(arcpy.env.workspace,"Line","POLYLINE", spatial_reference = arcpy.SpatialReference(3346))

        array = arcpy.Array()
        with arcpy.da.SearchCursor(waypoints, ["SHAPE@"]) as cursor:
            for row in cursor:
                array.add(arcpy.Point(row[0].centroid.X, row[0].centroid.Y))

        Line = arcpy.Polyline(array, spatial_reference = arcpy.SpatialReference(3346))

        with arcpy.da.InsertCursor("Line", ["SHAPE@"]) as cursor:
            cursor.insertRow([Line])

        current_map.addDataFromPath(lineFeatureClass)

        return


    def postExecute(self, parameters):
        """This method takes place after outputs are processed and
        added to the display."""
        return
