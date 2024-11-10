import arcpy

#test
"""
Script documentation

- Tool parameters are accessed using arcpy.GetParameter() or 
                                     arcpy.GetParameterAsText()
- Update derived parameter values using arcpy.SetParameter() or
                                        arcpy.SetParameterAsText()
"""



def script_tool(startX, startY, endX, endY):
    """Script code goes below"""

    currenProject = arcpy.mp.ArcGISProject('CURRENT')
    current_map = currenProject.activeMap
    outworkspace = arcpy.env.workspace
    lineFeatureClass = arcpy.CreateFeatureclass_management(outworkspace,"Line","POLYLINE", spatial_reference = arcpy.SpatialReference(3857))
    pointFeatureClass = arcpy.CreateFeatureclass_management(outworkspace,"Points","POINT", spatial_reference = arcpy.SpatialReference(3857))
    

    arcpy.AddField_management("Line","LineID","SHORT")
    arcpy.AddField_management("Line","Name","TEXT","","",16)

    arcpy.AddField_management("Points","PointID","SHORT")
    arcpy.AddField_management("Points","Name","TEXT","","",16)

    array = arcpy.Array()
    point=arcpy.Point(startX,startY)
    array.add(point)

    point.X = endX
    point.Y = endY
    array.add(point)

    Line = arcpy.Polyline(array, spatial_reference = arcpy.SpatialReference(3857))

    edit_lines = arcpy.da.InsertCursor("Line",["Shape@",'LineID','Name'])

    new_row = [Line,1,"FirstLine"]

    edit_lines.insertRow(new_row)


    edit_lines = arcpy.da.InsertCursor("Points",["Shape@",'PointID','Name'])
    p_index=0
    
    for p in array:
        point_obj=[p,p_index,"Point"]
        edit_lines.insertRow(point_obj)
        p_index=p_index+1

    del edit_lines    

    current_map.addDataFromPath(lineFeatureClass)
    current_map.addDataFromPath(pointFeatureClass)



    return


if __name__ == "__main__":

    param0 = arcpy.GetParameter(0)
    param1 = arcpy.GetParameter(1)
    param2 = arcpy.GetParameter(2)
    param3 = arcpy.GetParameter(3)

    script_tool(param0, param1, param2, param3)
    #arcpy.SetParameterAsText(2, "Result")"