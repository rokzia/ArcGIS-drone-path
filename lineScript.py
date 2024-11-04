import arcpy


"""
Script documentation

- Tool parameters are accessed using arcpy.GetParameter() or 
                                     arcpy.GetParameterAsText()
- Update derived parameter values using arcpy.SetParameter() or
                                        arcpy.SetParameterAsText()
"""



def script_tool(startX, startY, endX, endY):
    """Script code goes below"""
    outworkspace = arcpy.env.workspace
    arcpy.CreateFeatureclass_management(outworkspace,"TestLines","POLYLINE", spatial_reference = arcpy.SpatialReference(3857))
    arcpy.CreateFeatureclass_management(outworkspace,"Points","POINT", spatial_reference = arcpy.SpatialReference(3857))
    

    arcpy.AddField_management("TestLines","LineID","SHORT")
    arcpy.AddField_management("TestLines","Name","TEXT","","",16)

    arcpy.AddField_management("Points","PointID","SHORT")
    arcpy.AddField_management("Points","Name","TEXT","","",16)

    array = arcpy.Array()
    point=arcpy.Point(startX,startY)
    array.add(point)

    point.X = endX
    point.Y = endY
    array.add(point)

    Line = arcpy.Polyline(array, spatial_reference = arcpy.SpatialReference(3857))

    edit_lines = arcpy.da.InsertCursor("TestLines",["Shape@",'LineID','Name'])

    new_row = [Line,1,"FirstLine"]

    edit_lines.insertRow(new_row)


    edit_lines = arcpy.da.InsertCursor("Points",["Shape@",'PointID','Name'])
    p_index=0

    for p in array:
        point_obj=[p,p_index,"Point"]
        edit_lines.insertRow(point_obj)
        p_index=p_index+1

    del edit_lines    

    in_layer = arcpy.management.SelectLayerByAttribute("TestLines")
    arcpy.conversion.FeaturesToGraphics(in_layer, "LineGraphicalLayer", "EXCLUDE_FEATURES")
    in_layer = arcpy.management.SelectLayerByAttribute("Points")
    arcpy.conversion.FeaturesToGraphics(in_layer, "PointLayer", "EXCLUDE_FEATURES")
    return


if __name__ == "__main__":

    param0 = arcpy.GetParameter(0)
    param1 = arcpy.GetParameter(1)
    param2 = arcpy.GetParameter(2)
    param3 = arcpy.GetParameter(3)

    script_tool(param0, param1, param2, param3)
    #arcpy.SetParameterAsText(2, "Result")"