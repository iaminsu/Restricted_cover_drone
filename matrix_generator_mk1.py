#matrix generator 

import  arcpy, cPickle
from arcpy import env

arcpy.env.overwriteOutput = True
fd_delivery = 3.33
# Variables 

path = "F:\\Dropbox\\research\\Distance restricted covering model\\Locating recharging station\\data4\\"
demand = "sample_demand_2_p.shp"
facil = "sample_sites_2.shp"
demand_dict = {}
facil_dict = {}
id_demand = "FID"

demand_f = "TOTALPOP"


#create feature layer 
demand_lyr = 'demand_lyr'

arcpy.MakeFeatureLayer_management(path + demand, demand_lyr)
FcS = arcpy.SearchCursor(demand_lyr)
fc = FcS.next()
while fc:
    demand_dict[fc.getValue(id_demand)] = str(fc.getValue(demand_f))
    fc = FcS.next()

##facil_lyr = "f_lyr"
##arcpy.MakeFeatureLayer_management(path + facil, facil_lyr)
##FcS = arcpy.SearchCursor(facil_lyr)
##fc = FcS.next()
##while fc:
##    fid = fc.getValue(id_demand)
##    arcpy.SelectLayerByAttribute_management(facil_lyr, "NEW_SELECTION", id_demand + "= " + str(fid))
##    arcpy.SelectLayerByLocation_management(demand_lyr, "WITHIN_A_DISTANCE", facil_lyr, str(fd_delivery) + " Miles", "NEW_SELECTION")
##    aa = arcpy.Describe(demand_lyr)
##    ss = aa.FIDset
##    tt = ss.split("; ")
##    tt = [int(x) for x in tt]
##    facil_dict[fid] = tt    
    


f2 = open(path + "demands.txt", 'w')
f3 = open(path + "facil_initial_demand.txt", 'w')
cPickle.dump(demand_dict, f2)
cPickle.dump(facil_dict, f3)
f2.close()
f3.close()

