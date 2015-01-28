#matrix generator 

import  arcpy, cPickle
from arcpy import env

arcpy.env.overwriteOutput = True

# Variables 

path = "F:\\data\\rechargeable path\\"
demand = "sample_demand.shp"

demand_dict = {}

id_demand = "ID_d"

demand_f = "demand"


#create feature layer 
demand_lyr = 'demand_lyr'

arcpy.MakeFeatureLayer_management(path + demand, demand_lyr)
FcS = arcpy.SearchCursor(demand_lyr)
fc = FcS.next()
while fc:
    demand_dict[fc.getValue(id_demand)] = fc.getValue(demand_f)
    fc = FcS.next()



f2 = open(path + "demands.txt", 'w')

cPickle.dump(demand_dict, f2)

f2.close()


