#Simulated annealing for location optimization of recharging stations for drone delivery system 
#MCLP
#
# 1. using pre-made dictionary from ArcGIS to use attribute tables: read order always follows FID, so it is maintained.  
# 2. spatial relationship is assessed by shapley 
#   - ESP distance should be utilzed to measure distance between locations
#   - ArcGIS is way too slow
# 3. Spatially_restricted interchange heuristic
#   - 
# 4. Brown field: part of solution is fixed (warehouses)
# 5. Point demand representation 

import pysal,  shapefile, networkx, time, cPickle, random
from shapely.geometry import Point, Polygon, LineString, MultiPoint, MultiPolygon
from collections import defaultdict

path = ""
ffDict = ""
fdDict = ""
dDict = ""
facilities_f = ""
demands_f = ""
#loading matrices & initialize variables 
f_FF = open(path + ffDict)
f_FD = open(path + fdDict)
f_demand = open(path + dDict)
F_Fdict = cPickle.load(f_FF)
F_Ddict = cPickle.load(f_FD)

dDict = cPickle.load(f_demand)
facil_shp = generateGeometry(facilities_f)
demand_shp = generateGeometry(demands_f)
warehouses_ID = []    #id_f of warehouses
solution_sites = []
covered_demand = []
objective_value = 0
p = 0   #actual p - no. warehouses 
t0 = 0   #end temperature
max_iter = 0   #iteration limit
tf = 0         
cf = 0.2
remove_percent = 0.2 
fd_fullPayload = 5 
fd_empty = 10
fd_delivery = 3.33 

def generateGeometry(in_shp):
    resultingGeometry = []
    if in_shp.header['Shape Type'] == 1:
        for i in range(len(in_shp)):
            resultingGeometry.append(Point(in_shp.get_shape(i)['X'], in_shp.get_shape(i)['Y']))
    elif in_shp.header['Shape Type'] == 3:
        for i in range(len(in_shp)):
            resultingGeometry.append(LineString(in_shp.get_shape(i)['Vertices']))
    elif in_shp.header['Shape Type'] == 5:
        for i in range(len(in_shp)):
            resultingGeometry.append(Polygon(in_shp.get_shape(i)['Vertices']))
    return resultingGeometry    

def cal_obj(in_solution):
    covered = []
    for site in in_solution:
        covered.extend(F_Ddict[site])
    covered = list(set(covered))
    for de in covered:
        obj += dDict[de]
    return obj

def chk_isolation(in_sol):
    result = []
    for i in in_sol:
        if len(result) == 0:
            result.append(i.buffer(fd_fullPayload))
        else:
            result[0] = result[0].union(i.buffer(fd_fullPayload))
    if result[0].type == "MultiPolygon":
        return True
    else:
        return False

    

def resctricted_cadidates(in_solution):   #for spatial_interchange: 
    candis = []
    for site in in_solution:
        candis.extend(F_Fdict[site])
    candis = list(set(candis))
    candis = [x for x in candis if x not in in_solution]
    return candis

def spatial_interchange(in_solution):
    c = resctricted_cadidates(in_solution)
    flag = True
    while flag == True:
        flag = False
        while len(c) != 0:
            candi = random.choice(c)
            c.remove(candi)
            current_obj = [in_solution, cal_obj(in_solution)]
            for site in in_solution:
                temp_sol = in_solution
                temp_sol.remove(site)
                temp_sol.append(candi)
                temp_obj = cal_obj(temp_sol)
                if temp_obj > current_obj[1]:
                    if chk_isolation(temp_sol) == False: #prevent island in solution 
                        flag = True
                        current_obj = [temp_sol, temp_obj]
            if flag == True:
                in_solution = current_obj[0]
                
                
            

#initializing seed solution (random) 
solution_sites.extend(warehouses)
isol_chk = chk_isolation(solution_sites)
while isol_chk == True:
    while len(solution_sites) < p:
        random_pool = resctricted_cadidates(solution_sites)
        solution_sites.append(random.choice(random_pool))
    isol_chk = chk_isolation(solution_sites)




while t0 > 0.5:
    remove_number = int(remove_percent * p)
    for i in range(remove_number):
        facils = [x for x in solution_sites if not x in warehouses_ID]
        solution_sites.remove(random.choice(facils))
    
        
            
    
    
        
        
        
        
        
