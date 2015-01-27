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

import pysal,  shapefile, networkx, time, cPickle, random, math
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
temperature = 0   #end temperature
max_iter = 5   #iteration limit
terminate_temp = 0         
temp_ratio = 0.2
sa_count = 0
remove_percent = 0.2 
fd_fullPayload = 5 
fd_empty = 10
fd_delivery = 3.33 
rc = 0.1

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

    

def restricted_cadidates(in_solution):   #for spatial_interchange: 
    candis = []
    for site in in_solution:
        candis.extend(F_Fdict[site])
    candis = list(set(candis))
    candis = [x for x in candis if x not in in_solution]
    return candis

def spatial_interchange(in_solution):
    c = restricted_cadidates(in_solution)
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
    return in_solution
                

def random_fill(in_solution=[]):
    isolation = True
    
    while isolation == True:
        new_solution = in_solution
        while len(in_solution) < p:
            random_pool = restricted_cadidates(new_solution)
            new_solution.append(random.choice(random_pool))
        new_isolation = chk_isolation(new_solution)
        if new_isolation == False:
            in_solution = new_solution
            isolation = False
    return in_solution
            

#initializing seed solution (random) 

isol_chk = True
while isol_chk == True:
    solution_sites = warehouses_ID
    while len(solution_sites) < p:
        random_pool = restricted_cadidates(solution_sites)
        solution_sites.append(random.choice(random_pool))
    isol_chk = chk_isolation(solution_sites)


while temperature > 0.5:
    current_solution = solution_sites
    current_obj = cal_obj(current_solution)
    new_solution = current_solution
    remove_number = int(remove_percent * p)
    for i in range(remove_number):
        facils = [x for x in new_solution if not x in warehouses_ID]
        new_solution.remove(random.choice(facils))
    new_solution = random_fill(new_solution)
    new_obj = cal_obj(new_solution)
    
    if new_obj > current_obj:
        solution_sites = new_solution
        sa_count = 0 
    else:
        if sa_count > max_iter:
            sa_count = 0
            temperature = temperature - (temperature * temp_ratio)
            if temperature < terminate_temp:
                break
        else:
            sa_count += 1
            if random.random() < math.exp(((current_obj-new_obj))*rc/temperature):
                solution_sites = new_solution
                
                
        
        
            
    
    
        
        
        
        
        
