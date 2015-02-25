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

#Mk2:
# - add distance restriction 



import pysal,  shapefile, networkx, time, cPickle, random, math, copy
from shapely.geometry import Point, Polygon, LineString, MultiPoint, MultiPolygon
from collections import defaultdict

path = "/Users/insuhong/Dropbox/research/Distance restricted covering model/Locating recharging station/data4/"
ffDict = "FF_Dictsample_sites_2.shp_sample_demand_2_p.shp_obstacles_p.shp.txt"
fdDict = "FD_Dictsample_sites_2.shp_sample_demand_2_p.shp_obstacles_p.shp.txt"
demand_Dict = "demands.txt"
facilities_f = "sample_sites_2.shp"
demands_f = "sample_demand_2_p.shp"
#loading matrices & initialize variables 

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
    obj = 0 
    for site in in_solution:
        for i in F_Ddict[site]:
            covered.append(i[0])
        
    covered = list(set(covered))
    for de in covered:
        obj += float(dDict[de])
    return obj

def chk_isolation_old(in_sol):
    #check isolation: return false iff all sites are linked (overlapped) 
    #assume that the system is completed linked each other
    result = []
    for i in in_sol:
        if len(result) == 0:
            result.append(facil_shp[i].buffer(fd_fullPayload))
        else:
            result[0] = result[0].union(facil_shp[i].buffer(fd_fullPayload))
    if result[0].type == "MultiPolygon":
        print "Multi"
        return True
    else:
        return False
def chk_isolation(in_sol, wh_ids):   
    #return false if sites are linked to any of warehouses 
    #assume that the system allows separate delivery network from each warehouse
    
    result = []
    for i in in_sol:
        if len(result) == 0:
            result.append(facil_shp[i].buffer(fd_fullPayload))
        else:
            result[0] = result[0].union(facil_shp[i].buffer(fd_fullPayload))    
    if result[0].type == "MultiPolygon":
        indi = True
        for poly in result[0]:
            indi_poly = False
            for warehouse in wh_ids:
                if poly.intersects(facil_shp[warehouse]):
                    indi_poly = True
            if indi_poly == False:
                indi = False
                break
        if indi == True:
            return False
        else:
            return True
                    
    else:
        return False

def chk_feasibility(in_solution, save):
    feasibility = True
    covers = {}    
    for site in in_solution:
        if len(covers) == 0:
            covers[in_solution.index(site)] = facil_shp[site].buffer(fd_fullPayload)
        else:
            in_list = []
            for key in covers:
                if covers[key].intersects(facil_shp[site]):
                    in_list.append(key)
                    area = covers[key].union(facil_shp[site].buffer(fd_fullPayload))
                    covers[key] = area
            if len(in_list) == 0:
                covers[in_solution.index(site)] = facil_shp[site].buffer(fd_fullPayload)
            elif len(in_list) > 1: 
                
                chunk = covers[in_list[0]]
                for i in in_list:
                    chunk = chunk.union(covers[i])
                
                for i in in_list:
                    covers.pop(i)
                covers[in_solution.index(site)] = chunk
    for key in covers:
        indi = False
        for warehouse in warehouses_ID:
            if covers[key].intersects(facil_shp[warehouse]):
                indi = True
                break
        if indi == False:
            feasibility = False
    
    if save == True:
        w = shapefile.Writer(shapefile.POLYGON)
        w.field('net')
        for key in covers:
            w.poly(parts=[[list(x) for x in list(covers[key].exterior.coords)]])
            w.record('ff')
        w.save(path + "area")                       
    return feasibility

def chk_feasibility_all (in_solution, save):
    feasibility = True
    covers = {}    
    for site in in_solution:
        if len(covers) == 0:
            covers[in_solution.index(site)] = facil_shp[site].buffer(fd_fullPayload)
        else:
            in_list = []
            for key in covers:
                if covers[key].intersects(facil_shp[site]):
                    in_list.append(key)
                    area = covers[key].union(facil_shp[site].buffer(fd_fullPayload))
                    covers[key] = area
            if len(in_list) == 0:
                covers[in_solution.index(site)] = facil_shp[site].buffer(fd_fullPayload)
            elif len(in_list) > 1: 
                
                chunk = covers[in_list[0]]
                for i in in_list:
                    chunk = chunk.union(covers[i])
                
                for i in in_list:
                    covers.pop(i)
                covers[in_solution.index(site)] = chunk
    if len(covers) == 1:
        feasibility = True
    else:
        feasibility = False
    if save == True:
        w = shapefile.Writer(shapefile.POLYGON)
        w.field('net')
        for key in covers:
            w.poly(parts=[[list(x) for x in list(covers[key].exterior.coords)]])
            w.record('ff')
        w.save(path + "area")        
    return feasibility
    
#network-based feasibility check? 


def nn_distance(in_solution):
    distance_list = []
    for site in in_solution:
        dis_list = []
        for i in [x for x in in_solution if not x == site]:
            dis_list.append((facil_shp[site].distance(facil_shp[i]), i))
        dis_list.sort()
        
        distance_list.append((dis_list[0][0], site, dis_list[0][1]))
    distance_list.sort()
    return distance_list
        
def removal(in_solution, remove_no):
    nn_dist = nn_distance(in_solution)
    
    #print nn_dist
    for i in range(remove_no):
        if nn_dist[i][2] in in_solution:
            in_solution.remove(nn_dist[i][2])
    return in_solution
                          
    
    

def restricted_cadidates(in_solution):   #for spatial_interchange: 
    candis = []
    for site in in_solution:
        for i in F_Fdict[site]:
            if i[1] > min_dist:
                candis.append(i[0])
    too_close = []
    for site in in_solution:
        too_close.extend(F_F_close_d[site])
    
        
    candis = list(set(candis))
    candis = [x for x in candis if x not in in_solution]
    candis = [x for x in candis if x not in too_close]
    return candis



def spatial_interchange(in_solution):
    print "interchange start"
    c = restricted_cadidates(in_solution)
    print len(c)
    flag = True
    while flag == True:
        flag = False
        while len(c) != 0:
            candi = random.choice(c)
            c.remove(candi)
            current_obj = [in_solution, cal_obj(in_solution)]
            removable_solution = [x for x in in_solution if x not in warehouses_ID]
            for site in removable_solution:
                temp_sol = []
                temp_sol.extend(removable_solution)
                temp_sol.remove(site)
                temp_sol.append(candi)
                temp_sol.extend(warehouses_ID)
                temp_obj = cal_obj(temp_sol)
                if temp_obj > current_obj[1]:
                    if chk_isolation(temp_sol, warehouses_ID) == False: #prevent island in solution 
                        flag = True
                        current_obj = [temp_sol, temp_obj]
            if flag == True:
                in_solution = current_obj[0]
    print "interchange finished"
    return in_solution
        
def spatial_interchange_fast(in_solution):
    #print "interchange start"
    c = restricted_cadidates(in_solution)
    #print len(c)
    flag = True
    while flag == True:
        flag = False
        while len(c) != 0:
            candi = random.choice(c)
            c.remove(candi)
            current_obj = [in_solution, cal_obj(in_solution)]
            removable_solution = [x for x in in_solution if x not in warehouses_ID]
            for site in removable_solution:
                temp_sol = []
                temp_sol.extend(removable_solution)
                temp_sol.remove(site)
                temp_sol.append(candi)
                temp_sol.extend(warehouses_ID)
                temp_obj = cal_obj(temp_sol)
                if temp_obj > current_obj[1]:
                    if chk_isolation(temp_sol, warehouses_ID) == False: #prevent island in solution 
                        flag = True
                        current_obj = [temp_sol, temp_obj]
                        break
                    
            if flag == True:
                in_solution = current_obj[0]
                flag = False
                break
            
    #print "interchange finished"
    return in_solution

def spatial_interchage_mk2(in_solution):
    flag = True
    while flag == True:
        current_obj = [in_solution, cal_obj(in_solution)]
        removable_solution = [x for x in in_solution if x not in warehouses_ID]
        for site in removable_solution:
            temp_sol = copy.copy(removable_solution)
            temp_sol.remove(site)
            candis = restricted_cadidates(temp_sol)
            for c in candis:
                temp2_sol = copy.copy(temp_sol)
                if c == site:
                    continue
                else:
                    temp2_sol.append(c)
                    temp2_sol.extend(warehouses_ID)
                    temp2_obj = cal_obj(temp2_sol)
                    if temp2_obj > current_obj[1]:
                        if chk_feasibility_all(temp2_sol, False):
                            flag = False
                            in_solution = []
                            in_solution = copy.copy(temp2_sol)
                            
                            break
            if flag == False:
                break
    return in_solution
            
            
def greedy_fill(in_solution=[]):
    isolation = True
    tt = 0 
    
    while isolation == True:
        stime= time.time()
        new_sol = [] 
        new_sol = copy.copy(in_solution)
        c_obj = cal_obj(new_sol)
        while len(new_sol) < p:
            pool = restricted_cadidates(new_sol)
            temp = []
            for i in pool:
                temp_obj = cal_obj(new_sol + [i])
                temp.append((temp_obj, i))
            temp.sort()
            temp.reverse()
            c_obj = temp[0][0]
            new_sol = new_sol + [temp[0][1]]
        if chk_feasibility_all(new_sol, False):
            in_solution =[]
            in_solution = copy.copy(new_sol)
            isolation = False
        etime = time.time()
        tt += etime - stime
        if tt > 20: 
            print tt
            print new_sol       
            chk_feasibility_all(new_sol, True)
            f = raw_input()        
    return in_solution

def random_fill(in_solution=[]):
    isolation = True
    tt = 0
    while isolation == True:
        stime = time.time()
        new_sol = []
        new_sol = copy.copy(in_solution)
        while len(new_sol) < p:
            random_pool = restricted_cadidates(new_sol)
            new_sol.append(random.choice(random_pool))
        if chk_feasibility_all(new_sol, False):
            in_solution = []
            in_solution = copy.copy(new_sol)
            isolation = False
        etime = time.time()
        tt += etime - stime
        if tt > 20: 
            print tt
            print new_sol       
            chk_feasibility_all(new_sol, True)
            f = raw_input()
    return in_solution



f_FF = open(path + ffDict)
f_FD = open(path + fdDict)
f_demand = open(path + demand_Dict, 'rb')
F_Fdict = cPickle.load(f_FF)
F_Ddict = cPickle.load(f_FD)

facil_pysal = pysal.IOHandlers.pyShpIO.shp_file(path+facilities_f)
demand_pysal = pysal.IOHandlers.pyShpIO.shp_file(path + demands_f)


dDict = cPickle.load(f_demand)
facil_shp = generateGeometry(facil_pysal)
demand_shp = generateGeometry(demand_pysal)
warehouses_ID = [252,359,374,419]    #id_f of warehouses
solution_sites = []
covered_demand = []
objective_value = 0
p = 15  # 
temperature = 50   #end temperature
max_iter = 5   #iteration limit
terminate_temp = 1         
temp_ratio = 0.15
sa_count = 0
remove_percent = 0.2 
fd_fullPayload = 5 * 5280 
min_dist = fd_fullPayload * 0.6
fd_empty = 10 * 5280
fd_delivery = 3.33 *5280 
rc = 0.000001

            
F_F_close_d = defaultdict(list)
for i in F_Fdict:
    if F_Fdict[i][1] <= min_dist:
        F_F_close_d[i].append(F_Fdict[i][0])



#initializing seed solution (random) 

#isol_chk = True
#while isol_chk == True:
    #print "isolation"
    #solution_sites = []
    #solution_sites.extend(warehouses_ID)
    #while len(solution_sites) < p:
        
        #random_pool = restricted_cadidates(solution_sites)
        #solution_sites.append(random.choice(random_pool))
    
    #isol_chk = chk_isolation(solution_sites, warehouses_ID)
print "initializing solution"
solution_sites.extend(warehouses_ID)
solution_sites = greedy_fill(solution_sites)    

print "solution initialized"
print solution_sites
print cal_obj(solution_sites)

jjj = raw_input()
while temperature > 0.5:
    
    current_solution = copy.copy(solution_sites)
    current_obj = cal_obj(current_solution)
    #print "current Objective value: ", current_obj
    print "current solution: ", current_solution
    new_solution = copy.copy(current_solution)
    remove_number = int(remove_percent * p)
    print "remove", remove_number
    new_solution = removal(new_solution, remove_number)
    print "removed: ", new_solution
    print "random fill start"
    new_solution = greedy_fill(new_solution)
    print new_solution
    print "spatial interchange"
    new_solution = spatial_interchage_mk2(new_solution)
    new_obj = cal_obj(new_solution)
    print new_obj
    #print new_obj - current_obj
    
    if new_obj > current_obj:
        solution_sites = new_solution
        sa_count = 0 
        print "new solution accepted"
        #print "new objective value: ", new_obj
        print "new solution: ", solution_sites
    else:
        if sa_count > max_iter:
            sa_count = 0
            temperature = temperature - (temperature * temp_ratio)
            print "new temperature: ", temperature
            if temperature < terminate_temp:
                break
        else:
            sa_count += 1
            #print (new_obj - current_obj)*rc/temperature
            print math.exp((new_obj - current_obj)*rc/temperature)
            if random.random() < math.exp((new_obj - current_obj)*rc/temperature):
                solution_sites = new_solution
                #print "bad solution accepted"
                #print "new but bad objective: ", new_obj
                #print "new but bad solution: ", new_solution
            else:
                pass
                #print "bad solution reputed"
            
                
print cal_obj(solution_sites)
print solution_sites        
            
    
    
        
        
        
        
        
