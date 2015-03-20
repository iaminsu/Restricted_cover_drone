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

#mk4:
# dual objective, but minimizing both of them
# by turning demand obj into uncovered demand 


#mk5:
# new approach: no dual objective, only considering covered demand for obejctive value 
# minimizing graph generation & evaluation 
# eliminate redundancy 
# new interchagne algorithm 

# Bug fix: network removal
#new interchange algorithm 

#Issues: solution is fixed after some point. Need to change either or both of greedy_fill and 
#        spatial_interchange_mk3


#Mk5.2 
#fix issue with new dictionary files 

#Mk5.3
#modification in SA algorithm
# - "remember" the best solution ever, and if the final solution is inferior then the recored best solution, roll back to the best. 

#Mk5.4
#modification for new dictionary 

import pysal,  shapefile, networkx, time, cPickle, random, math, copy, Convexpath_module
from shapely.geometry import Point, Polygon, LineString, MultiPoint, MultiPolygon
from collections import defaultdict


path = "/Users/insuhong/Dropbox/research/Distance restricted covering model/Locating recharging station/data_all_ph2/"
ffDict = "FF_Dictfacils.shp_facils.shp_obstacles_p.shp.txt"

fdDict = "FD_Dictfacils.shp_demands.shp_obstacles_p.shp.txt"
demand_Dict = "demands.txt"
facilities_f = "facils.shp"
demands_f = "demands.shp"
obstacles_f = "obstacles_p"
ffcords = "FF_coords_Dict_facils.shp_obstacles_p.shp.txt"




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
    #print covered
    for de in covered:
        obj += float(dDict[de])
    return obj




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
                          
def delivery_network(in_solution, s_file, in_name = "temp_graph"):
    arc_list = []
    arc_shp_list = []
    connectivity = True
    resultingGraph = networkx.Graph()
    for i in range(len(in_solution)-1):
        sites = F_Fdict[in_solution[i]].keys()
        for j in range(i+1, len(in_solution)):
            if in_solution[j] in sites:
                resultingGraph.add_edge((facil_shp[in_solution[i]].x, facil_shp[in_solution[i]].y), (facil_shp[in_solution[j]].x, facil_shp[in_solution[j]].y), weight = F_Fdict[in_solution[i]][in_solution[j]][0])
                if F_Fdict[in_solution[i]][in_solution[j]][1] == "straight":
                    arc_list.append((in_solution[i], in_solution[j]))
                else:
                    arc_list.append("ESP_" + str(in_solution[i]) + "_" + str(in_solution[j]) + ".shp")

    for i in range(len(warehouse_coords)-1):
        for j in range(i+1, len(warehouse_coords)):
            try:
                route = networkx.dijkstra_path(resultingGraph, warehouse_coords[i], warehouse_coords[j])
            except:
                connectivity = False
                break
        if connectivity == False:
            break
    for site in in_solution:
        for whouse in warehouse_coords:
            try:
                route = networkx.dijkstra_path(resultingGraph, (facil_shp[site].x, facil_shp[site].y), whouse)
            except:
                connectivity = False
                break
        if connectivity == False:
            break
    
    if connectivity == True:
        if s_file == True:
            for i in arc_list:
                if type(i) == tuple:
                    arc_shp_list.append(LineString([list(facil_shp[i[0]].coords)[0], list(facil_shp[i[1]].coords)[0]]))
                else:
                    arc_pysal = pysal.IOHandlers.pyShpIO.shp_file(path+i)
                    arc_shp = generateGeometry(arc_pysal)
                    arc_shp_list.extend(arc_shp)            
            w = shapefile.Writer(shapefile.POLYLINE)
            w.field('nem')
            for line in arc_shp_list:
                
                w.line(parts=[[ list(x) for x in list(line.coords)]])
                w.record('chu')
            w.save(path + in_name)
        return resultingGraph
    else:
        return None
    
    
def delivery_network_mk2(in_solution, s_file, in_name = "temp_graph"):
    arc_list = []
    arc_shp_list = []
    connectivity = True
    resultingGraph = networkx.Graph()
    for i in range(len(in_solution)-1):
        sites = F_Fdict[in_solution[i]].keys()
        for j in range(i+1, len(in_solution)):
            if in_solution[j] in sites:
                resultingGraph.add_edge((facil_shp[in_solution[i]].x, facil_shp[in_solution[i]].y), (facil_shp[in_solution[j]].x, facil_shp[in_solution[j]].y), weight = F_Fdict[in_solution[i]][in_solution[j]][0])
                if F_Fdict[in_solution[i]][in_solution[j]][1] == "straight":
                    arc_list.append((in_solution[i], in_solution[j]))
                else:
                    arc_list.append("ESP_" + str(in_solution[i]) + "_" + str(in_solution[j]) + ".shp")
    
    if s_file == True:
        for i in arc_list:
            if type(i) == tuple:
                arc_shp_list.append(LineString([list(facil_shp[i[0]].coords)[0], list(facil_shp[i[1]].coords)[0]]))
            else:
                arc_pysal = pysal.IOHandlers.pyShpIO.shp_file(path+i)
                arc_shp = generateGeometry(arc_pysal)
                arc_shp_list.extend(arc_shp)          
        w = shapefile.Writer(shapefile.POLYLINE)
        w.field('nem')
        for line in arc_shp_list:
            w.line(parts=[[ list(x) for x in list(line.coords)]])
            w.record('chu')
        w.save(path + in_name)
    return resultingGraph

def delivery_network_mk3(in_solution, s_file, in_name = "temp_graph"):
    #check connectivity between warehouses 
    arc_list = []
    arc_shp_list = []
    connectivity = True
    resultingGraph = networkx.Graph()
    for i in range(len(in_solution)-1):
        sites = F_Fdict[in_solution[i]].keys()
        for j in range(i+1, len(in_solution)):
            if in_solution[j] in sites:
                resultingGraph.add_edge((facil_shp[in_solution[i]].x, facil_shp[in_solution[i]].y), (facil_shp[in_solution[j]].x, facil_shp[in_solution[j]].y), weight = F_Fdict[in_solution[i]][in_solution[j]][0])
                if F_Fdict[in_solution[i]][in_solution[j]][1] == "straight":
                    arc_list.append((in_solution[i], in_solution[j]))
                else:
                    arc_list.append("ESP_" + str(in_solution[i]) + "_" + str(in_solution[j]) + ".shp")
    
    for i in range(len(warehouse_coords)-1):
        for j in range(i+1, len(warehouse_coords)):
            try:
                route = networkx.dijkstra_path(resultingGraph, warehouse_coords[i], warehouse_coords[j])
            except:
                connectivity = False
                break
        if connectivity == False:
            break
    
    if connectivity == True:
        if s_file == True:
            for i in arc_list:
                if type(i) == tuple:
                    arc_shp_list.append(LineString([list(facil_shp[i[0]].coords)[0], list(facil_shp[i[1]].coords)[0]]))
                else:
                    arc_pysal = pysal.IOHandlers.pyShpIO.shp_file(path+i)
                    arc_shp = generateGeometry(arc_pysal)
                    arc_shp_list.extend(arc_shp)          
            w = shapefile.Writer(shapefile.POLYLINE)
            w.field('nem')
            for line in arc_shp_list:
                w.line(parts=[[ list(x) for x in list(line.coords)]])
                w.record('chu')
            w.save(path + in_name)
        return resultingGraph
    else:
        return None
    
def generate_graph(in_solution):
    arc_list = []
    arc_shp_list = []
    
    for i in range(len(in_solution)-1):
        sites = [x[0] for x in F_Fdict[in_solution[i]]]
        for j in range(i+1, len(in_solution)):
            if in_solution[j] in sites:
                arc_list.append("ESP_" + str(in_solution[i]) + "_" + str(in_solution[j]) + ".shp")
    resultingGraph = networkx.Graph()
    for arc in arc_list:
        arc_pysal = pysal.IOHandlers.pyShpIO.shp_file(path+arc)
        arc_shp = generateGeometry(arc_pysal)
        arc_shp_list.extend(arc_shp)
        for line in arc_shp:
            
            resultingGraph.add_edge(list(line.coords)[0], list(line.coords)[1], weight = line.length)
    w = shapefile.Writer(shapefile.POLYLINE)
    w.field('nem')
    for line in arc_shp_list:
        
        w.line(parts=[[ list(x) for x in list(line.coords)]])
        w.record('chu')
    w.save(path + "in_name")    
    
    return resultingGraph

def restricted_cadidates(in_solution):   #for spatial_interchange: 
    candis = []
    too_close = []
    for site in in_solution:
        for i in F_Fdict[site].keys():
            if F_Fdict[site][i][0] > min_dist:
                candis.append(i)
            else:
                too_close.append(i)
    candis = list(set(candis))
    too_close = list(set(too_close))
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
            candis = [x for x in candis if not x == site]
            for c in candis:
                temp2_sol = copy.copy(temp_sol)
                if c == site:
                    continue
                else:
                    temp2_sol.append(c)
                    temp2_sol.extend(warehouses_ID)
                    temp2_obj = cal_obj(temp2_sol)
                    if temp2_obj > current_obj[1]:
                        if delivery_network_mk3(temp2_sol, False) != None:
                        #if chk_feasibility_all(temp2_sol, False):
                            flag = False
                            in_solution = []
                            in_solution = copy.copy(temp2_sol)
                            
                            break
            if flag == False:
                break
    return in_solution

def spatial_interchage_mk3(in_solution):
    #modified interchange algorithm
    #conventional interchange algorithm cannot be applied since candidate set needs to be updated after any change in
    #current solution. So, this interchange algorithm iterate for each site in current solution, 
    #1) if a site is critical site: find better site that can maintaining connection 
    #2) if a site is not critical site: find better site from restricted candidate set for all other sites in current solution 
    
    current_obj = [in_solution, cal_obj(in_solution)]
    in_graph = delivery_network_mk2(in_solution, False)
    temp_sol = copy.copy(in_solution)        
    for site in temp_sol:
        if site not in warehouses_ID:
            temp_sol2 = copy.copy(in_solution)
            temp_sol2.remove(site)
            if delivery_network_mk3(temp_sol2, False) == None:  #site is critical node
                #then only candidates that can restablish connection are considered
                adj_nodes = in_graph[(facil_shp[site].x, facil_shp[site].y)].keys()
                candis = restricted_cadidates([adj_nodes[0]])
                
                for i in adj_nodes:
                    candis = [x for x in candis if x in restricted_cadidates[[i]]]
                for c in candis:
                    temp2_obj = cal_obj(temp_sol2 + [c])
                    if temp2_obj > current_obj[1]:
                        in_solution = temp_sol2 + [c]
                        current_obj = [in_solution, cal_obj(in_solution)]
            else:  #non-critical node
                candis = restricted_cadidates(temp_sol2)
                for c in candis:
                    temp2_obj = cal_obj(temp_sol2 + [c])
                    if temp2_obj > current_obj[1]:
                        if delivery_network(temp_sol2, False) != None:
                            in_solution = temp_sol2 + [c]
                            current_obj = [in_solution, cal_obj(in_solution)]                        
                    

    return in_solution


def spatial_interchage_mk4(in_solution):
    #modified interchange algorithm
    #conventional interchange algorithm cannot be applied since candidate set needs to be updated after any change in
    #current solution. This interchange algorithm *change only 1* site!!
    #1) if a site is critical site: find better site that can maintaining connection 
    #2) if a site is not critical site: find better site from restricted candidate set for all other sites in current solution 
    
    current_obj = [in_solution, cal_obj(in_solution)]
    in_graph = delivery_network_mk2(in_solution, False)
    temp_sol = copy.copy(in_solution)        
    for site in temp_sol:
        indi = False
        if site not in warehouses_ID:
            temp_sol2 = copy.copy(in_solution)
            temp_sol2.remove(site)
            if delivery_network_mk3(temp_sol2, False) == None:  #site is critical node
                #then only candidates that can restablish connection are considered
                adj_nodes = in_graph[(facil_shp[site].x, facil_shp[site].y)].keys()
                candis = restricted_cadidates([adj_nodes[0]])
                
                for i in adj_nodes:
                    
                    if len(candis) == 0:
                        candis = restricted_cadidates([F_FCoords[i]])
                    else:
                        candis = [x for x in candis if x in restricted_cadidates([F_FCoords[i]])]
                
                
                for c in candis:
                    temp2_obj = cal_obj(temp_sol2 + [c])
                    if temp2_obj > current_obj[1]:
                        in_solution = temp_sol2 + [c]
                        current_obj = [in_solution, cal_obj(in_solution)]
                        indi = True
            else:  #non-critical node
                candis = restricted_cadidates(temp_sol2)
                for c in candis:
                    temp2_obj = cal_obj(temp_sol2 + [c])
                    if temp2_obj > current_obj[1]:
                        if delivery_network(temp_sol2, False) != None:
                            in_solution = temp_sol2 + [c]
                            current_obj = [in_solution, cal_obj(in_solution)]                        
                            indi = True
            
                    
        if indi == True:
            break
    return in_solution





def greedy_fill(in_solution=[]):
    isolation = True
    tt = 0 
    
    while isolation == True:
        obj_time = 0 
        new_sol = [] 
        stime = time.time()
        new_sol = copy.copy(in_solution)
        c_obj = cal_obj(new_sol)
        loop_no = 0
        pool_len = 0 
        while len(new_sol) < p:
            loop_no += 1
            #print new_sol
            pool = restricted_cadidates(new_sol)
            pool_len += len(pool)
            temp = []
            stime_l = time.time()
            for i in pool:
                temp_obj = cal_obj(new_sol + [i])
                temp.append((temp_obj, i))
            etime_l = time.time()
            obj_time += etime_l - stime_l
            temp.sort()
            temp.reverse()
            c_obj = temp[0][0]
            new_sol = new_sol + [temp[0][1]]
        if delivery_network(new_sol, False) != None:
        #if chk_feasibility_all(new_sol, False):
            in_solution =[]
            in_solution = copy.copy(new_sol)
            isolation = False
        etime = time.time()
        tt += etime - stime
        if tt > 600: 
            print "greedy failed"
            print tt
            print new_sol
            nn = delivery_network_mk2(new_sol, True, "failed_greedy")
            chk_feasibility_all(new_sol, True)
            f = raw_input()        
    #print "total time: ", tt
    #print "obj time: ", obj_time
    #print "average pool: ", float(pool_len)/loop_no
    return in_solution

def greedy_fill_mk2(in_solution):
    pass


def random_fill(in_solution=[]):
    isolation = True
    tt = 0
    while isolation == True:
        
        stime = time.time()
        new_sol = []
        new_sol = copy.copy(in_solution)
        while len(new_sol) < p:
            random_pool = restricted_cadidates(new_sol)
            print "pool size:", len(random_pool)
            new_sol.append(random.choice(random_pool))
        if delivery_network(new_sol, False) != None:
            in_solution = []
            in_solution = copy.copy(new_sol)
            isolation = False
        else:
            delivery_network_mk2(new_sol, True, "test")
            print "WTF"
            print "S"
        #etime = time.time()
        #tt += etime - stime
        #if tt > 20: 
            #print tt
            #print new_sol       
            #chk_feasibility_all(new_sol, True)
            #f = raw_input()
    return in_solution

def random_fill_mk2(in_solution):
    
    # 1)2 warehouses case:
    #  - generate a corridor using ESP between them 
    #  - random select facilities in the corridor until warehoused are connected 
    #  - random select remaining facilities 
    # 2)More-than-2 warehouses case:
    #  - generate a convex hull for warehouses 
    #  - derive centroid of convex hull 
    #  - generate a corridor based on the ESPs that connect from warehoused to centroid 
    #  - random select facilities in the corridor until warehoused are connected 
    #  - random select remaining facilities 
    isolation = True
    if len(warehouses_ID) == 2:
        w_origin = facil_shp[warehouses_ID[0]]
        w_destination = facil_shp[warehouses_ID[1]]
        a = Convexpath_module.Convexpath_shapely(path, w_origin, w_destination, obstacles_shp)
        w_esp = a.esp  #esp is Linestring object
        w_corridor = w_esp.buffer(fd_delivery*0.5)
        
    else:
        pass
    
    while isolation == True: 
        new_sol = []
        new_sol = copy.copy(in_solution)
        
        while len(new_sol) < p:
            if delivery_network_mk3(new_sol, False) == None:
                random_pool = restricted_cadidates(new_sol)
                corridor_pool = []
                for i in random_pool:
                    if w_corridor.intersects(facil_shp[i]):
                        corridor_pool.append(i)
                new_sol.append(random.choice(corridor_pool))
            else:
                random_pool = restricted_cadidates(new_sol)
                new_sol.append(random.choice(random_pool))
        if delivery_network(new_sol, False) != None:
            in_solution = []
            in_solution = copy.copy(new_sol)
            isolation = False
    return in_solution
                
        

def network_removal (in_solution):
    #remove certain number of sites from solution. But if a site is part of critical link between warehouses, 
    #the site will not be removed. 
    #sites are randomly selected (not based on nn distance)
    #if some sites are separated from the delivery network, remove them also regardless of removal number. 
    remove_no = int(remove_percent * len(in_solution))
    sol_wo_wh = [x for x in in_solution if not x in warehouses_ID]
    while remove_no > 0:
        r_site = random.choice(sol_wo_wh)
        temp = copy.copy(sol_wo_wh)
        temp.extend(warehouses_ID)
        temp.remove(r_site)
        temp_graph = delivery_network(temp)
        #print temp
        #print remove_no
        if temp_graph != None:
            sol_wo_wh.remove(r_site)
            remove_no -= 1
            

    sol_wo_wh.extend(warehouses_ID)
    temp_graph = delivery_network(sol_wo_wh)
    additional_removal = []
    
    for site in sol_wo_wh:
        if site not in warehouses_ID:
             
            site_coords = (facil_shp[site].x, facil_shp[site].y)
            
            for whouse in warehouse_coords:
                try:
                    route = networkx.dijkstra_path(temp_graph, site_coords, whouse)
                    
                except:
                    
                    additional_removal.append(site)
                    break
    sol_wo_wh = [x for x in sol_wo_wh if not x in additional_removal]
                    
                
        
    return sol_wo_wh

def network_removal_mk2 (in_solution):
    #remove certain number of sites from solution. But if a site is part of critical link between warehouses, 
    #the site will not be removed. 
    #sites are randomly selected (not based on nn distance)
    #if some sites are separated from the delivery network, remove them also regardless of removal number. 
    remove_no = int(remove_percent * len(in_solution))
    #print remove_no
    removable_sites = []
    #print "in_solution:", in_solution
    for site in in_solution:
        if site not in warehouses_ID:
            temp = copy.copy(in_solution)
            temp.remove(site)
            temp_graph = delivery_network_mk3(temp, False)
            
            if temp_graph != None:
                removable_sites.append(site)
    if len(removable_sites) < remove_no:
        remove_no = len(removable_sites)
    #print "removeable,", removable_sites
    #print remove_no
    while remove_no > 0:
        r_site = random.choice(removable_sites)
        removable_sites.remove(r_site)
        in_solution.remove(r_site)
        temp_graph2 = delivery_network_mk3(in_solution, False)
        if temp_graph2 == None:
            in_solution.append(r_site)
        else:
            remove_no -= 1
    
    #print "removed", in_solution
    temp_graph = delivery_network_mk2(in_solution, True)
    additional_removal = []
    
    #print temp_graph 
    for site in in_solution:
        if site not in warehouses_ID:
            site_coords = (facil_shp[site].x, facil_shp[site].y)
            for whouse in warehouse_coords:
                
                try:
                    route = networkx.dijkstra_path(temp_graph, site_coords, whouse)
                except networkx.exception.NetworkXNoPath:
                    additional_removal.append(site)
                    break
                
                except KeyError:
                    additional_removal.append(site)
    in_solution = [x for x in in_solution if not x in additional_removal]
    if len(in_solution) < 5:
        print "shit again?"
        print "r", additional_removal
        print in_solution
        r = raw_input()
                
        
    return in_solution



f_FF = open(path + ffDict)
f_FD = open(path + fdDict)

f_demand = open(path + demand_Dict, 'rb')
F_Fdict = cPickle.load(f_FF)     


#F_Fdict2 = cPickle.load(open(path + ff2Dict))
F_Ddict = cPickle.load(f_FD)
F_FCoords = cPickle.load(open(path+ ffcords))
facil_pysal = pysal.IOHandlers.pyShpIO.shp_file(path+facilities_f)
demand_pysal = pysal.IOHandlers.pyShpIO.shp_file(path + demands_f)
obstacles_pysal = pysal.IOHandlers.pyShpIO.shp_file(path + obstacles_f)

dDict = cPickle.load(f_demand)
facil_shp = generateGeometry(facil_pysal)
demand_shp = generateGeometry(demand_pysal)
obstacles_shp = generateGeometry(obstacles_pysal)
warehouses_ID = [698, 1248]    #id_f of warehouses
warehouse_coords = []
for warehouse in warehouses_ID:
    warehouse_coords.append((facil_shp[warehouse].x, facil_shp[warehouse].y))
solution_sites = []
covered_demand = []
objective_value = 0
p = 25  # 
temperature = 30   #end temperature
max_iter = 3   #iteration limit
terminate_temp = 1         
temp_ratio = 0.15
sa_count = 0
remove_percent = 0.2 
fd_fullPayload = 5 * 5280 

fd_empty = 10 * 5280
fd_delivery = 3.33 *5280 
min_dist = fd_delivery
rc = 0.001
rc_obj = 0.1
total_demand = 0.0 
            

for i in dDict:
    total_demand += float(dDict[i])


#initializing seed solution (random) 



print "initializing solution"

solution_sites.extend(warehouses_ID)
solution_sites = random_fill_mk2(solution_sites)   
#print solution_sites

solution_graph = delivery_network_mk2(solution_sites, True, "initial_sol")
print "solution initialized"
print solution_sites
print len(solution_sites)

best_solution = [solution_sites, cal_obj(solution_sites)]


while temperature > 0.5:
    
    current_solution = copy.copy(solution_sites)
    current_graph = delivery_network_mk2(current_solution, True, "currrent_graph")
    current_obj = cal_obj(current_solution)
    print "current Objective value: ", current_obj
    
    new_solution = copy.copy(current_solution)
    s_time = time.time()
    new_solution = network_removal_mk2 (new_solution)
    e_time = time.time()
    #print "removed", new_solution
    #print "removal time 2: ", e_time - s_time
    #print "fill start"
    #print "removed obj: ", cal_obj(new_solution)
    new_solution = spatial_interchage_mk4(new_solution)
    #print "improved obj before greedy: ", cal_obj(new_solution)
    s_time = time.time()
    new_solution = greedy_fill(new_solution)
    n_graph = delivery_network_mk2(new_solution, True, "greey_graph")
    e_time = time.time()
    #print "fill time: ", e_time - s_time
    #print new_solution
    #print "spatial interchange start"
    s_time = time.time()
    new_solution = spatial_interchage_mk4(new_solution)
    e_time = time.time()
    #print "interchange time: ", e_time - s_time
    new_graph = delivery_network_mk2(new_solution, True, "new_solution")
    new_obj = cal_obj(new_solution)
    print new_obj
    #print new_obj - current_obj
    

    if new_obj > current_obj:
        if new_obj < best_solution[1]:
            
            sa_count += 1
            print "SA COUNT: ", sa_count
            if random.random() < math.exp((new_obj - best_solution[1])*rc/temperature):
                solution_sites = new_solution            
                print "new but not best"
                if sa_count >= max_iter:
                    sa_count = 0
                    temperature = temperature - (temperature * temp_ratio)
                    print "new temperature: ", temperature
                    if temperature < terminate_temp:
                        break                 
            else:
                solution_sites = best_solution[0]
                print "ignore new one roll back to the best"
                if sa_count >= max_iter:
                    sa_count = 0
                    temperature = temperature - (temperature * temp_ratio)
                    print "new temperature: ", temperature
                    if temperature < terminate_temp:
                        break                    
                   
        else:
            solution_sites = new_solution
            best_solution = [new_solution, new_obj]
            sa_count = 0 
            print "new solution accepted"
        print "best solution so far: ", best_solution[1]
        #print "new objective value: ", new_obj
        #print "new solution: ", solution_sites
    else:
        if sa_count >= max_iter:
            sa_count = 0
            temperature = temperature - (temperature * temp_ratio)
            print "new temperature: ", temperature
            if temperature < terminate_temp:
                break
        else:
            sa_count += 1
            print "SA COUNT: ", sa_count
            #print (new_obj - current_obj)*rc/temperature
            if math.exp((new_obj - current_obj)*rc/temperature) == 1:
                print "stucted"
                print "s"
            if random.random() < math.exp((new_obj - current_obj)*rc/temperature):
                solution_sites = new_solution
                #print "bad solution accepted"
                print "new but bad objective: ", new_obj
                #print "new but bad solution: ", new_solution
                if sa_count >= max_iter:
                    sa_count = 0
                    temperature = temperature - (temperature * temp_ratio)
                    print "new temperature: ", temperature
                    if temperature < terminate_temp:
                        break                
            else:
                if sa_count >= max_iter:
                    sa_count = 0
                    temperature = temperature - (temperature * temp_ratio)
                    print "new temperature: ", temperature
                    if temperature < terminate_temp:
                        break                    
                        
            
            
print "solution"                
solution_obj = cal_obj(solution_sites)
if solution_obj > best_solution[1]:
    print "final solution: ", solution_sites
    print "Objective value: ", solution_obj
    final_graph = delivery_network_mk2(solution_sites, True, "final_solution")
else:
    print "final solution: ", best_solution[0]
    print "Objective value: ", best_solution[1]
    final_graph = delivery_network_mk2(best_solution[0], True, "final_solution")


            
    
    
        
        
        
        
        
