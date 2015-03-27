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

#Mk6.3 
#(from 5.3)
# New objective function: number of covered population is adjusted by average deviation ratio. 
# Deviation ratio: for each selecte site, (network shortest distance)/ESP distance
# for greedy and random fill
#random fill bug fixed: centroid of corridor cannot be located in interior of obstacle 


import pysal,  shapefile, networkx, time, cPickle, random, math, copy, Convexpath_module
from shapely.geometry import Point, Polygon, LineString, MultiPoint, MultiPolygon
from collections import defaultdict
from shapely.ops import cascaded_union


path = "/Users/insuhong/Dropbox/research/Distance restricted covering model/Locating recharging station/data4/"
ffDict = "FF_old_Dictsample_sites_2.shp_sample_demand_2_p.shp_obstacles_p.shp.txt"
obstacles_f = "obstacles_p"

fdDict = "FD_Dictsample_sites_2.shp_sample_demand_2_p.shp_obstacles_p.shp.txt"
demand_Dict = "demands.txt"
facilities_f = "sample_sites_2.shp"
demands_f = "sample_demand_2_p.shp"
ffcords = "FF_coords_Dictsample_sites_2.shp_sample_demand_2_p.shp_obstacles_p.shp.txt"
wareDist = "w_distance_dict.txt"
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

def cal_obj_mk2(in_solution):
    #Calculate obj with deviation ratio
    sol_graph = delivery_network_mk2(in_solution, False)
    ratio_l = []
    for w in warehouses_ID:
        for site in in_solution:
            if site != w:
                try:
                    n_dist = networkx.dijkstra_path_length(sol_graph, (facil_shp[w].x,facil_shp[w].y), (facil_shp[site].x,facil_shp[site].y))
                except:
                    print in_solution
                    ss = delivery_network_mk2(in_solution, True, "err_graph")
                    raise networkx.exception.NetworkXNoPath
                d_ratio = n_dist/ware_dist[w][site]
                ratio_l.append(d_ratio)
    avg_ratio = sum(ratio_l)/float(len(ratio_l))
    obj_n = cal_obj(in_solution)
    obj_r = obj_n * 1/avg_ratio
    return obj_r

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
    #Generate graph
    #Check connectivity between warehouses
    #Check connectivity to each site 
    arc_list = []
    arc_shp_list = []
    connectivity = True
    resultingGraph = networkx.Graph()
    for i in range(len(in_solution)-1):
        sites = [x[0] for x in F_Fdict[in_solution[i]]]
        for j in range(i+1, len(in_solution)):
            if in_solution[j] in sites:
                resultingGraph.add_edge((facil_shp[in_solution[i]].x, facil_shp[in_solution[i]].y), (facil_shp[in_solution[j]].x, facil_shp[in_solution[j]].y), weight = F_Fdict2[in_solution[i]][in_solution[j]])
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
    #Generate graph, does not check connectivity
    arc_list = []
    arc_shp_list = []
    connectivity = True
    resultingGraph = networkx.Graph()
    for i in range(len(in_solution)-1):
        sites = [x[0] for x in F_Fdict[in_solution[i]]]
        for j in range(i+1, len(in_solution)):
            if in_solution[j] in sites:
                resultingGraph.add_edge((facil_shp[in_solution[i]].x, facil_shp[in_solution[i]].y), (facil_shp[in_solution[j]].x, facil_shp[in_solution[j]].y), weight = F_Fdict2[in_solution[i]][in_solution[j]])
                arc_list.append("ESP_" + str(in_solution[i]) + "_" + str(in_solution[j]) + ".shp")
    
    if s_file == True:
        for arc in arc_list:
            arc_pysal = pysal.IOHandlers.pyShpIO.shp_file(path+arc)
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
        sites = [x[0] for x in F_Fdict[in_solution[i]]]
        for j in range(i+1, len(in_solution)):
            if in_solution[j] in sites:
                resultingGraph.add_edge((facil_shp[in_solution[i]].x, facil_shp[in_solution[i]].y), (facil_shp[in_solution[j]].x, facil_shp[in_solution[j]].y), weight = F_Fdict2[in_solution[i]][in_solution[j]])
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


def spatial_interchage_mk5(in_solution):
    #modified interchange algorithm
    #conventional interchange algorithm cannot be applied since candidate set needs to be updated after any change in
    #current solution. This interchange algorithm *change only 1* site!!
    #1) if a site is critical site: find better site that can maintaining connection 
    #2) if a site is not critical site: find better site from restricted candidate set for all other sites in current solution 
    
    #mk5: include distance restriction for evaluation 
    
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
                        temp_graph = delivery_network_mk2(temp_sol2 + [c], False)
                        ind = True
                        for w in warehouses_ID:
                            dist = networkx.dijkstra_path_length(temp_graph, (facil_shp[w].x,facil_shp[w].y), (facil_shp[c].x,facil_shp[c].y))
                            if dist >= ware_dist[w][c] * dist_limit_ratio:
                                ind = False
                        if ind == True:
                            in_solution = temp_sol2 + [c]
                            current_obj = [in_solution, cal_obj(in_solution)]
                            indi = True
            else:  #non-critical node
                candis = restricted_cadidates(temp_sol2)
                for c in candis:
                    temp2_obj = cal_obj(temp_sol2 + [c])
                    if temp2_obj > current_obj[1]:
                        temp_graph = delivery_network_mk2(temp_sol2 + [c], False)
                        ind = True
                        for w in warehouses_ID:
                            dist = networkx.dijkstra_path_length(temp_graph, (facil_shp[w].x,facil_shp[w].y), (facil_shp[c].x,facil_shp[c].y))
                            if dist >= ware_dist[w][c] * dist_limit_ratio:
                                ind = False
                        if ind == True:
                            if delivery_network(temp_sol2, False) != None:
                                in_solution = temp_sol2 + [c]
                                current_obj = [in_solution, cal_obj(in_solution)]                        
                                indi = True

        if indi == True:
            break
    return in_solution



def spatial_interchage_mk6(in_solution):
    #modified interchange algorithm
    #conventional interchange algorithm cannot be applied since candidate set needs to be updated after any change in
    #current solution. This interchange algorithm *change only 1* site!!
    #1) if a site is critical site: find better site that can maintaining connection 
    #2) if a site is not critical site: find better site from restricted candidate set for all other sites in current solution 
    
    current_obj = [in_solution, cal_obj_mk2(in_solution)]
    in_graph = delivery_network_mk2(in_solution, False)
    temp_sol = copy.copy(in_solution)        
    for site in temp_sol:
        indi = False
        if site not in warehouses_ID:
            temp_sol2 = copy.copy(in_solution)
            temp_sol2.remove(site)
            
            adj_nodes = in_graph[(facil_shp[site].x, facil_shp[site].y)].keys()
            candis = restricted_cadidates([adj_nodes[0]])
            
            for i in adj_nodes:
                
                if len(candis) == 0:
                    candis = restricted_cadidates([F_FCoords[i]])
                else:
                    candis = [x for x in candis if x in restricted_cadidates([F_FCoords[i]])]
            
            
            for c in candis:
                temp2_obj = cal_obj_mk2(temp_sol2 + [c])
                if temp2_obj > current_obj[1]:
                    in_solution = temp_sol2 + [c]
                    current_obj = [in_solution, temp2_obj]
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
    #if ratio of network distance over ESP distance is less than given threshold, accept it. 
    
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
            
            for i in pool:
                temp_obj = cal_obj(new_sol + [i])
                temp.append((temp_obj, i))
            temp.sort()
            temp.reverse()
            for i in temp:
                temp_graph = delivery_network_mk2(new_sol+[i[1]], False)
                ind = True
                for w in warehouses_ID:
                    #dist = networkx.dijkstra_path(temp_graph, facil_shp[w], facil_shp[i[1]])
                    
                    dist = networkx.dijkstra_path_length(temp_graph, (facil_shp[w].x,facil_shp[w].y), (facil_shp[i[1]].x,facil_shp[i[1]].y))
                    
                    if dist >= ware_dist[w][i[1]] * dist_limit_ratio:
                        ind = False
                if ind == True:
                    c_obj = i[0]
                    new_sol = new_sol + [i[1]]
                    break
                        
        if delivery_network(new_sol, False) != None:
        #if chk_feasibility_all(new_sol, False):
            in_solution =[]
            in_solution = copy.copy(new_sol)
            isolation = False
    return in_solution


def greedy_fill_mk3(in_solution=[]):
    #same as mk1 except it uses cal_obj_mk2
    isolation = True
    while isolation == True:
        new_sol = [] 
        new_sol = copy.copy(in_solution)
        c_obj = cal_obj_mk2(new_sol)
        loop_no = 0
        pool_len = 0 
        while len(new_sol) < p:
            loop_no += 1
            pool = restricted_cadidates(new_sol)
            pool_len += len(pool)
            temp = []
            for i in pool:
                temp_obj = cal_obj_mk2(new_sol + [i])
                temp.append((temp_obj, i))
            temp.sort()
            temp.reverse()
            c_obj = temp[0][0]
            new_sol = new_sol + [temp[0][1]]
        if delivery_network(new_sol, False) != None:
            in_solution =[]
            in_solution = copy.copy(new_sol)
            isolation = False
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
        w_points = []
        for i in warehouse_coords:
            w_points.append(i)
        w_mp = MultiPoint(w_points)
        w_ch = w_mp.convex_hull
        w_cp = w_ch.centroid
        for obs in obstacles_shp:
            if obs.contains(w_cp):
                search_circle = w_cp.buffer(search_radius)
                for i in facil_shp:
                    if search_circle.contains(i):
                        w_cp = i
                        break
                break
        
        w_c_lines = []
        for i in warehouse_coords:
            a = Convexpath_module.Convexpath_shapely(path, Point(i), w_cp, obstacles_shp)
            w_c_lines.append(a.esp.buffer(fd_delivery * 0.5))
        w_corridor = cascaded_union(w_c_lines)

        
        
    w = shapefile.Writer(shapefile.POLYGON)
    w.field('net')
    for obs in [w_corridor]:
        w.poly(parts=[[list(x) for x in list(obs.exterior.coords)]])
        w.record('ff')
    w.save(path + "w_corridor")      
    while isolation == True: 
        new_sol = []
        new_sol = copy.copy(in_solution)
        
        while len(new_sol) < p:
            if delivery_network_mk3(new_sol, False) == None:
                random_pool = restricted_cadidates(new_sol)
                #print new_sol
                #print random_pool
                corridor_pool = []
                for i in random_pool:
                    if w_corridor.intersects(facil_shp[i]):
                        corridor_pool.append(i)
                if len(corridor_pool) != 0:
                    new_sol.append(random.choice(corridor_pool))
                else: 
                    new_sol.append(random.choice(random_pool))
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
ware_dist = cPickle.load(open(path + wareDist))
f_demand = open(path + demand_Dict, 'rb')
F_Fdict = cPickle.load(f_FF)     
F_Fdict2 = defaultdict(dict)
for i in F_Fdict:
    for j in F_Fdict[i]:
        F_Fdict2[i][j[0]] = j[1]

#F_Fdict2 = cPickle.load(open(path + ff2Dict))
F_Ddict = cPickle.load(f_FD)
F_FCoords = cPickle.load(open(path+ ffcords))
facil_pysal = pysal.IOHandlers.pyShpIO.shp_file(path+facilities_f)
demand_pysal = pysal.IOHandlers.pyShpIO.shp_file(path + demands_f)
obstacles_pysal = pysal.IOHandlers.pyShpIO.shp_file(path + obstacles_f)

obstacles_shp = generateGeometry(obstacles_pysal)
dDict = cPickle.load(f_demand)
facil_shp = generateGeometry(facil_pysal)
demand_shp = generateGeometry(demand_pysal)
warehouses_ID = [127,324, 411]    #id_f of warehouses
warehouse_coords = []
for warehouse in warehouses_ID:
    warehouse_coords.append((facil_shp[warehouse].x, facil_shp[warehouse].y))
solution_sites = []
covered_demand = []
objective_value = 0   #initialize objective value 
p = 20  #number of stations to be sited including existing warehouses 
temperature = 30   #start temperature
max_iter = 3   #iteration limit
terminate_temp = 1         
temp_ratio = 0.15   
sa_count = 0
remove_percent = 0.2 
fd_fullPayload = 5 * 5280    #flight distance with full payload
fd_empty = 10 * 5280         #flight distance with empty payload 
fd_delivery = 3.33 *5280     #max flight distance for safe return after delivery 
search_radius = 2 * 5280 
min_dist = fd_delivery
rc = 0.001
rc_obj = 0.1
total_demand = 0.0 
dist_limit_ratio = 1.4
            
F_F_close_d = defaultdict(list)
for i in F_Fdict:
    for j in F_Fdict[i]:
        if j[1] <= min_dist:
            F_F_close_d[i].append(j[0])

for i in dDict:
    total_demand += float(dDict[i])


#initializing seed solution (random) 



print "initializing solution"

solution_sites.extend(warehouses_ID)
solution_sites = random_fill_mk2(solution_sites)   
#print solution_sites

solution_graph = delivery_network_mk2(solution_sites, True)
print "solution initialized"

best_solution = [solution_sites, cal_obj_mk2(solution_sites)]

number = 0
while temperature > 0.5:
    number += 1
    current_solution = copy.copy(solution_sites)
    current_graph = delivery_network_mk2(current_solution, True, "currrent_graph_" + str(number))
    current_obj = cal_obj_mk2(current_solution)
    print "current Objective value: ", current_obj
    
    new_solution = copy.copy(current_solution)
    s_time = time.time()
    new_solution = network_removal_mk2 (new_solution)
    e_time = time.time()
    s_time = time.time()
    new_solution = greedy_fill_mk3(new_solution)
    n_graph = delivery_network_mk2(new_solution, True, "greey_graph")
    e_time = time.time()
    s_time = time.time()
    new_solution = spatial_interchage_mk6(new_solution)
    e_time = time.time()

    new_graph = delivery_network_mk2(new_solution, True, "new_solution_" + str(number))
    new_obj = cal_obj_mk2(new_solution)
    print new_obj

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
            if random.random() < math.exp((new_obj - current_obj)*rc/temperature):
                solution_sites = new_solution
                
                print "new but bad objective: ", new_obj
                
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
solution_obj = cal_obj_mk2(solution_sites)
if solution_obj > best_solution[1]:
    print "final solution: ", solution_sites
    print "Objective value_r: ", solution_obj
    print "Covered population: ", cal_obj(solution_sites)
    final_graph = delivery_network_mk2(solution_sites, True, "final_solution")
else:
    print "final solution: ", best_solution[0]
    print "Objective value_r: ", best_solution[1]
    print "Covered population: ", cal_obj(best_solution[0])
    final_graph = delivery_network_mk2(best_solution[0], True, "final_solution")


            
    
    
        
        
        
        
        
