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



#mk5_single:
# Add evaluation & removal functions for relaxed connectivity assumption. 
#  


import pysal,  shapefile, networkx, time, cPickle, random, math, copy
from shapely.geometry import Point, Polygon, LineString, MultiPoint, MultiPolygon
from collections import defaultdict

path = "/Users/insuhong/Dropbox/research/Distance restricted covering model/Locating recharging station/data4/"
ffDict = "FF_Dictsample_sites_2.shp_sample_demand_2_p.shp_obstacles_p.shp.txt"
ff2Dict = "FF2_Dictsample_sites_2.shp_sample_demand_2_p.shp_obstacles_p.shp.txt"
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
    #print covered
    for de in covered:
        obj += float(dDict[de])
    return obj






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


    
                          
def delivery_network(in_solution, s_file, in_name = "temp_graph"):
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


def delivery_network_mk3_single(in_solution, s_file, in_name = "temp_graph"):
     
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
    for site in in_solution:
        site_connectivity = False
        for whouse in warehouse_coords:
            try:
                route = networkx.dijkstra_path(resultingGraph, (facil_shp[site].x, facil_shp[site].y), whouse)
                site_connectivity = True
            except:
                pass
        if site_connectivity == False:
            connectivity = False
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
                    candis = [x for x in candis if x in restricted_cadidates[[i]]]
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


def spatial_interchage_mk4_single(in_solution):
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
            if delivery_network_mk3_single(temp_sol2, False) == None: 
                adj_nodes = in_graph[(facil_shp[site].x, facil_shp[site].y)].keys()
                candis = restricted_cadidates([adj_nodes[0]])

                for i in adj_nodes:
                    candis = [x for x in candis if x in restricted_cadidates[[i]]]
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
                        if delivery_network_mk3_single(temp_sol2, False) != None:
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
        stime= time.time()
        new_sol = [] 
        new_sol = copy.copy(in_solution)
        c_obj = cal_obj(new_sol)
        while len(new_sol) < p:
            #print new_sol
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
        if tt > 100: 
            print "greedy failed"
            print tt
            print new_sol       
            g = delivery_network_mk2(new_sol, True, "greedy_failed")
            f = raw_input()        
    return in_solution


def greedy_fill_single(in_solution=[]):

    new_solution = copy.copy(in_solution)
    c_obj = cal_obj(new_solution)
    while len(new_solution) < p:
        candis = restricted_cadidates(new_solution)
        temp = []
        for c in candis:
            temp_obj = cal_obj(new_solution + [c])
            temp.append((temp_obj, c))
        temp.sort()
        temp.reverse()
        new_solution = new_solution + [temp[0][1]]

    
    return new_solution


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

def random_fill_single(in_solution=[]):
    
    new_sol = copy.copy(in_solution)
    while len(new_sol) < p:
        random_pool = restricted_cadidates(new_sol)
        new_sol.append(random.choice(random_pool))
        
    return new_sol

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
    print remove_no
    removable_sites = []
    print "in_solution:", in_solution
    for site in in_solution:
        if site not in warehouses_ID:
            temp = copy.copy(in_solution)
            temp.remove(site)
            temp_graph = delivery_network_mk3(temp, False)
            
            if temp_graph != None:
                removable_sites.append(site)
    if len(removable_sites) < remove_no:
        remove_no = len(removable_sites)
    print "removeable,", removable_sites
    print remove_no
    while remove_no > 0:
        r_site = random.choice(removable_sites)
        removable_sites.remove(r_site)
        in_solution.remove(r_site)
        temp_graph2 = delivery_network_mk3(in_solution, False)
        if temp_graph2 == None:
            in_solution.append(r_site)
        else:
            remove_no -= 1
    
    print "removed", in_solution
    temp_graph = delivery_network_mk2(in_solution, True)
    additional_removal = []
    
    print temp_graph 
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

def network_removal_mk2_single (in_solution):
    #remove certain number of sites from solution. But if a site is part of critical link between warehouses, 
    #the site will not be removed. 
    #sites are randomly selected (not based on nn distance)
    #if some sites are separated from the delivery network, remove them also regardless of removal number. 
    remove_no = int(remove_percent * len(in_solution))
    removable_sites = [x for x in in_solution if not x in warehouses_ID]
    for i in range(remove_no):
        r_site = random.choice(removable_sites)
        removable_sites.remove(r_site)
        in_solution.remove(r_site)
    temp_graph = delivery_network_mk2(in_solution, False)
    additional_removal = []
    for site in in_solution:
        if site not in warehouses_ID:
            site_coords = (facil_shp[site].x, facil_shp[site].y)
            site_connectivity = False
            for warehouse in warehouse_coords:
                try:
                    route = networkx.dijkstra_path(temp_graph, site_coords, warehouse)
                    site_connectivity = True
                except:
                    pass
            if site_connectivity == False:
                additional_removal.append(site)
    in_solution = [x for x in in_solution if not x in additional_removal]
        
    
    return in_solution

f_FF = open(path + ffDict)
f_FD = open(path + fdDict)

f_demand = open(path + demand_Dict, 'rb')
F_Fdict = cPickle.load(f_FF)
F_Fdict2 = cPickle.load(open(path + ff2Dict))
F_Ddict = cPickle.load(f_FD)

facil_pysal = pysal.IOHandlers.pyShpIO.shp_file(path+facilities_f)
demand_pysal = pysal.IOHandlers.pyShpIO.shp_file(path + demands_f)


dDict = cPickle.load(f_demand)
facil_shp = generateGeometry(facil_pysal)
demand_shp = generateGeometry(demand_pysal)
warehouses_ID = [252,374]    #id_f of warehouses
warehouse_coords = []
for warehouse in warehouses_ID:
    warehouse_coords.append((facil_shp[warehouse].x, facil_shp[warehouse].y))
solution_sites = []
covered_demand = []
objective_value = 0
p = 20  # 
temperature = 30   #end temperature
max_iter = 3   #iteration limit
terminate_temp = 1         
temp_ratio = 0.15
sa_count = 0
remove_percent = 0.2 
fd_fullPayload = 5 * 5280 
min_dist = fd_fullPayload * 0.7
fd_empty = 10 * 5280
fd_delivery = 3.33 *5280 
rc = 0.001
rc_obj = 0.1
total_demand = 0.0 
            
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
solution_sites = random_fill_single(solution_sites)   
#print solution_sites

print "solution initialized"



while temperature > 0.5:
    
    current_solution = copy.copy(solution_sites)
    current_graph = delivery_network_mk2(current_solution, True, "currrent_graph")
    current_obj = cal_obj(current_solution)
    print "current Objective value: ", current_obj
    
    new_solution = copy.copy(current_solution)
    s_time = time.time()
    new_solution = network_removal_mk2_single(new_solution)
    e_time = time.time()
    print "removed", new_solution
    print "removal time 2: ", e_time - s_time
    print "fill start"
    s_time = time.time()
    new_solution = greedy_fill_single(new_solution)
    e_time = time.time()
    print "fill time: ", e_time - s_time
    #print new_solution
    print "spatial interchange start"
    s_time = time.time()
    new_solution = spatial_interchage_mk4_single(new_solution)
    e_time = time.time()
    print "interchange time: ", e_time - s_time
    new_graph = delivery_network_mk2(new_solution, True, "new_solution")
    new_obj = cal_obj(new_solution)
    print new_obj
    #print new_obj - current_obj
    
    if new_obj > current_obj:
        solution_sites = new_solution
        sa_count = 0 
        print "new solution accepted"
        #print "new objective value: ", new_obj
        #print "new solution: ", solution_sites
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
                print "new but bad objective: ", new_obj
                #print "new but bad solution: ", new_solution
            else:
                pass
                #print "bad solution reputed"
            
print "solution"                
print cal_obj_min(solution_sites)
print solution_sites

            
    
    
        
        
        
        
        
