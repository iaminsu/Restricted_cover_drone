#Convexpath approach sequential version 

#Including containing case & spatial filtering method 


import pysal,  shapefile, networkx, time, cPickle
from shapely.geometry import Point, Polygon, LineString, MultiPoint, MultiPolygon
from collections import defaultdict


path = "Users/insuhong/Dropbox/research/Distance restricted covering model/Locating recharging station/data/“

facilities = "sample_sites.shp"
demands = "sample_demand.shp"
obstacles = ""
fd_fullPayload = 2.5 
fd_empty = 5
fd_delivery = 1.665

demand_amounts_d = {}
#loading demand_amounts_d 

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
def createGraph( lineSet):
    resultingGraph = networkx.Graph()
    for line in lineSet:
        resultingGraph.add_edge(line[0], line[1], weight = LineString(list(line)).length)
    return resultingGraph
def createConvexhull( poly, endPoints = []):
    convexVerticex = []
    if poly.type == 'MultiPolygon':
        for i in poly.geoms:
            convexVerticex.extend(list(i.exterior.coords))
    else:
        convexVerticex.extend(list(poly.exterior.coords))
    convexVerticex.extend(endPoints)
    convex_hull = MultiPoint(convexVerticex).convex_hull

    return convex_hull
def splitBoundary(lineSet, poly):
    poly_vertices = list(poly.exterior.coords)


    for v in range(len(poly_vertices)):
        if (v + 1) == len(poly_vertices):
            a = ((poly_vertices[v][0], poly_vertices[v][1]), (poly_vertices[-1][0], poly_vertices[-1][1]))
            if a[0] != a[1]:
                if not lineSet.has_key(a):
                    if not lineSet.has_key((a[1],a[0])):
                        lineSet[a] = LineString(a)                      
        else:
            a = ((poly_vertices[v][0], poly_vertices[v][1]), (poly_vertices[v+1][0], poly_vertices[v+1][1]))
            if a[0] != a[1]:
                if not lineSet.has_key(a):
                    if not lineSet.has_key((a[1],a[0])):
                        lineSet[a] = LineString(a)                
                        
                        def splitPolygon(line, poly):
                                minX, minY, maxX, maxY = poly.bounds
                                polyBound = Polygon([(minX, minY), (minX, maxY), (maxX, maxY), (maxX, minY)])
                                splitLine = polyBound.intersection(line)        
                                lcoord = list(splitLine.coords)
                                Ax, Ay, Bx, By = 0, 0, 0, 0 
                                if lcoord[0][0] > lcoord[1][0]:
                                    Ax, Ay, Bx, By = lcoord[0][0], lcoord[0][1], lcoord[1][0], lcoord[1][1]
                                elif lcoord[0][0] == lcoord[1][0]:  #vertical line 
                                    if lcoord[0][1] > lcoord[1][1]:
                                        Ax, Ay, Bx, By = lcoord[0][0], lcoord[0][1], lcoord[1][0], lcoord[1][1]
                                    else:
                                        Ax, Ay, Bx, By = lcoord[1][0], lcoord[1][1], lcoord[0][0], lcoord[0][1]
                                elif lcoord[0][0] < lcoord[1][0]:
                                    Ax, Ay, Bx, By = lcoord[1][0], lcoord[1][1], lcoord[0][0], lcoord[0][1]
                                
                                if Ax == maxX:
                                    if Bx == minX:
                                        s1 = Polygon([(minX, maxY),(maxX, maxY), (Ax, Ay), (Bx, By)])
                                        s2 = Polygon([(Bx, By), (Ax, Ay), (maxX, minY), (minX, minY)])
                                    elif By == maxY:
                                        s1 = Polygon([(Bx, By), (maxX, maxY), (Ax, Ay)])
                                        s2 = Polygon([(minX, maxY), (Bx, By), (Ax, Ay), (maxX, minY), (minX, minY)])
                                    elif By == minY:
                                        s1 = Polygon([(minX, maxY), (maxX, maxY), (Ax, Ay), (Bx, By), (minX, minY)])
                                        s2 = Polygon([(Ax, Ay), (maxX, minY), (Bx, By)])
                                elif Ay == maxY:
                                    if By == minY:
                                        s1 = Polygon([(minX, maxY), (Ax, Ay), (Bx, By), (minX, minY)])
                                        s2 = Polygon([(Ax, Ay), (maxX, maxY), (maxX, minY), (Bx, By)])
                                    elif Bx == minX:
                                        s1 = Polygon([(minX, maxY), (Ax, Ay), (Bx, By)])
                                        s2 = Polygon([(Bx, By), (Ax, Ay), (maxX, maxY), (maxX, minY), (minX, minY)])
                                elif Ay == minY:
                                    if By == maxY:
                                        s1 = Polygon([(minX, maxY), (Bx, By), (Ax, Ay), (minX, minY)])
                                        s2 = Polygon([(Bx, By), (maxX, maxY), (maxX, minY), (Ax, Ay)])
                                    elif Bx == minX:
                                        s1 = Polygon([(Bx, By), (Ax, Ay), (minX, minY)])
                                        s2 = Polygon([(Bx, By), (minX, maxY), (maxX, maxY), (maxX, minY), (Ax, Ay)])
                                return s1, s2
                                
                                



def createConvexPath(pair, case):
    
    
    print pair[1]
    odPointsList = ((pair[0][0].x, pair[0][0].y), (pair[0][1].x, pair[0][1].y))
    st_line = LineString(odPointsList)
    labeledObstaclePoly = []
    totalConvexPathList = {}
    if case == 'ff':
        if st_line.length > fd_fullPayload * 5280:
            return 0
    elif case == "fd":
        if st_line.length > fd_delivery * 5280:
            return 0
    
        
    dealtArcList = {}
    totalConvexPathList[odPointsList] = LineString(odPointsList)
    LineString
    terminate = 0
    idx_loop1 = 0
    time_loop1 = 0
    time_contain2 = 0
    time_crossingDict = 0
    time_convexLoop = 0 
    time_impedingArcs = 0
    time_spatialFiltering = 0
    time_loop1_crossingDict = 0
    time_buildConvexHulls = 0
    no_obs = False
    while terminate == 0:
        t1s = time.time()
        idx_loop1 += 1
        
        t6s = time.time()
        #w = shapefile.Writer(shapefile.POLYLINE)
        #w.field('nem')
        #for line in totalConvexPathList:
            #w.line(parts=[[ list(x) for x in line ]])
            #w.record('ff')
        #w.save(path + "graph_" + str(idx_loop1) + version_name)

        totalGrpah = createGraph(totalConvexPathList.keys())
        spatial_filter_n = networkx.dijkstra_path(totalGrpah, odPointsList[0], odPointsList[1])            
        spatial_filter = []
        for i in xrange(len(spatial_filter_n)-1):
            spatial_filter.append([spatial_filter_n[i], spatial_filter_n[i+1]])

        #w = shapefile.Writer(shapefile.POLYLINE)
        #w.field('nem')
        #for line in spatial_filter:
            #w.line(parts=[[ list(x) for x in line ]])
            #w.record('ff')
        #w.save(self.path + "spatial Filter_" + str(idx_loop1) + self.version_name)
        
        #sp_length = 0
        #for j in spatial_filter:
            #sp_length += LineString(j).length        
        #sp_l_set.append(sp_length)
        
        crossingDict = defaultdict(list)
        
        for line in spatial_filter:
            Line = LineString(line)
            for obs in obstaclesPolygons:
                if Line.crosses(obs):
                    if obs not in labeledObstaclePoly:
                        labeledObstaclePoly.append(obs)
                
                    crossingDict[tuple(line)].append(obs)
        
        t6e = time.time()
        time_spatialFiltering += t6e - t6s 
        
        if len(crossingDict.keys()) == 0:
            terminate = 1
            no_obs = True
            continue
        else:
            t7s = time.time()
            for tLine in crossingDict.keys():
                #cLine = list(tLine)
                if dealtArcList.has_key(tLine):
                    try:
                        del totalConvexPathList[tLine]
                    except:
                        del totalConvexPathList[(tLine[1], tLine[0])]
                    continue
                else:
                    dealtArcList[tLine] = LineString(list(tLine))
                    try:
                        del totalConvexPathList[tLine]
                    except:
                        del totalConvexPathList[(tLine[1], tLine[0])]
                    containingObs = []
                    for obs in crossingDict[tLine]:
                        
                        convexHull = createConvexhull(obs, tLine)
                        splitBoundary(totalConvexPathList, convexHull)
                        convexHull = createConvexhull(obs, odPointsList)
                        splitBoundary(totalConvexPathList, convexHull)
                        convexHull2 = createConvexhull(obs)
                        if convexHull2.contains(Point(tLine[0])):
                            containingObs.append(obs)
                        elif convexHull2.contains(Point(tLine[1])):
                            containingObs.append(obs)
                    if len(containingObs) != 0:   #SPLIT
                        subConvexPathList = {}
                        vi_obs = MultiPolygon([x for x in containingObs])
                        containedLineCoords = list(tLine)
                        fromX = containedLineCoords[0][0]
                        fromY = containedLineCoords[0][1]
                        toX = containedLineCoords[1][0]
                        toY = containedLineCoords[1][1]
                        fxA = (fromY - toY) / (fromX - toX)
                        fxB = fromY - (fxA * fromX)
                        minX = vi_obs.bounds[0]
                        maxX = vi_obs.bounds[2]
                        split_line = LineString([(min(minX, fromX, toX), fxA * min(minX, fromX, toX) + fxB), (max(maxX, fromX, toX), fxA * max(maxX, fromX, toX) + fxB)])
                        
                        for obs in containingObs:
                            s1, s2 = splitPolygon(split_line, obs)
                            dividedObsPoly = []
                            #to deal with multipolygon
                            a = s1.intersection(obs)
                            b = s2.intersection(obs)
                            if a.type == "Polygon":
                                dividedObsPoly.append(a)
                            else:
                                for o in a.geoms:
                                    if o.type == "Polygon":
                                        dividedObsPoly.append(o)
                            if b.type == "Polygon":
                                dividedObsPoly.append(b)
                            else:
                                for o2 in b.geoms:
                                    if o2.type == "Polygon":
                                        dividedObsPoly.append(o2)
                            
                            for obs2 in dividedObsPoly:
                                for pt in tLine:
                                    convexHull = createConvexhull(obs2, [pt])
                                    splitBoundary(subConvexPathList, convexHull)
                        subVertices = []
                        for line in subConvexPathList:
                            subVertices.extend(line)
                        subVertices = list(set(subVertices))
                        containingObsVertices = []
                        for obs in containingObs:
                            containingObsVertices.extend(list(obs.exterior.coords))
                        subVertices = [x for x in subVertices if x in containingObsVertices]
                        deleteList = []
                        for line in subConvexPathList:
                            chk_cross = 0
                            for obs in containingObs:
                                if subConvexPathList[line].crosses(obs):
                                    chk_cross = 1
                            if chk_cross == 1:
                                deleteList.append(line)
                        for line in deleteList:
                            del subConvexPathList[line]
                            #subConvexPathList.remove(line)
                        pairList = []
                        for i in range(len(subVertices)):
                            for j in range(i+1, len(subVertices)):
                                pairList.append((subVertices[i], subVertices[j]))
                        for i in pairList:
                            Line = LineString(i)
                            chk_cross = 0
                            for obs in containingObs:
                                if Line.crosses(obs):
                                    chk_cross = 1
                                elif Line.within(obs):
                                    chk_cross = 1
                            if chk_cross == 0:
                                subConvexPathList[i] = Line
                                #subConvexPathList.append(i)
                        buffer_st_line = split_line.buffer(0.1)
                        deleteList = []
                        for line in subConvexPathList:
                            if buffer_st_line.contains(subConvexPathList[line]):
                                deleteList.append(line)
                        for line in deleteList:
                            if subConvexPathList.has_key(line):
                                del subConvexPathList[line]
                        #subConvexPathList = [x for x in subConvexPathList if x not in deleteList]
                        for line in subConvexPathList:
                            if not totalConvexPathList.has_key(line):
                                if not totalConvexPathList.has_key((line[1],line[0])):
                                    totalConvexPathList[line] = subConvexPathList[line]                                #if line not in totalConvexPathList:
                                #if [line[1], line[0]] not in totalConvexPathList:
                                    #totalConvexPathList.append(line)

            #w = shapefile.Writer(shapefile.POLYLINE)
            #w.field('nem')
            #for line in totalConvexPathList:
                #w.line(parts=[[ list(x) for x in line ]])
                #w.record('ff')
            #w.save(self.path + "graph2_" + str(idx_loop1) + self.version_name) 
            t7e = time.time()
            time_loop1_crossingDict += t7e - t7s
            #new lines            
            labeled_multyPoly = MultiPolygon([x for x in labeledObstaclePoly])
            convexHull = createConvexhull(labeled_multyPoly, odPointsList)
            splitBoundary(totalConvexPathList, convexHull)
            #new lines end             
                              
            #impededPathList 
            t5s = time.time()
            impededPathList = {}
            for line in totalConvexPathList:
                for obs in labeledObstaclePoly:
                    if totalConvexPathList[line].crosses(obs):
                        impededPathList[line] = totalConvexPathList[line]
                        break
            t5e = time.time()
            time_impedingArcs += t5e - t5s
            for line in impededPathList:
                del totalConvexPathList[line]
           
            terminate2 = 0
            idx_loop2 = 0
            t1e = time.time()
            time_loop1 += t1e - t1s                   
            while terminate2 == 0:
                idx_loop2 += 1

                deleteList = []
                crossingDict = defaultdict(list)

                for line in dealtArcList:
                    if impededPathList.has_key(line):
                        del impededPathList[line]
                    elif impededPathList.has_key((line[1], line[0])):
                        del impededPathList[(line[1],line[0])]
                
                t3s = time.time()
                #pr.enable()
                for line in impededPathList:
                    for obs in labeledObstaclePoly:
                        if impededPathList[line].crosses(obs):
                            crossingDict[line].append(obs)
                
                t3e = time.time()
                time_crossingDict += t3e - t3s
                #at this point, impededArcList should be emptied, as it only contains crossing arcs, and all of them 
                #should be replaced by convex hulls. 
                for line in crossingDict:
                    del impededPathList[line]
                for line in impededPathList:
                    if not totalConvexPathList.has_key(line):
                        totalConvexPathList[line] = impededPathList[line]
                impededPathList = {}

                if len(crossingDict.keys()) == 0:
                    terminate2 = 1
                    continue
                else:
                    #w = shapefile.Writer(shapefile.POLYLINE)
                    #w.field('nem')
                    #for line in crossingDict:
                        #w.line(parts=[[ list(x) for x in line ]])
                        #w.record('ff')
                    #w.save(self.path + "crossingDict_" + str(idx_loop1) + "_"+ str(idx_loop2) +"_"+ self.version_name)                        
                    t4s = time.time()
                    
                    for tLine in crossingDict.keys():
                        dealtArcList[tLine] = crossingDict[tLine]                
                        containingObs = []
                        for obs in crossingDict[tLine]:
                            chk_contain = 0
                            convexHull2 = createConvexhull(obs)
                            if convexHull2.contains(Point(tLine[0])):
                                containingObs.append(obs)
                                chk_contain = 1
                            elif convexHull2.contains(Point(tLine[1])):
                                containingObs.append(obs)
                                chk_contain = 1
                            if chk_contain == 0:
                                t10s = time.time()
                                convexHull = createConvexhull(obs, tLine)
                                splitBoundary(impededPathList, convexHull)
                                t10e = time.time()
                                time_buildConvexHulls += t10e - t10s

                        if len(containingObs) != 0:  #SPLIT
                            #print "SPLIT"
                            t2s = time.time()
                            subConvexPathList = {}
                            vi_obs = MultiPolygon([x for x in containingObs])
                            containedLineCoords = tLine
                            fromX = containedLineCoords[0][0]
                            fromY = containedLineCoords[0][1]
                            toX = containedLineCoords[1][0]
                            toY = containedLineCoords[1][1]
                            fxA = (fromY - toY) / (fromX - toX)
                            fxB = fromY - (fxA * fromX)
                            minX = vi_obs.bounds[0]
                            maxX = vi_obs.bounds[2]
                            split_line = LineString([(min(minX, fromX, toX), fxA * min(minX, fromX, toX) + fxB), (max(maxX, fromX, toX), fxA * max(maxX, fromX, toX) + fxB)])
                            
                            for obs in containingObs:
                                s1, s2 = splitPolygon(split_line, obs)
                                dividedObsPoly = []
                                #to deal with multipolygon
                                a = s1.intersection(obs)
                                b = s2.intersection(obs)
                                if a.type == "Polygon":
                                    dividedObsPoly.append(a)
                                else:
                                    for o in a.geoms:
                                        if o.type == "Polygon":
                                            dividedObsPoly.append(o)
                                if b.type == "Polygon":
                                    dividedObsPoly.append(b)
                                else:
                                    for o2 in b.geoms:
                                        if o2.type == "Polygon":
                                            dividedObsPoly.append(o2)
                                
                                for obs2 in dividedObsPoly:
                                    for pt in tLine:
                                        convexHull = createConvexhull(obs2, [pt])
                                        splitBoundary(subConvexPathList, convexHull)
                            subVertices = []
                            for line in subConvexPathList:
                                subVertices.extend(line)
                            subVertices = list(set(subVertices))
                            containingObsVertices = []
                            for obs in containingObs:
                                containingObsVertices.extend(list(obs.exterior.coords))
                            subVertices = [x for x in subVertices if x in containingObsVertices]
                            deleteList = []
                            for line in subConvexPathList:
                                chk_cross = 0
                                for obs in containingObs:
                                    if subConvexPathList[line].crosses(obs):
                                        chk_cross = 1
                                if chk_cross == 1:
                                    deleteList.append(line)
                            for line in deleteList:
                                del subConvexPathList[line]
                                
                            pairList = []
                            for i in range(len(subVertices)):
                                for j in range(i+1, len(subVertices)):
                                    pairList.append((subVertices[i], subVertices[j]))
                            
                            for i in pairList:
                                Line = LineString(list(i))
                                chk_cross = 0
                                for obs in containingObs:
                                    if Line.crosses(obs):
                                        chk_cross = 1
                                    elif Line.within(obs):
                                        chk_cross = 1
                                if chk_cross == 0:
                                    subConvexPathList[i] = Line
                                  
                            buffer_st_line = split_line.buffer(0.1)
                            deleteList = []
                            for line in subConvexPathList:
                                if buffer_st_line.contains(subConvexPathList[line]):
                                    deleteList.append(line)
                            for line in deleteList:
                                del subConvexPathList[line]
                            for line in subConvexPathList:
                                if not impededPathList.has_key(line):
                                    if not impededPathList.has_key((line[1], line[0])):
                                        impededPathList[line] = subConvexPathList[line]
                                
                            t2e = time.time()
                            time_contain2 += t2e - t2s
                    #pr.disable()
                    for line in dealtArcList:
                        if impededPathList.has_key(line):
                            del impededPathList[line]
                    #impededPathList = [x for x in impededPathList if x not in dealtArcList]
                    t4e = time.time()
                    time_convexLoop += t4e - t4s
                    #end of else
                #w = shapefile.Writer(shapefile.POLYLINE)
                #w.field('nem')
                #for line in impededPathList:
                    #w.line(parts=[[ list(x) for x in line ]])
                    #w.record('ff')
                #w.save(path + "After_graph_" + str(idx_loop1) + "_"+ str(idx_loop2) +"_"+ version_name)
                #end of while2
            for line in impededPathList:
                if not totalConvexPathList.has_key(line):
                    totalConvexPathList[line] = impededPathList[line]
            
            #totalConvexPathList.extend(impededPathList)
    #no obstruction 
    if no_obs == True:
        return 1
    
    totalGraph = createGraph(totalConvexPathList.keys())
    esp_n = networkx.dijkstra_path(totalGraph, odPointsList[0], odPointsList[1])
    esp = []
    for i in range(len(esp_n)-1):
        esp.append([esp_n[i], esp_n[i+1]])
    
    #w = shapefile.Writer(shapefile.POLYLINE)
    #w.field('nem')
    #no_edges = 0
    #for line in totalConvexPathList.keys():
        #no_edges += 1
        #w.line(parts=[[ list(x) for x in line ]])
        #w.record('ff')
    #w.save(path + "totalpath" + version_name + "%d" % pair[1] )              
    w = shapefile.Writer(shapefile.POLYLINE)
    w.field('nem')
    for line in esp:
        w.line(parts=[[ list(x) for x in line ]])
        w.record('ff')
    w.save(path + "ESP_" + version_name + "%d" % pair[1])
    
    targetPysal = pysal.IOHandlers.pyShpIO.shp_file(path + "ESP_" + version_name + "%d" % pair[1])
    targetShp = generateGeometry(targetPysal)
    total_length = 0
    for line in targetShp:
        total_length += line.length
    if case == "ff":
        if total_length <= fd_fullPayload:
            return 1
        else:
            return 0
    else case == 'fd':
        if total_length <= fd_delivery:
            return 1
        else:
            return 0 
    








def createConvexPath_FD(pair):
    #For F_D pair only 
    #return demand_id if ESP distance to target demand is less than fd_delivery
    print pair[1]
    odPointsList = ((pair[0][0].x, pair[0][0].y), (pair[0][1].x, pair[0][1].y))
    st_line = LineString(odPointsList)
    labeledObstaclePoly = []
    totalConvexPathList = {}
    if st_line.length > fd_delivery * 5280:
        return 0
        
    dealtArcList = {}
    totalConvexPathList[odPointsList] = LineString(odPointsList)
    LineString
    terminate = 0
    idx_loop1 = 0
    time_loop1 = 0
    time_contain2 = 0
    time_crossingDict = 0
    time_convexLoop = 0 
    time_impedingArcs = 0
    time_spatialFiltering = 0
    time_loop1_crossingDict = 0
    time_buildConvexHulls = 0
    no_obs = False
    while terminate == 0:
        t1s = time.time()
        idx_loop1 += 1
        
        t6s = time.time()
        #w = shapefile.Writer(shapefile.POLYLINE)
        #w.field('nem')
        #for line in totalConvexPathList:
            #w.line(parts=[[ list(x) for x in line ]])
            #w.record('ff')
        #w.save(path + "graph_" + str(idx_loop1) + version_name)

        totalGrpah = createGraph(totalConvexPathList.keys())
        spatial_filter_n = networkx.dijkstra_path(totalGrpah, odPointsList[0], odPointsList[1])            
        spatial_filter = []
        for i in xrange(len(spatial_filter_n)-1):
            spatial_filter.append([spatial_filter_n[i], spatial_filter_n[i+1]])

        #w = shapefile.Writer(shapefile.POLYLINE)
        #w.field('nem')
        #for line in spatial_filter:
            #w.line(parts=[[ list(x) for x in line ]])
            #w.record('ff')
        #w.save(self.path + "spatial Filter_" + str(idx_loop1) + self.version_name)
        
        #sp_length = 0
        #for j in spatial_filter:
            #sp_length += LineString(j).length        
        #sp_l_set.append(sp_length)
        
        crossingDict = defaultdict(list)
        
        for line in spatial_filter:
            Line = LineString(line)
            for obs in obstaclesPolygons:
                if Line.crosses(obs):
                    if obs not in labeledObstaclePoly:
                        labeledObstaclePoly.append(obs)
                
                    crossingDict[tuple(line)].append(obs)
        
        t6e = time.time()
        time_spatialFiltering += t6e - t6s 
        
        if len(crossingDict.keys()) == 0:
            terminate = 1
            no_obs = True
            continue
        else:
            t7s = time.time()
            for tLine in crossingDict.keys():
                #cLine = list(tLine)
                if dealtArcList.has_key(tLine):
                    try:
                        del totalConvexPathList[tLine]
                    except:
                        del totalConvexPathList[(tLine[1], tLine[0])]
                    continue
                else:
                    dealtArcList[tLine] = LineString(list(tLine))
                    try:
                        del totalConvexPathList[tLine]
                    except:
                        del totalConvexPathList[(tLine[1], tLine[0])]
                    containingObs = []
                    for obs in crossingDict[tLine]:
                        
                        convexHull = createConvexhull(obs, tLine)
                        splitBoundary(totalConvexPathList, convexHull)
                        convexHull = createConvexhull(obs, odPointsList)
                        splitBoundary(totalConvexPathList, convexHull)
                        convexHull2 = createConvexhull(obs)
                        if convexHull2.contains(Point(tLine[0])):
                            containingObs.append(obs)
                        elif convexHull2.contains(Point(tLine[1])):
                            containingObs.append(obs)
                    if len(containingObs) != 0:   #SPLIT
                        subConvexPathList = {}
                        vi_obs = MultiPolygon([x for x in containingObs])
                        containedLineCoords = list(tLine)
                        fromX = containedLineCoords[0][0]
                        fromY = containedLineCoords[0][1]
                        toX = containedLineCoords[1][0]
                        toY = containedLineCoords[1][1]
                        fxA = (fromY - toY) / (fromX - toX)
                        fxB = fromY - (fxA * fromX)
                        minX = vi_obs.bounds[0]
                        maxX = vi_obs.bounds[2]
                        split_line = LineString([(min(minX, fromX, toX), fxA * min(minX, fromX, toX) + fxB), (max(maxX, fromX, toX), fxA * max(maxX, fromX, toX) + fxB)])
                        
                        for obs in containingObs:
                            s1, s2 = splitPolygon(split_line, obs)
                            dividedObsPoly = []
                            #to deal with multipolygon
                            a = s1.intersection(obs)
                            b = s2.intersection(obs)
                            if a.type == "Polygon":
                                dividedObsPoly.append(a)
                            else:
                                for o in a.geoms:
                                    if o.type == "Polygon":
                                        dividedObsPoly.append(o)
                            if b.type == "Polygon":
                                dividedObsPoly.append(b)
                            else:
                                for o2 in b.geoms:
                                    if o2.type == "Polygon":
                                        dividedObsPoly.append(o2)
                            
                            for obs2 in dividedObsPoly:
                                for pt in tLine:
                                    convexHull = createConvexhull(obs2, [pt])
                                    splitBoundary(subConvexPathList, convexHull)
                        subVertices = []
                        for line in subConvexPathList:
                            subVertices.extend(line)
                        subVertices = list(set(subVertices))
                        containingObsVertices = []
                        for obs in containingObs:
                            containingObsVertices.extend(list(obs.exterior.coords))
                        subVertices = [x for x in subVertices if x in containingObsVertices]
                        deleteList = []
                        for line in subConvexPathList:
                            chk_cross = 0
                            for obs in containingObs:
                                if subConvexPathList[line].crosses(obs):
                                    chk_cross = 1
                            if chk_cross == 1:
                                deleteList.append(line)
                        for line in deleteList:
                            del subConvexPathList[line]
                            #subConvexPathList.remove(line)
                        pairList = []
                        for i in range(len(subVertices)):
                            for j in range(i+1, len(subVertices)):
                                pairList.append((subVertices[i], subVertices[j]))
                        for i in pairList:
                            Line = LineString(i)
                            chk_cross = 0
                            for obs in containingObs:
                                if Line.crosses(obs):
                                    chk_cross = 1
                                elif Line.within(obs):
                                    chk_cross = 1
                            if chk_cross == 0:
                                subConvexPathList[i] = Line
                                #subConvexPathList.append(i)
                        buffer_st_line = split_line.buffer(0.1)
                        deleteList = []
                        for line in subConvexPathList:
                            if buffer_st_line.contains(subConvexPathList[line]):
                                deleteList.append(line)
                        for line in deleteList:
                            if subConvexPathList.has_key(line):
                                del subConvexPathList[line]
                        #subConvexPathList = [x for x in subConvexPathList if x not in deleteList]
                        for line in subConvexPathList:
                            if not totalConvexPathList.has_key(line):
                                if not totalConvexPathList.has_key((line[1],line[0])):
                                    totalConvexPathList[line] = subConvexPathList[line]                                #if line not in totalConvexPathList:
                                #if [line[1], line[0]] not in totalConvexPathList:
                                    #totalConvexPathList.append(line)

            t7e = time.time()
            time_loop1_crossingDict += t7e - t7s
            #new lines            
            labeled_multyPoly = MultiPolygon([x for x in labeledObstaclePoly])
            convexHull = createConvexhull(labeled_multyPoly, odPointsList)
            splitBoundary(totalConvexPathList, convexHull)
            #new lines end             
                              
            #impededPathList 
            t5s = time.time()
            impededPathList = {}
            for line in totalConvexPathList:
                for obs in labeledObstaclePoly:
                    if totalConvexPathList[line].crosses(obs):
                        impededPathList[line] = totalConvexPathList[line]
                        break
            t5e = time.time()
            time_impedingArcs += t5e - t5s
            for line in impededPathList:
                del totalConvexPathList[line]
           
            terminate2 = 0
            idx_loop2 = 0
            t1e = time.time()
            time_loop1 += t1e - t1s                   
            while terminate2 == 0:
                idx_loop2 += 1

                deleteList = []
                crossingDict = defaultdict(list)

                for line in dealtArcList:
                    if impededPathList.has_key(line):
                        del impededPathList[line]
                    elif impededPathList.has_key((line[1], line[0])):
                        del impededPathList[(line[1],line[0])]
                
                t3s = time.time()
                #pr.enable()
                for line in impededPathList:
                    for obs in labeledObstaclePoly:
                        if impededPathList[line].crosses(obs):
                            crossingDict[line].append(obs)
                
                t3e = time.time()
                time_crossingDict += t3e - t3s
                #at this point, impededArcList should be emptied, as it only contains crossing arcs, and all of them 
                #should be replaced by convex hulls. 
                for line in crossingDict:
                    del impededPathList[line]
                for line in impededPathList:
                    if not totalConvexPathList.has_key(line):
                        totalConvexPathList[line] = impededPathList[line]
                impededPathList = {}

                if len(crossingDict.keys()) == 0:
                    terminate2 = 1
                    continue
                else:
                    
                    t4s = time.time()
                    
                    for tLine in crossingDict.keys():
                        dealtArcList[tLine] = crossingDict[tLine]                
                        containingObs = []
                        for obs in crossingDict[tLine]:
                            chk_contain = 0
                            convexHull2 = createConvexhull(obs)
                            if convexHull2.contains(Point(tLine[0])):
                                containingObs.append(obs)
                                chk_contain = 1
                            elif convexHull2.contains(Point(tLine[1])):
                                containingObs.append(obs)
                                chk_contain = 1
                            if chk_contain == 0:
                                t10s = time.time()
                                convexHull = createConvexhull(obs, tLine)
                                splitBoundary(impededPathList, convexHull)
                                t10e = time.time()
                                time_buildConvexHulls += t10e - t10s

                        if len(containingObs) != 0:  #SPLIT
                            #print "SPLIT"
                            t2s = time.time()
                            subConvexPathList = {}
                            vi_obs = MultiPolygon([x for x in containingObs])
                            containedLineCoords = tLine
                            fromX = containedLineCoords[0][0]
                            fromY = containedLineCoords[0][1]
                            toX = containedLineCoords[1][0]
                            toY = containedLineCoords[1][1]
                            fxA = (fromY - toY) / (fromX - toX)
                            fxB = fromY - (fxA * fromX)
                            minX = vi_obs.bounds[0]
                            maxX = vi_obs.bounds[2]
                            split_line = LineString([(min(minX, fromX, toX), fxA * min(minX, fromX, toX) + fxB), (max(maxX, fromX, toX), fxA * max(maxX, fromX, toX) + fxB)])
                            
                            for obs in containingObs:
                                s1, s2 = splitPolygon(split_line, obs)
                                dividedObsPoly = []
                                #to deal with multipolygon
                                a = s1.intersection(obs)
                                b = s2.intersection(obs)
                                if a.type == "Polygon":
                                    dividedObsPoly.append(a)
                                else:
                                    for o in a.geoms:
                                        if o.type == "Polygon":
                                            dividedObsPoly.append(o)
                                if b.type == "Polygon":
                                    dividedObsPoly.append(b)
                                else:
                                    for o2 in b.geoms:
                                        if o2.type == "Polygon":
                                            dividedObsPoly.append(o2)
                                
                                for obs2 in dividedObsPoly:
                                    for pt in tLine:
                                        convexHull = createConvexhull(obs2, [pt])
                                        splitBoundary(subConvexPathList, convexHull)
                            subVertices = []
                            for line in subConvexPathList:
                                subVertices.extend(line)
                            subVertices = list(set(subVertices))
                            containingObsVertices = []
                            for obs in containingObs:
                                containingObsVertices.extend(list(obs.exterior.coords))
                            subVertices = [x for x in subVertices if x in containingObsVertices]
                            deleteList = []
                            for line in subConvexPathList:
                                chk_cross = 0
                                for obs in containingObs:
                                    if subConvexPathList[line].crosses(obs):
                                        chk_cross = 1
                                if chk_cross == 1:
                                    deleteList.append(line)
                            for line in deleteList:
                                del subConvexPathList[line]
                                
                            pairList = []
                            for i in range(len(subVertices)):
                                for j in range(i+1, len(subVertices)):
                                    pairList.append((subVertices[i], subVertices[j]))
                            
                            for i in pairList:
                                Line = LineString(list(i))
                                chk_cross = 0
                                for obs in containingObs:
                                    if Line.crosses(obs):
                                        chk_cross = 1
                                    elif Line.within(obs):
                                        chk_cross = 1
                                if chk_cross == 0:
                                    subConvexPathList[i] = Line
                                  
                            buffer_st_line = split_line.buffer(0.1)
                            deleteList = []
                            for line in subConvexPathList:
                                if buffer_st_line.contains(subConvexPathList[line]):
                                    deleteList.append(line)
                            for line in deleteList:
                                del subConvexPathList[line]
                            for line in subConvexPathList:
                                if not impededPathList.has_key(line):
                                    if not impededPathList.has_key((line[1], line[0])):
                                        impededPathList[line] = subConvexPathList[line]
                                
                            t2e = time.time()
                            time_contain2 += t2e - t2s
                    #pr.disable()
                    for line in dealtArcList:
                        if impededPathList.has_key(line):
                            del impededPathList[line]
                    #impededPathList = [x for x in impededPathList if x not in dealtArcList]
                    t4e = time.time()
                    time_convexLoop += t4e - t4s
                    #end of else
                #end of while2
            for line in impededPathList:
                if not totalConvexPathList.has_key(line):
                    totalConvexPathList[line] = impededPathList[line]
            
            #totalConvexPathList.extend(impededPathList)
    #no obstruction 
    if no_obs == True:
        return 1
    
    totalGraph = createGraph(totalConvexPathList.keys())
    esp_n = networkx.dijkstra_path(totalGraph, odPointsList[0], odPointsList[1])
    esp = []
    for i in range(len(esp_n)-1):
        esp.append([esp_n[i], esp_n[i+1]])
    
    #w = shapefile.Writer(shapefile.POLYLINE)
    #w.field('nem')
    #no_edges = 0
    #for line in totalConvexPathList.keys():
        #no_edges += 1
        #w.line(parts=[[ list(x) for x in line ]])
        #w.record('ff')
    #w.save(path + "totalpath" + version_name + "%d" % pair[1] )              
    w = shapefile.Writer(shapefile.POLYLINE)
    w.field('nem')
    for line in esp:
        w.line(parts=[[ list(x) for x in line ]])
        w.record('ff')
    w.save(path + "ESP_" + version_name + "%d" % pair[1])
    
    targetPysal = pysal.IOHandlers.pyShpIO.shp_file(path + "ESP_" + version_name + "%d" % pair[1])
    targetShp = generateGeometry(targetPysal)
    total_length = 0
    for line in targetShp:
        total_length += line.length
    if total_length <= fd_delivery:
        return 1
    else:
        return 0
    
    










ori_shp = pysal.IOHandlers.pyShpIO.shp_file(path + facilities)
des_shp = pysal.IOHandlers.pyShpIO.shp_file(path + demands)
obs_shp = pysal.IOHandlers.pyShpIO.shp_file(path + obstacles)
facilPoints = generateGeometry(ori_shp)
demandPoints = generateGeometry(des_shp)
obstaclesPolygons = generateGeometry(obs_shp)

version_name = "_Convexpath_Sequential_"
pair_number = 0
F_D_Pairs = []
for i in facilPoints:
    for j in demandPoints:
        F_D_Pairs.append([(i,j), pair_number])
        pair_number += 1
F_F_pairs = []
for i in facilPoints:
    for j in facilPoints:
        if not i == j:
            F_F_pairs.append((i,j))
            
f = open(path + "FF_Dict" + version_name + facilities + "_" + demands +"_"+obstacles+".txt", "w")
f2 = open(path + "FD_Dict" + version_name + facilities + "_" + demands +"_"+obstacles+".txt", "w")
facil_dict = defaultdict(list)
facil_demand_dict = defaultdict(list)
for pair in F_F_pairs:
    result = createConvexPath(pair, "ff")
    if result == 1:
        facil_dict[pair[0]].append(pair[1])

print "facil dict completed"

for pair in F_D_Pairs:
    result = createConvexPath(pair, "fd")
    if result == 1:
        facil_demand_dict[pair[0]].append([pair[1], demand_amounts_d[pair[1]]])
        
print "demand dict completed"
            
cPickle.dump(facil_dict, f)
cPickle.dump(facil_demand_dict, f2)