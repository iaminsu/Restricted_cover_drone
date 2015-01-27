#Convexpath approach sequential version for rechargeable drone path

#Including containing case & spatial filtering method 
#B version: using dictionary. Much faster

#Mk1: ROI 
# - generate ESP
# - mark for delivery network (Gd) and for return network (Gr)
import pysal,  shapefile, networkx, time
from shapely.geometry import Point, Polygon, LineString, MultiPoint, MultiPolygon
from collections import defaultdict
#import cProfile, pstats, StringIO

class Convexpath():
    def __init__(self, path, origin, destination, obstacles):
        ori_shp = pysal.IOHandlers.pyShpIO.shp_file(path + origin)
        des_shp = pysal.IOHandlers.pyShpIO.shp_file(path + destination)
        obs_shp = pysal.IOHandlers.pyShpIO.shp_file(path + obstacles)
        self.originPoints = self.generateGeometry(ori_shp)
        self.destinationPoints = self.generateGeometry(des_shp)
        self.obstaclesPolygons = self.generateGeometry(obs_shp)
        self.path = path
        self.version_name = "_Convexpath_Sequential_"
        pair_number = 0
        

        ODPairs = []
        if origin == destination:
            for i in range(len(self.originPoints)):
                for j in range(i+1, len(self.destinationPoints)):
                    ODPairs.append([(self.originPoints[i],self.destinationPoints[j]), pair_number])
                    pair_number += 1       
        else:
            for i in self.originPoints:
                for j in self.destinationPoints:
                    ODPairs.append([(i,j), pair_number])
                    pair_number += 1
        print "pairs"
        
        f = open(path + "Results" + self.version_name + origin + "_" + destination +"_"+obstacles+".txt", "w")
        for pair in ODPairs:
            t_start = time.time()
            result = self.createConvexPath(pair)
            t_end = time.time()
            print str(t_end - t_start)
            f.write(result + " " + str(t_end - t_start) + "\n" )
        f.close()
        
    def generateGeometry(self, in_shp):
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
    def createGraph(self, lineSet):
        resultingGraph = networkx.Graph()
        for line in lineSet:
            resultingGraph.add_edge(line[0], line[1], weight = LineString(list(line)).length)
        return resultingGraph
    def createConvexhull(self, poly, endPoints = []):
        convexVerticex = []
        if poly.type == 'MultiPolygon':
            for i in poly.geoms:
                convexVerticex.extend(list(i.exterior.coords))
        else:
            convexVerticex.extend(list(poly.exterior.coords))
        convexVerticex.extend(endPoints)
        convex_hull = MultiPoint(convexVerticex).convex_hull

        return convex_hull
    def splitBoundary(self, lineSet, poly):
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
                            


            
    def splitPolygon(self, line, poly):
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
        
        
        
                            
    def createConvexPath(self, pair):
        #pr = cProfile.Profile()
        #pr2 = cProfile.Profile()
        
        print pair[1]
        odPointsList = ((pair[0][0].x, pair[0][0].y), (pair[0][1].x, pair[0][1].y))
        #st_line = LineString(odPointsList)
        labeledObstaclePoly = []
        totalConvexPathList = {}
        
        dealtArcList = {}
        totalConvexPathList[odPointsList] = LineString(odPointsList)
        
        terminate = 0
        idx_loop1 = 0
        #sp_l_set = []
        time_loop1 = 0
        time_contain2 = 0
        time_crossingDict = 0
        time_convexLoop = 0 
        time_impedingArcs = 0
        time_spatialFiltering = 0
        time_loop1_crossingDict = 0
        time_buildConvexHulls = 0
        while terminate == 0:
            t1s = time.time()
            idx_loop1 += 1
            
            t6s = time.time()
            #w = shapefile.Writer(shapefile.POLYLINE)
            #w.field('nem')
            #for line in totalConvexPathList:
                #w.line(parts=[[ list(x) for x in line ]])
                #w.record('ff')
            #w.save(self.path + "graph_" + str(idx_loop1) + self.version_name)

            totalGrpah = self.createGraph(totalConvexPathList.keys())
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
                for obs in self.obstaclesPolygons:
                    if Line.crosses(obs):
                        if obs not in labeledObstaclePoly:
                            labeledObstaclePoly.append(obs)
                    
                        crossingDict[tuple(line)].append(obs)
            
            t6e = time.time()
            time_spatialFiltering += t6e - t6s 
            
            if len(crossingDict.keys()) == 0:
                terminate = 1
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
                            
                            convexHull = self.createConvexhull(obs, tLine)
                            self.splitBoundary(totalConvexPathList, convexHull)
                            
                            
                            convexHull = self.createConvexhull(obs, odPointsList)
                            self.splitBoundary(totalConvexPathList, convexHull)
                            convexHull2 = self.createConvexhull(obs)
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
                                s1, s2 = self.splitPolygon(split_line, obs)
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
                                        convexHull = self.createConvexhull(obs2, [pt])
                                        self.splitBoundary(subConvexPathList, convexHull)
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
                convexHull = self.createConvexhull(labeled_multyPoly, odPointsList)
                self.splitBoundary(totalConvexPathList, convexHull)
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
                #w = shapefile.Writer(shapefile.POLYGON)
                #w.field('net')
                #for obs in labeledObstaclePoly:
                    #w.poly(parts=[[list(x) for x in list(obs.exterior.coords)]])
                    #w.record('ff')
                #w.save(self.path + "obs"+ str(idx_loop1) + "_" + self.version_name)                  
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
                                convexHull2 = self.createConvexhull(obs)
                                if convexHull2.contains(Point(tLine[0])):
                                    containingObs.append(obs)
                                    chk_contain = 1
                                elif convexHull2.contains(Point(tLine[1])):
                                    containingObs.append(obs)
                                    chk_contain = 1
                                if chk_contain == 0:
                                    t10s = time.time()
                                    convexHull = self.createConvexhull(obs, tLine)
                                    self.splitBoundary(impededPathList, convexHull)
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
                                    s1, s2 = self.splitPolygon(split_line, obs)
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
                                            convexHull = self.createConvexhull(obs2, [pt])
                                            self.splitBoundary(subConvexPathList, convexHull)
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
                    #w.save(self.path + "After_graph_" + str(idx_loop1) + "_"+ str(idx_loop2) +"_"+ self.version_name)
                    #end of while2
                for line in impededPathList:
                    if not totalConvexPathList.has_key(line):
                        totalConvexPathList[line] = impededPathList[line]
                
                #totalConvexPathList.extend(impededPathList)
        totalGraph = self.createGraph(totalConvexPathList.keys())
        esp_n = networkx.dijkstra_path(totalGraph, odPointsList[0], odPointsList[1])
        esp = []
        for i in range(len(esp_n)-1):
            esp.append([esp_n[i], esp_n[i+1]])
        w = shapefile.Writer(shapefile.POLYLINE)
        w.field('nem')
        no_edges = 0
        for line in totalConvexPathList.keys():
            no_edges += 1
            w.line(parts=[[ list(x) for x in line ]])
            w.record('ff')
        w.save(self.path + "totalpath" + self.version_name + "%d" % pair[1] )              
        w = shapefile.Writer(shapefile.POLYLINE)
        w.field('nem')

        #calculate ESP distance 
        
        ESP_dist = 0
        for line in esp:
            segment = LineString(line)
            ESP_dist += segment.length
        ESP_dist = ESP_dist * 0.000189393939
        
        #category a: less than 5 mi
        #b: 5~10 mi
        #c: 10 ~
        for line in esp:
            w.line(parts=[[ list(x) for x in line ]])
            w.record('ff')
        if ESP_dist <= 3.33:
            w.save(self.path + "a_ESP_" + self.version_name + "%d" % pair[1])
        elif ESP_dist > 3.33: #and ESP_dist <= 10.005
            w.save(self.path + "b_ESP_" + self.version_name + "%d" % pair[1])
        #else:
            #w.save(self.path + "c_ESP_" + self.version_name + "%d" % pair[1])
        #sp_len_str = str(sp_l_set)[1:-1]
        
        #s = StringIO.StringIO()
        #sortby = 'cumulative'
        #ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
        #ps.print_stats()
        #print s.getvalue()
#        
#        s = StringIO.StringIO()
#        sortby = 'cumulative'
#        ps = pstats.Stats(pr2, stream=s).sort_stats(sortby)
#        ps.print_stats()
#        print s.getvalue()
        
        print "loop1: ", time_loop1
        print "Spatial Filtering: ", time_spatialFiltering
        print "loop1 crossingDict: ", time_loop1_crossingDict
        print "crossingDict: ", time_crossingDict
        print 'convexLoop: ', time_convexLoop
        print "time_contain: ", time_contain2
        print "impedingArcs: ", time_impedingArcs
        print "convexHUll: ", time_buildConvexHulls
        return 'convexpath %d %d %d %f %f %f' % (pair[1], no_edges, len(labeledObstaclePoly), time_convexLoop, time_crossingDict, time_buildConvexHulls)
        
path_imac = "/Users/insu/Dropbox/research/Distance restricted covering model/Refuelable drone path/data/"
path_home = "F:\\data\\grid\\"
path_air = "/Users/insuhong/Dropbox/research/Convexpath Approach/testField/data/"
path_ubuntu = "/home/insu/Dropbox/research/Convexpath Approach/Convexpath_HiDensity/data/"

start = time.time()
a = Convexpath(path_imac, "test_facil.shp", "test_demands.shp", "obstacles_disol.shp")

stop = time.time()
print stop - start
