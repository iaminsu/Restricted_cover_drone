import cPickle, pysal, Convexpath_module
from shapely.geometry import Point, Polygon, LineString, MultiPoint, MultiPolygon
from collections import defaultdict


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

facilities_f = "sample_sites_2.shp"
obstacles_f = "obstacles_p.shp"
path = "/Users/insuhong/Dropbox/research/Distance restricted covering model/Locating recharging station/data4/"

facil_pysal = pysal.IOHandlers.pyShpIO.shp_file(path+facilities_f)
obs_pysal = pysal.IOHandlers.pyShpIO.shp_file(path+obstacles_f)
facil_shp = generateGeometry(facil_pysal)
obs_shp = generateGeometry(obs_pysal)

warehouse_ID = [127,324]
w_dis_dict = defaultdict(dict)
for warehouse in warehouse_ID:
    w_Point = facil_shp[warehouse]
    for i in range(len(facil_shp)):
        if i != warehouse:
            a = Convexpath_module.Convexpath_shapely(path, w_Point, facil_shp[i], obs_shp)
            w_dis_dict[warehouse][i] = a.esp.length

f = open(path + "w_distance_dict.txt", 'w')
cPickle.dump(w_dis_dict, f)
f.close()


