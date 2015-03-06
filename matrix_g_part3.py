import cPickle, pysal
from shapely.geometry import Point, Polygon, LineString, MultiPoint, MultiPolygon



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
path = "/Users/insuhong/Dropbox/research/Distance restricted covering model/Locating recharging station/data4/"

facil_pysal = pysal.IOHandlers.pyShpIO.shp_file(path+facilities_f)
facil_shp = generateGeometry(facil_pysal)

coords_dict = {}

for i in range(len(facil_shp)):
    coords_dict[(facil_shp[i].x, facil_shp[i].y)] = i

f = open(path + "FF_coords_Dictsample_sites_2.shp_sample_demand_2_p.shp_obstacles_p.shp.txt","w")
cPickle.dump(coords_dict, f)
f.close()

