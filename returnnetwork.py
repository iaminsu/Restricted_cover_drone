import pysal, shapefile, os
from shapely.geometry import Point, Polygon, LineString, MultiPoint, MultiPolygon


path = "f:\\Dropbox\\research\\Distance restricted covering model\\Locating recharging station\\data7\\ESPs\\"


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


graph = []
names = []

for line in os.listdir(path):
    if line.endswith('shp'):
        line_f = pysal.IOHandlers.pyShpIO.shp_file(path + line)
        line_shp = generateGeometry(line_f)
        leng = 0
        for i in line_shp:
            leng += i.length 
        if leng <= 52800:
            graph.append(line_shp)
            names.append(line)

w = shapefile.Writer(shapefile.POLYLINE)
w.field('nem')
for line in graph:
    w.line(parts=[[ list(x) for x in list(line[0].coords)]])
    w.record('chu')
w.save(path + 'r_n')
print names