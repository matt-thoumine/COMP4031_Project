import numpy as np
import gpxpy
import gpxpy.gpx
import g

class Path():
    
    def __init__(self,namep):
        self.name=namep
        self.points = routing()       
        self.radius = 20 #Change this to change road width
        
    def display(self,canvas):
        canvas.delete(self.name)
        points = []
        for p in self.points:
            points.append([p[0],p[1]])
        canvas.create_line(points, fill='grey', width=self.radius*2, tags=self.name)
        canvas.create_line(points, fill='black', width=2, tags=self.name)


def routing():
    #Much of below taken from https://pypi.org/project/gpxpy/ 
    #Modified for purpose
    
    #Change File Name Depending Upon Route
    gpx_file = open('25km_Straight_0.gpx','r')
    gpx = gpxpy.parse(gpx_file)
    
    diff = 0
    route = []
    
    #Scale Factor for degrees long/lat to meters
    sf = 111000/1
    
    for track in gpx.tracks:
        for segment in track.segments:
            for point in segment.points:
                if len(route) != 0:
                    p = np.array([point.latitude*sf,point.longitude*sf,point.elevation]) - diff
                else:
                    diff = np.array([point.latitude*sf,point.longitude*sf,point.elevation]) - g.course_start
                    p = g.course_start
                
                route.append(p)
    
    #Below for now unincluded, may be useful with different gpx files in the future
    '''
    for waypoint in gpx.waypoints:
        print('waypoint {0} -> ({1},{2})'.format(waypoint.name, waypoint.latitude, waypoint.longitude))
    
    for route in gpx.routes:
        for point in route.points:
            print('Point at ({0},{1}) -> {2}'.format(point.latitude, point.longitude, point.elevation))
    '''
    
    return route
