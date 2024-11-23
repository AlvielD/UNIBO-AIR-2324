"""
============== UniBo: AI and Robotics 2024 ==============
Base code: internal map of the environment, to be used for the Final Challenge
This version may needs to be customized and improved by the students

(c) 2024 Alessandro Saffiotti
"""
from math import sqrt


class WorldMap:
    """
    This is the static map of the environment where our robot lives
    It contains the given position of all named objects and their topological relations
    """
    geometry  = {}              # extent of rooms, as (bottomleftx, bottomlefty, xsize, yzise)
    topology  = {}              # relations between rooms and objects
    locations = {}              # poses and size of known objects, as (mt, mt, rad, radius)
    properties = {}             # properties of objects
    holding = {}
    startpose = (0.0, 0.0, 0.0) # starting pose of robot in map's coordinates

    def find_room (self, pos):
        """
        Given an (x,y) position, find the room where it belongs
        For each room, it checks if (x,y) falls inside that room's bounding box
        """
        for room, bbox in self.geometry.items():
            if pos[0] < bbox[0] or pos[0] > bbox[0]+bbox[2]:
                continue
            if pos[1] < bbox[1] or pos[1] > bbox[1]+bbox[3]:
                continue
            return room
        return None

    def find_location (self, pos):
        """
        Given an (x,y) position, find its symbolic location
        For each object, if checks if (x,y) is close to it considering its radius
        If the given pos is not close to any object, return 'openspace'
        """
        for object, loc in self.locations.items():
            dx = pos[0] - loc[0]
            dy = pos[1] - loc[1]
            dist = sqrt(dx*dx + dy*dy)
            if dist < loc[3] + 1.0:
                return object
        return 'openspace'


""" 
Here is the specific map for our environment
"""

map = WorldMap()
map.geometry = {
    'Room1' : [-11.0, -1.5, 12.0, 8.0],
    'Room2' : [-11.0, -9.5, 8.0, 8.0],
    'Room3' : [-3.0, -9.5, 4.0, 8.0],
    'Room4' : [1.0, -9.5, 10.0, 16.0]
}
map.topology = {
    'Room1' : ['bed1', 'wardrobe1', 'D2', 'D3'],
    'Room2' : ['fridge1', 'sink1', 'stove1', 'D4'],
    'Room3' : ['entrance', 'D1', 'D3', 'D4'],
    'Room4' : ['table1', 'table2', 'table3', 'D1', 'D2']
}
map.locations = {
    'table1'    : (5.0, 0.0, 0.0, 0.8),
    'table2'    : (5.0, 5.5, 0.0, 1.2),
    'table3'    : (5.0, -5.0, 0.0, 1.6),
    'bed1'      : (-9.5, 1.5, 1.57, 1.6),
    'fridge1'   : (-7.0, -9.0, 3.14, 1.2),
    'sink1'     : (-10.4, -5.0, 1.57, 1.0),
    'stove1'    : (-7.0, -2.3, 0.0, 0.8),
    'wardrobe1' : (-8.0, 5.76, 0.0, 2.2),
    'entrance'  : (-1.0, -8.0, 0.0, 0.0),
    'D1'        : (1.0, -5.4, 0.0, 0.4),   
    'D2'        : (1.0, 2.6, 0.0, 0.4),   
    'D3'        : (-1.2, -1.2, 1.57, 0.4),   
    'D4'        : (-3.0, -5.4, 0.0, 0.4)
}
map.properties = {
    'D1' : 'Open',
    'D2' : 'Open',
    'D3' : 'Open',
    'D4' : 'Open'
}
map.holding = {
    'me' : None
}
map.startpose = (3.0, -2.0, 0.0)
#map.startpose = (-6.0, -8.0, 0.0)
#map.startpose = (4.0, -7.0, 0.0)
#map.startpose = (8.0, -1.0, 0.0)
#map.startpose = (-3.0, -5.0, 0.0)

#map.startpose = (-7.0, 3.0, 0.0)
#map.startpose = (-7.0, -5.0, 0.0)
#map.startpose = (-2.0, -5.0, 0.0)