#!/usr/bin/python3

class Object:

    def __init__(self, id, points):
        """ Takes in a unique ID identifying the object AND an array consisting of location tuples of the form
        (x, y) containing every point within the object. """
        self.id = id
        self.points = points

object = Object(id="Cylinder1", points=[(0,0), (1,0), (1,1), (0,1)])
