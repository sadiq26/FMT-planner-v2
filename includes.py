#constants and helpful
import math
import numpy as np

step_len=1
MAXITER=15000
HEIGHT=100
WIDTH=100
COLORS = ['#6AB71F', '#FF5733', '#4DAAEA', '#C0120A']
DELTA=10  #Step size
GOAL_SAMPLING_RATE=0.05

#helpful functions
def calc_distance(p1,p2):
    
    return math.sqrt((p2[0]-p1[0])**2+(p2[1]-p1[1])**2)



#Node data structure for 
class Node2:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost=np.inf

def euclidean_distance(n1:Node2, n2:Node2):
    #
    dx = n2.x - n1.x
    dy = n2.y - n1.y
    return math.hypot(dx, dy), math.atan2(dy, dx)

def calc_dist2(x_start, x_end):
        return math.hypot(x_start.x - x_end.x, x_start.y - x_end.y)
