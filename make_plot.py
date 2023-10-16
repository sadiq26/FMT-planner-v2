import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.collections as mpl
from further_includes import *
from includes import*
from cspace import *


class Plot:
    def __init__(self,x_start,x_goal):
        self.xI,self.xG=x_start,x_goal
        self.env=ENV
        self.obstacle_prop=self.env.obstacle_props
    
 
