#My own FMT* implementation
import random
#import pygame
#import math
#import numpy as np
from includes import*

class Map_graph:
    ''' Class for holding the physical characteristics of the environments obstacle location and so on'''
    def __init__(self):

        self.height=HEIGHT
        self.width=WIDTH
        self._obstacle = []
        self.num_obs = random.randint(5, 10) #generate a random no of obstacles
        self.obstacle_props=self.generate_obstacles(self.num_obs)
        self.start,self.end=self.start_position() #plot the start and end position
        self.goal_distance=calc_distance(self.start, self.end) #helper here


    def generate_obstacles(self,num):
        random.seed(0)

        obstacles=[]
        while len(obstacles)<num:
            overlap=[]
            center = (self.height*random.random(), self.width*random.random())
            radius = 10*random.random()

            for _,props in enumerate(obstacles):
                
                if calc_distance(center,props[0])>=radius+props[1]:
                    
                                                                
                    overlap.append(False)
                else:
                    overlap.append(True)
            if any(overlap):
                pass
            else:
                obstacles.append([center,radius])
        return obstacles
    
    def collision_check(self,point):           #Rewrite as node centric
        for _,props in enumerate(self.obstacle_props):
            if calc_distance(point,props[0])<=props[1]:
                #print(props[1])
                return True
            else:
                pass
        return False
    
    def collision_check__test_node(self,nodee):           #Rewrite as node centric
        for _,props in enumerate(self.obstacle_props):
            if calc_distance((nodee.x,nodee.y),props[0])<=props[1]:
                print(props[1])
                return True
            else:
                pass
        return False


    def start_position(self):
        start_ok=False
        end_ok=False

        while not start_ok:
            start=(10+20*random.random(),10+20*random.random())
            if self.collision_check(start):
                pass
            else:
                start_ok=True

        while not end_ok:
            end=(30+20*random.random(),30+20*random.random())

            if self.collision_check(end):
                pass
            else:
                end_ok=True
        
        print(start,end)
        
        return start, end
    



