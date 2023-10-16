import numpy as np
import random
from includes import*
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.collections as mpl


class FMTstar_palnner_v2:
    '''This is a version combining environment construction, planning and plotting'''
    def __init__(self):
        
        #cspace generation
        self.height=HEIGHT
        self.width=WIDTH
        self._obstacle = []
        self.num_obs = random.randint(50, 100) #generate a random no of obstacles
        
         #plot the start and end position
         #helper here
        
        #plotting
        #Reminder: Most of the functions of plotting are called at the planning stage

        #FMT planning
        
        
        self.no_of_samples=0
        self.V = set()
        self.V_unvisited = set()
        self.V_open = set()
        self.V_closed = set()
        self.sample_numbers = 1000

        self.x_range=(0,WIDTH)
        self.y_range=(0,HEIGHT)
    def Inint(self):
        #
        self.obstacle_props=self.generate_obstacles(self.num_obs)
        self.start,self.end=self.start_position()
        self.goal_distance=calc_distance(self.start, self.end)
        self.start_position=Node2(self.start[0],self.start[1])
        self.end_position=Node2(self.end[0],self.end[1])


def mainv2(self):
    
    
    
    
    pass


#FMT* PLanner Class

    def planning(self):
        #
        #line 1 to 4 of the algorithm
        samples=self.Sample_free()
        #self.x_init.cost=0.0
        self.start_position.cost=0.0
        self.V.add(self.start_position)
        self.V.update(samples)
        self.V_unvisited.update(samples)
        self.V_unvisited.add(self.end_position)
        self.V_open.add(self.start_position)
        z=self.start_position
        n=self.sample_numbers
        self.rn = 40 * math.sqrt((math.log(n) / n))
        Visited=[]
        while z is not self.end_position:
            V_open_new=set()
            X_near = self.Near(self.V_unvisited, z, self.rn)
            Visited.append(z)

            for x in X_near:
                Y_Near=self.Near(self.V_open, x, self.rn)
                cost_list = {y: y.cost + self.Cost(y, x) for y in Y_Near}
                y_min=min(cost_list,key=cost_list.get)

                if not self.collision_checker(y_min,x):
                    x.parent = y_min
                    V_open_new.add(x)
                    self.V_unvisited.remove(x)
                    x.cost = y_min.cost + self.Cost(y_min, x)

            self.V_open.update(V_open_new)
            self.V_open.remove(z)
            self.V_closed.add(z)

            if not self.V_open:
                break
            cost_open = {y: y.cost for y in self.V_open}
            z = min(cost_open, key=cost_open.get)
            path_x, path_y = self.ExtractPath()
            self.animation_v2(path_x,path_y, Visited[1: len(Visited)])
            print("Problem Solved")
            plt.show()


    def collision_checker(self,nodee:Node2):
        
         #Rewrite as node centric Returns true if collision
        for _,props in enumerate(self.obstacle_props):
            
            if calc_distance((nodee.x,nodee.y),props[0])<=props[1]:
                
                #print(props[1])
                return True
            else:
                pass
        return False
    

    def is_collision_free_path(self,n1:Node2,n2:Node2):

        if self.collision_checker(n1)==True or self.collision_checker(n2)==True:
            return True
        else:
            return False
    
    #Algorithm in line one of paper
    def Sample_free(self):
        n=self.no_of_samples
        self.delta=DELTA
        Sample=set()
        i=0
        while i<n:
            
            node = Node2((random.uniform(self.x_range[0] + self.delta, self.x_range[1] - self.delta),
                         random.uniform(self.y_range[0] + self.delta, self.y_range[1] - self.delta)))

            if self.collision_checker(node):
                continue
            else:
                Sample.add(node)
                i+=1
                    
        return Sample 
    
    @staticmethod
    def Near(nodelist, z, rn):
        #Select nodes within a nodelist
        return {nd for nd in nodelist
                if 0 < (nd.x - z.x) ** 2 + (nd.y - z.y) ** 2 <= rn ** 2}

    def Cost(self, x_start, x_end):
        if self.collision_checker(x_start, x_end):
            return np.inf
        else:
            return calc_dist2(x_start,x_end)
        
    #def choose_goal_point(self):
     #   Near=self.Near(self.V,self.x_goal,2.0)
      #  cost={y: y.cost + self.Cost(y, self.x_goal) for y in Near}
       # return min(cost, key=cost.get)
    
    def ExtractPath(self):
        path_x, path_y = [], []
        node = self.end_position

        while node.parent:
            path_x.append(node.x)
            path_y.append(node.y)
            node = node.parent

        path_x.append(self.start_position.x)
        path_y.append(self.start_position.y)

        return path_x, path_y


    def animation_v2(self, path_x, path_y, visited):
        self.plot_environmentv2()
        

        for node in self.V:
            plt.plot(node.x, node.y, marker='.', color='lightgrey', markersize=3)

        count = 0
        for node in visited:
            count += 1
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], '-g')
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            if count % 10 == 0:
                plt.pause(0.001)

        plt.plot(path_x, path_y, linewidth=2, color='red')
        plt.pause(0.01)
        plt.show()






























    #Cspace functions
    def generate_obstacles(self,num):
        '''generate n amount of random obsctacles part of cspace'''
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
    
    def collision_check(self,point):
        
                   #Rewrite as node centric
        for _,props in enumerate(self.obstacle_props):
            if calc_distance(point,props[0])<=props[1]:
                #print(props[1])
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
            end=(70+20*random.random(),70+20*random.random())

            if self.collision_check(end):
                pass
            else:
                end_ok=True
        
        print(start,end)
        
        return start, end
    

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    #plotting functions
    def animation(self, nodelist, path, animation=False):
        self.plot_environmentv2()
        self.plot_visited(nodelist, animation)
        self.plot_path(path)
    
    
    def plot_environmentv2(self):

        FIG, AX = plt.subplots(nrows=1, ncols=1, sharex=True, sharey=True, figsize=(9, 9))
        plt.ylim(0, self.width)
        plt.xlim(0, self.height)
        plt.scatter(self.start[0], self.start[1], s=500, c=COLORS[0], marker='v')
        plt.scatter(self.end[0], self.end[1], s=800, c=COLORS[1], marker='v')
        plt.plot(self.start[0],self.start[1],'bo',)
        plt.plot()
    
        OBSTACLES = [plt.Circle(j[0], j[1]) for i, j in enumerate(self.obstacle_props)]
        OBS_PATCHES = mpl.collections.PatchCollection(OBSTACLES, facecolors='black')
        AX.add_collection(OBS_PATCHES)
        
    def animation_v2(self, path_x, path_y, visited,V):
        self.plot_environmentv2()
        

        for node in V:
            plt.plot(node.x, node.y, marker='.', color='lightgrey', markersize=3)

        count = 0
        for node in visited:
            count += 1
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], '-g')
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            if count % 10 == 0:
                plt.pause(0.001)

        plt.plot(path_x, path_y, linewidth=2, color='red')
        plt.pause(0.01)
        plt.show()

        
    
    def plot_visited(self,nodelist):
        #
        count=0
        for node in nodelist:
            #
            count+=1
            if node.parent:
                #
                plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                plt.gcf().canvas.mpl_connect('key_release_event',lambda event:[exit(0) if event.key == 'escape' else None])
                if count %10==0:
                    #
                    plt.pause(0.01)
        else:
            #
            for node in nodelist:
                #
                if node.parent:
                    #
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    
   
    def plot_path(path):
        if len(path)!=0:
            plt.plot([x[0] for x in path], [x[1] for x in path], '-r', linewidth=2)
            plt.pause(0.01)
            print(path)
        plt.show()




planner=FMTstar_palnner_v2()
planner.planning
plt.show()