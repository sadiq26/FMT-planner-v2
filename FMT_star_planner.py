from cspace import *
from includes import*
from make_plot import *
from further_includes import *



class FMT_star_plannner:
    def __init__(self) -> None:
        self.env=ENV
        self.start_position=Node2(self.env.start[0],self.env.start[1])
        self.end_position=Node2(self.env.end[0],self.env.end[1])
        self.no_of_samples=1000
        self.plotting =Plot(self.start_position,self.end_position )  
        self.obstacle_props=self.env.obstacle_props
        print(self.obstacle_props) 
        #self.fig, self.ax = plt.subplots()
        #self.plotting.plot_environmentv2()
        #plot everything here
        #self.FIG, self.AX = plt.subplots(nrows=1, ncols=1, sharex=True, sharey=True, figsize=(9, 9))
        #plt.ylim(0, self.env.width)
        #plt.xlim(0, self.env.height)
        #plt.scatter(self.env.start[0], self.env.start[1], s=500, c=COLORS[0], marker='v')
        #plt.scatter(self.env.end[0], self.env.end[1], s=800, c=COLORS[1], marker='v')
        #plt.plot(self.env.start[0],self.env.start[1],'bo',)
        #plt.plot()
        #plt.title("FMT* by Salman")
        #self.OBSTACLES = [plt.Circle(j[0], j[1]) for i, j in enumerate(self.env.obstacle_props)]
        #self.OBS_PATCHES = mpl.PatchCollection(self.OBSTACLES, facecolors='black')
        #self.AX.add_collection(self.OBS_PATCHES)
        #plt.show()
        self.delta=DELTA
        self.FIG, self.AX = plt.subplots()
        plt.ylim(0, self.env.width)
        plt.xlim(0, self.env.height)

        self.V = set()
        self.V_unvisited = set()
        self.V_open = set()
        self.V_closed = set()
        self.sample_numbers = 500

        self.x_range=(0,WIDTH)
        self.y_range=(0,HEIGHT)
        #self.ppp=self.planning()
        #self.FIG, self.AX = plt.subplots(nrows=1, ncols=1, sharex=True, sharey=True, figsize=(9, 9))
        #plt.show(block=False)

    def Init(self):
        self.samples = self.Sample_free()

        self.start_position.cost = 0.0
        self.V.add(self.start_position)
        self.V.update(self.samples)
        self.V_unvisited.update(self.samples)
        self.V_unvisited.add(self.end_position)
        self.V_open.add(self.start_position)

    def planning(self):
        self.Init()
        z=self.start_position
        n=self.sample_numbers
        rn = 40 * math.sqrt((math.log(n) / n))
        Visited=[]
        #print("planning")
        
        while z is not self.end_position:
            V_open_new=set()
            X_near = self.Near(self.V_unvisited, z, rn)
            Visited.append(z)

            for x in X_near:
                #print(x)
                Y_Near=self.Near(self.V_open, x, rn)
                cost_list = {y: y.cost + self.Cost(y, x) for y in Y_Near}
                y_min=min(cost_list,key=cost_list.get)

                if not self.is_collision_free_path(y_min,x):
                    x.parent = y_min
                    V_open_new.add(x)
                    self.V_unvisited.remove(x)
                    x.cost = y_min.cost + self.Cost(y_min, x)
            #print("This position you ar at")
            self.V_open.update(V_open_new)
            self.V_open.remove(z)
            self.V_closed.add(z)

            if not self.V_open:
                print("Open set empty")
                break
            cost_open = {y: y.cost for y in self.V_open}
            z = min(cost_open, key=cost_open.get)

            path_x, path_y = self.ExtractPath()
            self.animation_v3(path_x,path_y, Visited[1: len(Visited)])
            print("Treeis growing")
            #print(Visited,"visited")
            #plt.show()

    
    def animation_v3(self,path_x, path_y, visited):

        #self.FIG, self.AX = plt.subplots(nrows=1, ncols=1, sharex=True, sharey=True, figsize=(9, 9))
        #self.FIG, self.AX = plt.subplots()

        #plt.ylim(0, self.env.width)
        #plt.xlim(0, self.env.height)
        #plt.scatter(self.env.start[0], self.env.start[1], s=500, c=COLORS[0], marker='v')
        #plt.scatter(self.env.end[0], self.env.end[1], s=800, c=COLORS[1], marker='v')
        #plt.plot(self.env.start[0],self.env.start[1],'bo',)
        #plt.plot(self.FIG)
        #plt.title("FMT* by Salman")
        #self.plot_env()
        
        #self.OBSTACLES = [plt.Circle(j[0], j[1]) for i, j in enumerate(self.env.obstacle_props)]
        #self.OBS_PATCHES = mpl.PatchCollection(self.OBSTACLES, facecolors='black')
        #self.OBS_PATCHES = self.AX.add_patch((),self.OBSTACLES)
        #for 
        #self.AX.add_collection(self.OBS_PATCHES)
        #plt.show()
        #self.plot_env("FMT* by Salman")
        #self.OBSTACLES = [plt.Circle(j[0], j[1]) for i, j in enumerate(self.env.obstacle_props)]
        #self.ax.add_patch(patches.Circle())

        for (ox,oy),r in self.env.obstacle_props:
             self.AX.add_patch(patches.Circle((ox, oy), r,edgecolor='black',facecolor='black',fill=True))
             
             
        plt.scatter(self.env.start[0], self.env.start[1], s=500, c=COLORS[0], marker='v')
        plt.scatter(self.env.end[0], self.env.end[1], s=800, c=COLORS[1], marker='v')
        plt.plot(self.env.start[0],self.env.start[1],'bo',)
        plt.title("FMT* by Salman")
        plt.axis("equal")




        for node in self.V:
            plt.plot(node.x, node.y, marker='.', color='lightgrey', markersize=3)
            #plot all the samples that we have 
        count = 0
        for node in visited:
            count += 1
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], '-m')
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            if count % 10 == 0:
                plt.pause(0.01)

        plt.plot(path_x, path_y, linewidth=2, color='red')
        plt.pause(0.01)
        plt.show()


    def plot_env(self):
        #
        pass

        


    def collision_checker(self,nodee:Node2):
        
         #Rewrite as node centric Returns true if collision
        for _,props in enumerate(self.env.obstacle_props):
            
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
        n=self.sample_numbers
        
        Sample=set()
        i=0
        while i<n:
            node = Node2((random.uniform(self.x_range[0] + self.delta, self.x_range[1] - self.delta)),
                          (random.uniform(self.y_range[0] + self.delta, self.y_range[1] - self.delta)))
            #print("sampling free")
            if self.collision_checker(node):
                continue
            else:
                Sample.add(node)
                
                #print(Sample)
                i+=1
                    
        return Sample 
    
    @staticmethod
    def Near(nodelist, z, rn):
        #Select nodes within a nodelist
        return {nd for nd in nodelist
                if 0 < (nd.x - z.x) ** 2 + (nd.y - z.y) ** 2 <= rn ** 2}

    def Cost(self, x_start, x_end):
        if self.is_collision_free_path(x_start, x_end):
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
        #self.plotting.plot_environmentv2()
        

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


    #Here is where the magic happens
 


fmt=FMT_star_plannner()
lcdgg=fmt.planning()
#plt.show()

