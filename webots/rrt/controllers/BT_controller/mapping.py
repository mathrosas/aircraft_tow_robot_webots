import py_trees
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy import signal
from map_utils import world2map

class Mapping(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Mapping, self).__init__(name)
        self.hasrun = False
        self.robot = blackboard.read('robot')
        
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        self.lidar = self.robot.getDevice("Hokuyo UTM-30LX")
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()
        self.display = self.robot.getDevice("display")
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        
    def initialise(self):
        print(self.name)
        self.map = np.zeros((300,300))
        self.kernel = np.ones((30,30))  
        self.angles = np.linspace(124*math.pi/180,-124*math.pi/180,1080)
        self.angles = self.angles[280:len(self.angles)-280]
        
    def update(self):
        self.hasrun = True
        # get pose of the robot
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        theta = np.arctan2(self.compass.getValues()[0],self.compass.getValues()[1])

        # convert to map coordinates to draw robot's location on the map
        px, py = world2map(xw,yw)          

        # get lidar range measurements removing the first and last 80 data returns
        ranges = np.array(self.lidar.getRangeImage())
        ranges[ranges == np.inf] = 100000
        ranges[ranges > 12.0] = 100000
        ranges = ranges[280:-280]

        # convert lidar data returns in world coordinate                  
        w_T_r = np.array([[np.cos(theta),-np.sin(theta),xw],
                        [np.sin(theta),np.cos(theta),yw],
                        [0,0,1]])

        X_i = np.array([ranges*np.cos(self.angles)+0.202, ranges*np.sin(self.angles), np.ones(len(ranges))])
        D = w_T_r @ X_i

        # convert from world coordinates to map coordinates to display a map of the environment                
        for d in D.transpose():
            px, py = world2map(d[0], d[1])
            if (map[px, py] < 1):
                map[px, py] += 0.0001
            v = int(map[px, py]*255)
            color = (v*256**2 + v*256 + v)
            self.display.setColor(int(color))
            self.display.drawPixel(px, py)
        

        return py_trees.common.Status.RUNNING
    
    # generate obstacle map to be used for planning and navigation    
    def terminate(self, new_status):
        if(self.hasrun):
            cspace = signal.convolve2d(self.map,self.kernel,mode='same')
            plt.figure(0)
            plt.imshow(cspace)
            plt.figure(1)
            plt.imshow(cspace > 0.9)            
            plt.show()
            np.save("cspace0",cspace)
            np.save("cspace", cspace > 0.9)

            
            new_status = py_trees.common.Status.SUCCESS

            return new_status