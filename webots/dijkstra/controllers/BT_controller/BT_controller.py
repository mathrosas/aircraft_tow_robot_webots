from os.path import exists
import numpy as np
import py_trees
from py_trees.composites import Sequence, Parallel, Selector
from mapping import Mapping
from navigation import Navigation
from planning import Planning
from controller import Robot, Supervisor
from map_utils import generate_wp

# create the Robot instance
# robot = Robot()
# to see the pelotita
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# waypoints for mapping the environment
WP = generate_wp()

# check to verify if map already exists before mapping the environment    
class DoesMapExist(py_trees.behaviour.Behaviour):
    def update(self):
        file_exists = exists('cspace.npy')
        if(file_exists):
            print("Map already exists")
            map = np.load('cspace.npy')
            blackboard.write('map',map)
            return py_trees.common.Status.SUCCESS
        else:
            print("Map does not exist")
            map = np.zeros((300,300))
            blackboard.write('map',map)
            return py_trees.common.Status.FAILURE
        
class Blackboard:
    def __init__(self):
        self.data = {}
    def write(self,key,value):
        self.data[key] = value
    def read(self,key):
        return self.data.get(key)

# store the robot and waypoints to the shared blackboard
blackboard = Blackboard()
blackboard.write('robot',robot)
blackboard.write('waypoints',WP)

# behavior Tree with Sequence and Selector nodes
tree = Sequence("Main",children=[
        Selector("Does map exist?",children=[
            DoesMapExist("Test for map"),
            Parallel("Mapping",policy=py_trees.common.ParallelPolicy.SuccessOnOne(),children=[
                Mapping("Map the environment",blackboard),
                Navigation("Move around the aerodrome",blackboard)
                ])
            ],memory=True),                
        Planning("Move to hangar",blackboard,(-128,5))
        ],memory=True)
            
tree.setup_with_descendants()

while robot.step(timestep) != -1:
    # activate behavior tree
    tree.tick_once()

