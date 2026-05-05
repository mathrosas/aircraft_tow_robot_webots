import py_trees
import math
import numpy as np
from grid import OccupancyGridMap, SLAM
from d_star_lite import DStarLite
from map_utils import world2map, map2world, display_map, obstacle_distance
from motor_utils import configure_speeds, set_motor_positions, set_motor_velocities

class Planning(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, goal):
        super(Planning, self).__init__(name)
        self.blackboard = blackboard
        self.robot = blackboard.read('robot')
        self.goal = goal

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
        self.leftBackMotor = self.robot.getDevice('wheel_left_back_joint')
        self.leftFrontMotor = self.robot.getDevice('wheel_left_front_joint')
        self.rightBackMotor = self.robot.getDevice('wheel_right_back_joint')
        self.rightFrontMotor = self.robot.getDevice('wheel_right_front_joint')
        set_motor_positions(self.leftBackMotor, self.leftFrontMotor, self.rightBackMotor, self.rightFrontMotor, float('inf'))
        set_motor_velocities(self.leftBackMotor, self.leftFrontMotor, self.rightBackMotor, self.rightFrontMotor, 0.0)

    def initialise(self):
        print(self.name)
        self.p1, self.p2, self.MAX_SPEED = configure_speeds(self.name)
        self.angles = np.linspace(124*math.pi/180, -124*math.pi/180, 1080)
        self.angles = self.angles[280:len(self.angles)-280]
        self.index = 0
        self.marker = self.robot.getFromDef("marker").getField("translation")
        self.finish = False
        self.WP_distance = 1

        self.map = self.blackboard.read('map')
        display_map(self.display, self.map)
        xw, yw = self.gps.getValues()[0], self.gps.getValues()[1]
        px,py = world2map(xw,yw)
        self.start = (px,py)        
        wx, wy = self.goal
        gx, gy = world2map(wx,wy)
        self.goal = (gx,gy)
        self.grid_map = OccupancyGridMap(map_data=self.map, x_dim=self.map.shape[0], y_dim=self.map.shape[1])
        self.d_star_lite = DStarLite(map=self.grid_map, s_start=self.start, s_goal=self.goal)
        self.slam = SLAM(map=self.grid_map,view_range=20)
        initial_path, _, _ = self.d_star_lite.move_and_replan(robot_position=self.start)

        # Convert the initial path to world coordinates and use it as the initial waypoints
        self.WP = [map2world(x, y) for x, y in initial_path]

    def update(self):
        if obstacle_distance(self.lidar) < 12.0 or (len(self.WP) - self.index) < 10:
            self.WP_distance = 0.1
            if self.new_obstacle():
                self.recalculate_path()
        else:
            self.WP_distance = 1
        self.follow_waypoints()
        return py_trees.common.Status.RUNNING

    def recalculate_path(self):
        # calculate new path here
        self.slam.set_ground_truth_map(OccupancyGridMap(map_data=self.map, x_dim=self.map.shape[0], y_dim=self.map.shape[1]))
        xw, yw = self.gps.getValues()[0], self.gps.getValues()[1]
        px,py = world2map(xw,yw)
        self.start = (px,py)

        wx, wy = self.WP[-1]
        gx, gy = world2map(wx,wy)
        self.goal = (gx,gy)    

        # SLAM
        self.d_star_lite.new_edges_and_old_costs, self.d_star_lite.sensed_map = self.slam.rescan(global_position=self.start)

        # Update D* Lite with the new start position and replan
        path, _, _ = self.d_star_lite.move_and_replan(robot_position=self.start)

        # Convert path back to world coordinates and set as waypoints
        self.WP = [map2world(x, y) for x, y in path]
        self.index = 1

    def follow_waypoints(self):
        if not self.finish:
            xw, yw = self.gps.getValues()[0], self.gps.getValues()[1]
            theta = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])
            
            px,py = world2map(xw, yw)
            self.display.setColor(0xFF0000)
            self.display.drawPixel(px,py)

            self.marker.setSFVec3f([*self.WP[self.index], 0])

            rho = np.sqrt((xw - self.WP[self.index][0]) ** 2 + (yw - self.WP[self.index][1]) ** 2)
            alpha = np.arctan2(self.WP[self.index][1] - yw, self.WP[self.index][0] - xw) - theta
            alpha = (alpha + np.pi) % (2 * np.pi) - np.pi

            if rho < self.WP_distance:
                self.index += 1
                if self.index == len(self.WP):
                    self.finish = True

            phildot = -alpha * self.p1 + rho * self.p2
            phirdot = alpha * self.p1 + rho * self.p2

            phildot = max(min(phildot, self.MAX_SPEED), -self.MAX_SPEED)        
            phirdot = max(min(phirdot, self.MAX_SPEED), -self.MAX_SPEED)

            set_motor_velocities(self.leftBackMotor, self.leftFrontMotor, self.rightBackMotor, self.rightFrontMotor, phildot, phirdot)
            return py_trees.common.Status.RUNNING
        else:
            set_motor_velocities(self.leftBackMotor, self.leftFrontMotor, self.rightBackMotor, self.rightFrontMotor, 0.0)
            return py_trees.common.Status.SUCCESS

    def new_obstacle(self):
        obstacles = 0
        new_obstacle = False
        self.updated_map = np.copy(self.map)
        xw, yw = self.gps.getValues()[0], self.gps.getValues()[1]
        theta = np.arctan2(self.compass.getValues()[0],self.compass.getValues()[1])

        # get lidar range measurements removing the first and last 80 data returns
        ranges = np.array(self.lidar.getRangeImage())
        ranges[ranges == np.inf] = 100000
        ranges[ranges > 12.0] = 100000
        ranges = ranges[280:-280]

        # convert lidar data returns in world coordinate                  
        w_T_r = np.array([[np.cos(theta),-np.sin(theta),xw],
                        [np.sin(theta),np.cos(theta),yw],
                        [0,0,1]])

        X_i = np.array([ranges*np.cos(self.angles), ranges*np.sin(self.angles), np.ones(len(ranges))])#+0.202
        D = w_T_r @ X_i

        SAFETY_MARGIN_OBSTACLE = 6
        margin_cells = int(SAFETY_MARGIN_OBSTACLE)

        # convert from world coordinates to map coordinates to display a map of the environment
        self.display.setColor(0xFFFFFF)
        for d in D.transpose():
            dx, dy = world2map(d[0], d[1])
            min_x = max(0, dx - margin_cells)
            max_x = min(self.updated_map.shape[0], dx + margin_cells+1)
            min_y = max(0, dy - margin_cells)
            max_y = min(self.updated_map.shape[1], dy + margin_cells+1)
            for x in range(min_x, max_x):
                for y in range(min_y, max_y):
                    if (self.updated_map[x,y] < 1):
                        obstacles += 1
                        new_obstacle = True
                        self.updated_map[x,y] = 1                        
                        self.display.drawPixel(x,y)

        self.map = self.updated_map

        return new_obstacle
