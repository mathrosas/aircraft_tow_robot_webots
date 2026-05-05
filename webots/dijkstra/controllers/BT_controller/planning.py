import py_trees
import math
import numpy as np
from grid import OccupancyGridMap, SLAM
from map_utils import world2map, map2world, display_map, obstacle_distance
from motor_utils import configure_speeds, set_motor_positions, set_motor_velocities
from dijkstra import dijkstra_search

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
        set_motor_positions(self.leftBackMotor, 
                            self.leftFrontMotor, 
                            self.rightBackMotor, 
                            self.rightFrontMotor, 
                            float('inf'))
        set_motor_velocities(self.leftBackMotor,
                             self.leftFrontMotor,
                             self.rightBackMotor,
                             self.rightFrontMotor,
                             0.0)

    def initialise(self):
        print(self.name)
        self.p1, self.p2, self.MAX_SPEED = configure_speeds(self.name)
        # Filter out some LIDAR angles as in your code
        self.angles = np.linspace(124*math.pi/180, -124*math.pi/180, 1080)
        self.angles = self.angles[280:len(self.angles)-280]
        self.index = 0
        self.marker = self.robot.getFromDef("marker").getField("translation")
        self.finish = False
        self.WP_distance = 1

        # Display map
        self.map = self.blackboard.read('map')
        display_map(self.display, self.map)

        # Convert start from world to map
        xw, yw = self.gps.getValues()[0], self.gps.getValues()[1]
        px, py = world2map(xw, yw)
        self.start = (px, py)

        # Convert goal from world to map
        wx, wy = self.goal
        gx, gy = world2map(wx, wy)
        self.goal = (gx, gy)

        # Make a grid map and a SLAM object (if you still want to update obstacles)
        self.grid_map = OccupancyGridMap(map_data=self.map,
                                         x_dim=self.map.shape[0],
                                         y_dim=self.map.shape[1])
        self.slam = SLAM(map=self.grid_map, view_range=20)

        # ---------------------------------------------------------------------
        # Instead of D* Lite, call a Dijkstra-based planner:
        # ---------------------------------------------------------------------
        initial_path = dijkstra_search(self.grid_map, self.start, self.goal)

        # Convert the path to world coordinates for driving
        self.WP = [map2world(x, y) for (x, y) in initial_path]

    def update(self):
        # If there's an obstacle close or near the end, reduce WP_distance
        if obstacle_distance(self.lidar) < 12.0 or (len(self.WP) - self.index) < 10:
            self.WP_distance = 0.1
            if self.new_obstacle():
                self.recalculate_path()
        else:
            self.WP_distance = 1

        self.follow_waypoints()
        return py_trees.common.Status.RUNNING

    def recalculate_path(self):
        # Update map with newly sensed obstacles
        self.slam.set_ground_truth_map(OccupancyGridMap(map_data=self.map,
                                                        x_dim=self.map.shape[0],
                                                        y_dim=self.map.shape[1]))

        # Current position in map coordinates
        xw, yw = self.gps.getValues()[0], self.gps.getValues()[1]
        px, py = world2map(xw, yw)
        self.start = (px, py)

        # We keep the last WP in world coords as the ultimate goal
        wx, wy = self.WP[-1]
        gx, gy = world2map(wx, wy)
        self.goal = (gx, gy)

        # (Optional) SLAM step
        self.slam.rescan(global_position=self.start)

        # ---------------------------------------------------------------------
        # Re-run Dijkstra whenever we detect a new obstacle
        # ---------------------------------------------------------------------
        path = dijkstra_search(self.grid_map, self.start, self.goal)

        # Convert path back to world coordinates
        self.WP = [map2world(x, y) for x, y in path]
        self.index = 1  # start tracking from second node (index=1)

    def follow_waypoints(self):
        if not self.finish:
            xw, yw = self.gps.getValues()[0], self.gps.getValues()[1]
            # In Webots, compass heading is reversed: 
            #   angle = atan2(compass_x, compass_y)
            theta = np.arctan2(self.compass.getValues()[0],
                               self.compass.getValues()[1])

            # Just for visualization in the display:
            px, py = world2map(xw, yw)
            self.display.setColor(0xFF0000)
            self.display.drawPixel(px, py)

            # Place a marker (optional) at the next waypoint
            self.marker.setSFVec3f([*self.WP[self.index], 0])

            # Compute distance and heading to next WP
            rho = np.sqrt((xw - self.WP[self.index][0])**2 + (yw - self.WP[self.index][1])**2)
            alpha = np.arctan2(self.WP[self.index][1] - yw,
                               self.WP[self.index][0] - xw) - theta
            # Wrap alpha to [-pi, pi]
            alpha = (alpha + np.pi) % (2 * np.pi) - np.pi

            # Check if we reached the current WP
            if rho < self.WP_distance:
                self.index += 1
                # If we reached the end of the path
                if self.index == len(self.WP):
                    self.finish = True

            # Simple P-controller for wheels
            phildot = -alpha * self.p1 + rho * self.p2
            phirdot =  alpha * self.p1 + rho * self.p2

            # Clamp wheel speeds
            phildot = max(min(phildot, self.MAX_SPEED), -self.MAX_SPEED)
            phirdot = max(min(phirdot, self.MAX_SPEED), -self.MAX_SPEED)

            set_motor_velocities(self.leftBackMotor,
                                 self.leftFrontMotor,
                                 self.rightBackMotor,
                                 self.rightFrontMotor,
                                 phildot,
                                 phirdot)
            return py_trees.common.Status.RUNNING
        else:
            # Stop the motors
            set_motor_velocities(self.leftBackMotor, 
                                 self.leftFrontMotor, 
                                 self.rightBackMotor, 
                                 self.rightFrontMotor, 
                                 0.0)
            return py_trees.common.Status.SUCCESS

    def new_obstacle(self):
        obstacles = 0
        new_obstacle = False
        self.updated_map = np.copy(self.map)
        xw, yw = self.gps.getValues()[0], self.gps.getValues()[1]
        theta = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])

        # get lidar data
        ranges = np.array(self.lidar.getRangeImage())
        ranges[ranges == np.inf] = 100000
        ranges[ranges > 12.0] = 100000
        ranges = ranges[280:-280]

        # Convert to world coords
        w_T_r = np.array([[np.cos(theta), -np.sin(theta), xw],
                          [np.sin(theta),  np.cos(theta), yw],
                          [0,              0,             1  ]])

        X_i = np.array([ranges * np.cos(self.angles), 
                        ranges * np.sin(self.angles), 
                        np.ones(len(ranges))])
        D = w_T_r @ X_i

        SAFETY_MARGIN_OBSTACLE = 6
        margin_cells = int(SAFETY_MARGIN_OBSTACLE)

        # Mark obstacles in the local area
        self.display.setColor(0xFFFFFF)
        for d in D.transpose():
            dx, dy = world2map(d[0], d[1])
            min_x = max(0, dx - margin_cells)
            max_x = min(self.updated_map.shape[0], dx + margin_cells + 1)
            min_y = max(0, dy - margin_cells)
            max_y = min(self.updated_map.shape[1], dy + margin_cells + 1)
            for x in range(min_x, max_x):
                for y in range(min_y, max_y):
                    if (self.updated_map[x, y] < 1):
                        obstacles += 1
                        new_obstacle = True
                        self.updated_map[x, y] = 1
                        self.display.drawPixel(x, y)

        self.map = self.updated_map
        return new_obstacle