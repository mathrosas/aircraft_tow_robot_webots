import py_trees
import math
import numpy as np
from map_utils import world2map
from motor_utils import configure_speeds, set_motor_positions, set_motor_velocities

class Navigation(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Navigation, self).__init__(name)
        self.blackboard = blackboard
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
        self.leftBackMotor = self.robot.getDevice('wheel_left_back_joint')
        self.leftFrontMotor = self.robot.getDevice('wheel_left_front_joint')
        self.rightBackMotor = self.robot.getDevice('wheel_right_back_joint')
        self.rightFrontMotor = self.robot.getDevice('wheel_right_front_joint')
        set_motor_positions(self.leftBackMotor, self.leftFrontMotor, self.rightBackMotor, self.rightFrontMotor, float('inf'))
        set_motor_velocities(self.leftBackMotor, self.leftFrontMotor, self.rightBackMotor, self.rightFrontMotor, 0.0)
        
    def initialise(self):
        print(self.name)
        self.p1, self.p2, self.MAX_SPEED = configure_speeds(self.name)
        self.map = np.zeros((300,300))
        self.angles = np.linspace(124*math.pi/180,-124*math.pi/180,1080) 
        self.angles = self.angles[280:len(self.angles)-280]
        self.index = 0        
        self.marker = self.robot.getFromDef("marker").getField("translation")
        self.finish = False
        self.WP = self.blackboard.read('waypoints')        
        
    def update(self):
        if not self.finish:
            xw, yw = self.gps.getValues()[0], self.gps.getValues()[1]
            theta = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])

            if self.name != "Move around the aerodrome":
                px,py = world2map(xw, yw)
                self.display.setColor(0xFFF000)
                self.display.drawPixel(px,py)

            self.marker.setSFVec3f([*self.WP[self.index], 0])

            rho = np.sqrt((xw - self.WP[self.index][0]) ** 2 + (yw - self.WP[self.index][1]) ** 2)
            alpha = np.arctan2(self.WP[self.index][1] - yw, self.WP[self.index][0] - xw) - theta
            alpha = (alpha + np.pi) % (2 * np.pi) - np.pi

            if rho < 0.3:
                self.index += 1
                if self.index == len(self.WP) - 1:
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