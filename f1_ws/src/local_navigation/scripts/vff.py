import numpy as np
import threading
import time
from datetime import datetime
import math
from Target import Target
from Parser import Parser
from visualization import Arrow, Sphere

time_cycle = 80

class VFF(threading.Thread):

    def __init__(self, pose3d, laser, motors):
        self.pose3d = pose3d
        self.laser = laser
        self.motors = motors

        # Car direction
        self.carx = 0.0
        self.cary = 0.0
        self.car_arrow = Arrow(0, (0, 1, 0, 1), (0, 0, 0, 1))

        # Obstacles direction
        self.obsx = 0.0
        self.obsy = 0.0
        self.obs_arrow = Arrow(1, (1, 0, 0, 1), (0, 0, 0, 1))

        # Average direction
        self.avgx = 0.0
        self.avgy = 0.0
        self.avg_arrow = Arrow(2, (0, 0, 0, 1), (0, 0, 0, 1))

        # Current target
        self.targetx = 0.0
        self.targety = 0.0
        self.targetid = "NaN"
        self.target_marker = Sphere(3)

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.kill_event)

        # Init targets
        parser = Parser('targets.json')
        self.targets = parser.getTargets()

    def getNextTarget(self):
        for target in self.targets:
            if target.isReached() == False:
                return target

        return None

    def getCarDirection(self):
        return (self.carx, self.cary)

    def getObstaclesDirection(self):
        return (self.obsx, self.obsy)

    def getAverageDirection(self):
        return (self.avgx, self.avgy)

    def getCurrentTarget(self):
        return (self.targetx, self.targety, self.targetid)

    def run (self):

        while (not self.kill_event.is_set()):
                        
            start_time = datetime.now()

            if not self.stop_event.is_set():
                self.execute()

            finish_Time = datetime.now()

            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            #print (ms)
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)


    def stop (self):
        self.stop_event.set()

    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()

    def kill (self):
        self.kill_event.set()

    def check_target(self):
        if(abs(self.targetx - self.posx) <= 6 and abs(self.targety - self.posy) <= 6):
            self.currentTarget.setReached(True)
            self.currentTarget = self.getNextTarget()
            print("Target Reached")
            if(self.currentTarget != None):
                return 1
            else:
                return 0

    def execute(self):
        self.currentTarget=self.getNextTarget()
        self.targetx = self.currentTarget.getPose().x
        self.targety = self.currentTarget.getPose().y
        self.posx = self.pose3d.getPose3d().x
        self.posy = self.pose3d.getPose3d().y
        self.targetid = self.currentTarget.getId()

        # TODO
        k_obstacle = 0.6
        k_car = 0.2
        k_target = -0.2

        target_vector = [self.targetx - self.posx, self.targety - self.posy]
        rot_angle = (-1) * (self.pose3d.getPose3d().yaw + math.pi/2)
        target_vector = [target_vector[0] * math.cos(rot_angle) - target_vector[1] * math.sin(rot_angle), target_vector[0] * math.sin(rot_angle) + target_vector[1] * math.cos(rot_angle)]
        target_marker = [target_vector[0], target_vector[1]]

        target_vector[0] = k_target * target_vector[0]
        target_vector[1] = k_target * target_vector[1]

        obstacle_vector = [0, 0]
        angle = 180

        for r in self.laser.getLaserData().values:
            if angle > 45 and angle < 135:
                obstacle_vector[0] += k_obstacle * (100/(r ** 2)) *  math.cos(math.radians(angle))
            else:			
                obstacle_vector[0] += k_obstacle * (10/(r ** 2)) *  math.cos(math.radians(angle))		
            angle -= 1

        if(abs(obstacle_vector[0]) > 20):
            obstacle_vector[0] = 20 *(abs(obstacle_vector[0])/obstacle_vector[0])
        
        car_vector = [0, 0]

        running = self.check_target()
        if(running == 0):
            self.motors.sendV(0)
            self.motors.sendW(0)
            

        car_vector[0] = k_car * (target_vector[0] + obstacle_vector[0])
        car_vector[1] = k_car * (target_vector[1] + obstacle_vector[1])
        
        self.motors.sendV(3.5)
        #Add PID to angle actual - where it should be headed times a constant
        k_angle = -0.3
        self.motors.sendW(k_angle * (car_vector[0]))

        # Car direction <GREEN>
        self.carx = -1 * car_vector[0]
        self.cary = car_vector[1]
        self.car_arrow.send_vector([self.carx, self.cary])

        # Obstacles direction <RED>
        self.obsx = -1 * obstacle_vector[0]
        self.obsy = obstacle_vector[1]
        self.obs_arrow.send_vector([self.obsx, self.obsy])

        # Target Vector <BLACK>
        self.avgx = -1 * target_vector[0]
        self.avgy = target_vector[1]
        self.avg_arrow.send_vector([self.avgx, self.avgy])

        # Target
        self.target_marker.send_vector(target_marker)