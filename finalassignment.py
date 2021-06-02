#!/usr/bin/env python
import rospy
import time
import math
import sys
import os
import rospkg
import csv
sys.path.append(os.path.join(rospkg.RosPack().get_path('me_cs301_robots'), 'scripts'))
from robot_control import RobotControl

'''
MotorIDstrings for Hexapod follow the convention legN_jM, where N can be 1,2,3,4,5,6 (for each of the 6 legs) and M can 1,2,3. M=1 is the joint closest to the body and M=3 is the joint farthest from the body.

Available Robot API commands:

1. setMotorTargetJointPosition(motor_id_string, target_joint_angle) - send the target position for motor_id_string to CoppeliaSim
2. getSensorValue(sensor_type), where sensor_type can be from ['front', 'left', 'right'] - retrieves the current reading of the sensor sensor_type
3. getMotorCurrentJointPosition(motor_id_string) - retrieves the current angle for motor motor_id_string
4. getRobotWorldLocation() - returns (position, orientation) of the robot with respect to the world frame. Note, that orientation is represented as a quaternion.
5. getCurrentSimTime() - returns the current simulation time

Helper functions

degToRad() - Converts degrees to radians

Note that, this list of API functions could potentially grow. You will be notified via Canvas if anything is updated

'''

class HexapodControl(RobotControl):
    def __init__(self):
        super(HexapodControl, self).__init__(robot_type='hexapod')
        rospy.loginfo("hexapod setup complete")
        time.sleep(2.0)


        lowdif = 10000
        currturn = 0
        currSensVals = [self.getSensorValue('left'), self.getSensorValue('front'), self.getSensorValue('right')]
        print("Current sensors values.")
        print(currSensVals)
        csv_file_object = csv.reader(open("training_data_B.csv", 'r'), delimiter = ",")
        for row in csv_file_object:
            for x in range(0,4):
                row[x] = float(row[x])

            dif = self.difference(currSensVals, row)

            if dif < lowdif:
                lowdif = dif
                currturn = row[3]


        #main control loop
        while not rospy.is_shutdown(): 
            if currturn == 1:
                print("Turning left.")
                self.turnLeft90()
                return 

            if currturn == 2:
                print("Turning right.")
                self.turnRight90()
                return

            if currturn == 3:
                print("Turning around.")
                self.turnAround180()
                return 

            if currturn == 4:
                print("Not turning.")
                return
            
    def difference(self, x, y):
        return abs((x[0] - y[0])) + abs((x[1] - y[1])) + abs((x[2] - y[2]))

    #22.5 degree turn to the left
    def turnLeft22(self):
        self.setMotorTargetJointPosition('leg1_j2', -0.6)
        self.setMotorTargetJointPosition('leg3_j2', -0.6)
        self.setMotorTargetJointPosition('leg5_j2', -0.6)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg1_j1', 0.368)
        self.setMotorTargetJointPosition('leg3_j1', 0.368)
        self.setMotorTargetJointPosition('leg5_j1', 0.368)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j2', -0.6)
        self.setMotorTargetJointPosition('leg4_j2', -0.6)
        self.setMotorTargetJointPosition('leg6_j2', -0.6)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg1_j1', 0.0)
        self.setMotorTargetJointPosition('leg3_j1', 0.0)
        self.setMotorTargetJointPosition('leg5_j1', 0.0)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j1', 0.368)
        self.setMotorTargetJointPosition('leg4_j1', 0.368)
        self.setMotorTargetJointPosition('leg6_j1', 0.368)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j2', -0.5)
        self.setMotorTargetJointPosition('leg4_j2', -0.5)
        self.setMotorTargetJointPosition('leg6_j2', -0.5)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg1_j2', -0.6)
        self.setMotorTargetJointPosition('leg3_j2', -0.6)
        self.setMotorTargetJointPosition('leg5_j2', -0.6)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j1', 0.0)
        self.setMotorTargetJointPosition('leg4_j1', 0.0)
        self.setMotorTargetJointPosition('leg6_j1', 0.0)
        time.sleep(.25)

    #90 degree left turn
    def turnLeft90(self):
        self.turnLeft22()
        self.turnLeft22()
        self.turnLeft22()
        self.turnLeft22()

    #22.5 degree turn to the right
    def turnRight22(self):
        self.setMotorTargetJointPosition('leg1_j2', -0.6)
        self.setMotorTargetJointPosition('leg3_j2', -0.6)
        self.setMotorTargetJointPosition('leg5_j2', -0.6)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg1_j1', -0.368)
        self.setMotorTargetJointPosition('leg3_j1', -0.368)
        self.setMotorTargetJointPosition('leg5_j1', -0.368)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j2', -0.6)
        self.setMotorTargetJointPosition('leg4_j2', -0.6)
        self.setMotorTargetJointPosition('leg6_j2', -0.6)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg1_j1', 0.0)
        self.setMotorTargetJointPosition('leg3_j1', 0.0)
        self.setMotorTargetJointPosition('leg5_j1', 0.0)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j1', -0.368)
        self.setMotorTargetJointPosition('leg4_j1', -0.368)
        self.setMotorTargetJointPosition('leg6_j1', -0.368)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j2', -0.5)
        self.setMotorTargetJointPosition('leg4_j2', -0.5)
        self.setMotorTargetJointPosition('leg6_j2', -0.5)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg1_j2', -0.6)
        self.setMotorTargetJointPosition('leg3_j2', -0.6)
        self.setMotorTargetJointPosition('leg5_j2', -0.6)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j1', 0.0)
        self.setMotorTargetJointPosition('leg4_j1', 0.0)
        self.setMotorTargetJointPosition('leg6_j1', 0.0)
        time.sleep(.25)

    #90 degree turn to the right
    def turnRight90(self):
        self.turnRight22()
        self.turnRight22()
        self.turnRight22()
        self.turnRight22()

    #180 degree turn to the left
    def turnAround180(self):
        self.turnLeft90()
        self.turnLeft90()



if __name__ == "__main__":
    q = HexapodControl()
    rospy.spin()